# `model` Transport（草稿设计 / TODO）

> 本页是一份**草稿设计**，尚未实现。收集在这里是为了把思路固化下来，便于讨论和后续落地。

## 动机

当前 `robonix/srv/cognition/reason` 走 gRPC：Pilot → `vlm_service` (Python adapter) → HTTP → 上游 LLM。这有几个问题：

1. **多一层跳**。模型本身就是 HTTP + JSON，gRPC adapter 除了转协议什么都没干，纯开销。图像/长上下文尤其明显。
2. **flavor 绑死了 OpenAI 协议族**。要接 Claude 原生 `/v1/messages` 或 Gemini 原生格式就得在 `vlm_service` 里加分支，契约本身帮不上忙。
3. **多模型路由没地方声明**。Atlas 里如果有三个 `reason` provider（本地 7B、远端 GPT、远端 Claude），Pilot 目前只能选第一个——没有 capability / cost / latency 元数据。
4. **LLM serving 框架原生是 HTTP**（vLLM、SGLang、LM Studio、Ollama、LiteLLM、TGI ……），每加一个就得写一次 Python gRPC adapter，生态不复用。

## 设计 sketch

引入第四种 data-plane transport：`model`。

| 已有 | 新增 |
|---|---|
| gRPC、ROS 2、MCP、shared_memory | **`model`**（HTTP + 特定 flavor 的 JSON） |

### Provider 注册

```yaml
# LLM serving 节点（或云端 API 代理）在启动时声明：
DeclareInterface:
  contract_id: "robonix/srv/cognition/reason"
  supported_transports: ["model"]
  metadata_json:
    flavor:       "openai"          # openai | anthropic | gemini | ollama
    url:          "http://127.0.0.1:8000/v1"
    model:        "Qwen2.5-VL-7B-Instruct"
    auth:         { type: "bearer_env", var: "QWEN_API_KEY" }   # 或 "none"
    capabilities: ["chat", "tools", "vision"]
    context_len:  128000
    max_output:   4096
    cost:         { prompt: 0.0, completion: 0.0, unit: "USD/1M" }  # 可选
    latency_ms:   { p50: 800, p99: 3500 }                            # 可选
```

Atlas 对 `model` transport 不强制 contract id 校验（任何人都可以注册自己的 `robonix/srv/cognition/*`），但至少要验 `metadata_json` 必填字段。

### Pilot 消费流程

```
Pilot.run_turn(intent):
    # 1. 候选发现
    providers = atlas.QueryNodes(contract_id="robonix/srv/cognition/reason",
                                  transport="model")

    # 2. Capability 过滤
    if intent.has_images:
        providers = [p for p in providers if "vision" in p.metadata.capabilities]

    # 3. 策略选一个（后续可插入真正的 router）
    chosen = pick_by_policy(providers, policy="lowest_latency")
        # e.g. "round_robin" / "lowest_cost" / "lowest_latency" / "explicit_pref"

    # 4. 按 flavor 选 adapter，直连
    match chosen.metadata.flavor:
        "openai"    -> openai.OpenAI(base_url=..., api_key=...)
        "anthropic" -> anthropic.Anthropic(base_url=..., api_key=...)
        "gemini"    -> google.genai.Client(...)
        "ollama"    -> ollama.Client(host=...)

    # 5. 流式响应直接发回给 PilotEvent stream
```

### 路由策略（Phase 1 最简）

先做硬编码策略：

- `explicit`：manifest 里指定 `preferred_model_id`
- `round_robin`：多个同等能力的 provider 轮询
- `cheapest_first` / `fastest_first`：按 metadata 排序

后续拓展：语义路由（小模型先看，难题才走大模型）、A/B、fallback chain。

## 不建 contract TOML

model transport **不在 `rust/contracts/` 下定义 contract 文件**。理由：

- OpenAI / Anthropic / Gemini / Ollama 的 API schema 已经是事实标准 + 公开文档 + 官方 SDK（`openai`、`anthropic`、`google-genai`、`ollama`）直接可用。Robonix 再包一层 contract 是纯负担：我们的 TOML 能写的东西上游 OpenAPI spec 都写了，手写 adapter 还得追着上游版本改。
- contract 的存在意义是"把不同实现黏到同一个 shape"。这里的 shape 已经被业界统一了，不需要我们再定义。
- 真正要约定的是 **`metadata_json` 的字段形状** + **flavor 枚举值**，这两个是**运行时配置**，不是数据面 IDL。

所以 model transport 的"契约"就是**本页文档** + 上游官方 API spec。

### metadata_json 字段规范

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `flavor` | enum | ✅ | `openai` \| `anthropic` \| `gemini` \| `ollama`，决定 Pilot 用哪个 SDK |
| `url` | string | ✅ | HTTP endpoint base URL |
| `model` | string | ✅ | 模型 ID（上游命名，如 `gpt-5`、`claude-opus-4-6`、`Qwen2.5-VL-7B`） |
| `auth` | object | ✅ | `{type: "bearer_env", var: "OPENAI_API_KEY"}` 或 `{type: "none"}` |
| `capabilities` | string[] | ✅ | 从 `["chat", "tools", "vision", "embed", "rerank", "tts", "asr"]` 取子集 |
| `context_len` | int | 建议 | 最大上下文 token 数，供 Pilot 做 history 裁剪 |
| `max_output` | int | 建议 | 最大输出 token |
| `cost` | object | 可选 | `{prompt: 0.0, completion: 0.0, unit: "USD/1M"}`，给 cost-based router 用 |
| `latency_ms` | object | 可选 | `{p50, p99}`，给 latency-based router 用 |

同一个节点可以声明多条 `transport=model` interface（不同 model / 不同 capability），Pilot 按 capability 过滤候选。

### flavor vs capability

- **flavor** = 线协议方言（决定用哪个 SDK）
- **capability** = 能力标签（决定这个 provider 能干什么）

二者正交。举例：一个 flavor=`openai` 的 provider 可以同时有 `capabilities: ["chat", "embed"]`——因为 OpenAI 本身就既有 `/v1/chat/completions` 也有 `/v1/embeddings`。Pilot 查询时按 capability 过滤，拿到 metadata 后按 flavor 选 SDK 调不同路径。

### 开放问题

1. **MCP vs model**：MCP tools 本身也是 HTTP，但语义是"工具调用"而不是"模型推理"。保持分开——MCP 走 `transport=mcp`，model 走 `transport=model`。
2. **`reason` 契约的定位**：现有 `srv/cognition/reason` 是 gRPC chat 契约，语义上和 model transport 重叠。长期方向是把 `reason` 重新定义为**高层认知服务**（内部组装语义地图 / 记忆 / RTDL prompt → 调 model transport），让 `reason` 消费 model，而不是和 model 并列。具体分层见下方"与 `cognition/reason` 的关系"。

## 与 `cognition/reason` 的关系：reason 契约废弃

早期设计里有一个 `srv/cognition/reason` gRPC 契约，意图把"认知推理"做成独立服务。结合 model transport 的引入重新审视，**这个契约应该废弃**：

- `reason` 原本的职责（接 chat 请求、调上游模型）完全被 model transport 覆盖——HTTP + flavor SDK 直连，不需要再包一层 gRPC。
- 如果把 `reason` 重新定义为"富 context 推理服务"（内部组装 memory / map / prompt 再调 model），那它**只有 Pilot 一个消费者**，拆成独立服务是过早抽象：多一跳 RPC、多一个部署单元、多一道故障面，换不到可复用性。
- Pilot 本身就是 ReAct 循环的宿主，prompt engineering、context 组装、memory retrieval 是认知循环的**内部步骤**，不是跨进程服务。留在 Pilot 里逻辑反而聚拢。

### 最终分层（两层，不是三层）

```
Liaison ──Intent──► Pilot ──tool_calls──► Executor ──► {MCP, gRPC, Built-in}
                      │
                      │ (Pilot 内部模块)
                      ├── context assembler   : 查 srv/memory/* + srv/common/map/*
                      ├── prompt builder       : 组装 system / tools / history
                      ├── model router         : 查 Atlas transport=model 候选 + 策略选一个
                      └── http client          : 按 flavor 调用 openai/anthropic/gemini/ollama SDK
                                 │
                                 └──HTTP──► 任意 model provider
                                             (本地 vllm / 云端 openrouter / 自建 serving)
```

**Pilot** 管认知循环 + 模型调用；**model provider** 管 LLM 原语 serving。中间没有第三方。

### `vlm_service` 的退路

现在的 `vlm_service`（gRPC→HTTP 转协议）按本设计**整体退役**。同一个功能由"Pilot 直连 model provider"完成，不再需要这一层。`vlm_service` 代码可以保留一段时间作为迁移期的兼容 shim，但不是长期路径。

### 什么时候再考虑把 reason 拆出来

只在以下场景出现时重启"独立 reason 服务"的讨论：

- 出现 Pilot 以外的第二个消费者（例如独立 Agent、Executor 自主决策）
- reason 实现真的需要热插拔（VLM / 规则引擎 / BT 生成器并存），且切换逻辑复杂到不适合塞进 Pilot
- 团队规模扩大到需要把"认知"和"状态机"拆给不同人维护

在这些需求没落地前，单体 Pilot 更健康。

## 实现任务拆解

见 [Roadmap · model transport](../roadmap.md#model-transport-草稿设计)。

## 替代方案 —— 为什么不直接在 `vlm_service` 里加 flavor switch

短期方便，长期有两个硬伤：

- **gRPC 单点**：所有流量经过 `vlm_service`，它成为瓶颈和故障点。
- **元数据丢失**：Atlas 看不到"这个模型是什么、能干嘛"，多模型编排无从谈起。

所以结论是：**`vlm_service` 保留作为"把 OpenAI-only 模型伪装成 model transport provider"的便利包装**，但不再是强制路径。

## 兼容策略

- 现有 gRPC 版 `reason` 契约继续可用（`supported_transports: ["grpc"]`），`vlm_service` 不需要马上弃用。
- 同一个契约 ID 可以同时存在两种 transport 的 provider，Pilot 根据自身支持决定走哪种。
- 迁移期：Pilot 先发现 `model` transport provider；找不到才回退到 gRPC。
