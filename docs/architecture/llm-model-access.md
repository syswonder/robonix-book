# 模型目录与用量记录

本页说明一次模型调用如何选择端点、调用结束后如何记录用量。内容面向接入 `robonix_api.llm` 或排查模型连接问题的开发者；配置步骤见[开发者指南第 12 节](../developer-guide.md#12-部署清单)。

## 选择端点的顺序

Robonix 不代理模型请求。消费者取得端点后自行连接，模型目录只决定这次调用使用哪个端点。

选择端点时，显式配置优先于目录。读环境变量的消费者（Scene、memgraph、memsearch）如果设置了 `VLM_BASE_URL`，就直接使用自己的配置，不再读取模型目录。memgraph 的 LLM 检索还会读取 `MEMGRAPH_LLM_BASE_URL`。对这些消费者，只设置 `VLM_API_KEY` 或 `VLM_MODEL` 不算显式端点，目录仍然生效。Pilot 和 VLM Verifier 的规则见下表。

| 组件 | 显式端点 | 目录选择 | 不完整配置 |
|---|---|---|---|
| Pilot | `vlm.upstream`、`vlm.api_key`、`vlm.model` 齐全 | 未写 `upstream` 但写了 `model` 时按条目 ID 选择；两者都未写时按 `ROBONIX_LLM_FEATURES` 解析；另写的 `vlm.api_key` 替换条目的密钥 | 写了 `upstream` 但缺 `model` 或 `api_key` 时报错 |
| VLM Verifier | `vlm.base_url`、`vlm.api_key`、`vlm.model` 齐全 | 只写 `vlm.model` 时按 ID 选择；三项都未写（包括不写 `vlm` 块）时按 `vision` 解析 | 只写部分字段时报 `vlm.<field> is required`；`vlm.mode: jev` 只接受三项齐全的显式端点，不读取目录 |

下表列出仓库内置消费者写入记录的 `consumer` 和 `operation`，以及从目录解析时的要求。目录中没有满足某一行要求的条目时，`resolve_from_env` 抛出 `RuntimeError`，对应调用失败。例如目录缺少 `json_object` 条目时，memgraph 的检索会失败。

| 消费者 | `consumer` | `operation` | 所需特性 | `max_output_tokens` |
|---|---|---|---|---|
| Pilot | `pilot` | `plan`、`summary` | `ROBONIX_LLM_FEATURES`，默认 `vision,tools,stream` | `ROBONIX_LLM_MAX_OUTPUT_TOKENS`，默认 `0` |
| Scene 感知 | `scene` | `perception` | `vision` | `SCENE_VLM_MAX_TOKENS`，默认 `1024` |
| Scene 关系与物体描述 | `scene` | `relations`、`image_relations`、`caption` | `vision` | `0` |
| memgraph 检索 | `memory` | `search` | `json_object` | `512` |
| memgraph 观察与问答 | `memory` | `observe`、`answer` | `vision` | `256` |
| memsearch | `memsearch` | `compact` | 无 | `1024` |
| VLM Verifier | `vlm_verifier` | `judge` | `vision` | `0` |

启用目录后，启动器会收窄组件的环境变量。Pilot 由 `rbnx boot` 清空环境后启动，Python 消费者经 `python -m robonix_api.llm` 启动，两者只保留白名单中的变量。白名单包括 `robonix_api/llm/runtime_env.txt` 列出的运行时变量、目录中全部 `api_key_env`，以及与非空端点成对的 `VLM_API_KEY` 和 `MEMGRAPH_LLM_API_KEY`。`OPENAI_*` 和其他未列出的变量不会传入组件。

Python 侧会按路径缓存目录，最多 8 个，同一路径在一个进程内只加载一次；Rust 的 `resolve_from_env` 每次调用都会重新读取。修改目录后需要重启消费者。

## 模型目录文件

模型目录是 YAML 或 JSON 文件。顶层是对象，`models` 必须是非空列表；其他顶层键会被忽略。每个条目是一个可以连接的模型端点：

```yaml
models:
  - id: local/vision
    provider: vllm
    base_url: http://127.0.0.1:8000/v1
    model: vision
    features: {vision: true, tools: true, stream: true, json_object: true}
```

| 字段 | 必填 | 默认值 | 说明 |
|---|---|---|---|
| `id` | 是 | — | 目录内唯一的条目 ID；只写 `vlm.model` 时用它选择条目 |
| `provider` | 是 | — | 模型厂商标识，供运维识别，不决定协议 |
| `base_url` | 是 | — | OpenAI 兼容的接口地址（base URL）；末尾斜杠会被去掉，不要在这里写密钥 |
| `model` | 是 | — | 请求中发送的模型名 |
| `api_key_env` | 否 | 无 | 环境变量名；省略表示端点无需密钥（通常是本地服务）。设置后必须匹配 `[A-Za-z_][A-Za-z0-9_]*` |
| `protocol` | 否 | `openai-chat` | 仓库内置消费者只实现 `openai-chat`；目录解析不按它过滤 |
| `features` | 否 | `{}` | 布尔特性表，按 `Requirements.features` 匹配；未列出的特性按 `false` 处理 |
| `context` | 否 | 无 | 上下文窗口（context），单位 token；解析时要求 `input_tokens + max_output_tokens` 不超过它，加载时只校验它是正整数。Pilot 未设置 `vlm.context_window_tokens` 时，以它作为上下文窗口 |
| `latency_p95_ms` | 否 | 无 | 非负的延迟参考值（毫秒），只用于排序；缺省时排在最后 |
| `price_input_per_mtok` | 否 | `0` | 非负的输入价格参考值，单位是每百万 token；只用于排序 |
| `price_output_per_mtok` | 否 | `0` | 非负的输出价格参考值，单位是每百万 token；只用于排序 |

Python 侧加载时，`.json` 文件由标准库解析，其他后缀交给 PyYAML。文件读不到抛 `OSError`，YAML 语法错误抛解析错误；顶层不是对象、`models` 不是非空列表、条目不是对象、`id` 重复、必填字符串缺失或为空白、`api_key_env` 格式不对、价格或延迟为负或不是有限数、`context` 不是正整数，都会抛出 `ValueError`。Rust 侧通过 serde 反序列化，错误类型不同。

## 解析规则

`ModelCatalog.resolve(requirements)` 先筛掉不支持所需特性的条目，再检查上下文窗口。条目的 `context` 已设置时，要求 `input_tokens + max_output_tokens` 不超过它；`context` 未设置，或者 `input_tokens` 与 `max_output_tokens` 都为 0，就跳过这项检查。

剩下的条目按 `latency_p95_ms` 升序排列，没有该字段的排在最后；延迟相同再比较输入价格与输出价格之和；还相同就按 `id` 排序。返回值是按优先级排序的全部候选；`resolve_from_env` 只取第一个可用条目，请求失败不会自动切换到下一个。

`resolve_from_env` 在此基础上处理部署配置：

| 情况 | 结果 |
|---|---|
| `ROBONIX_LLM_CATALOG` 未设置或为空白 | 返回 `None`，调用方继续使用原有的 `VLM_*` 或清单配置 |
| 目录无法读取，或内容不合法 | 抛出加载错误，例如 `OSError`、`ValueError` 或 YAML 解析错误 |
| 没有条目满足要求 | 抛出 `RuntimeError`，错误信息会列出要求的特性 |
| 有匹配条目，但候选条目的 `api_key_env` 变量都未设置 | 抛出 `RuntimeError`，错误信息会列出需要设置的变量 |
| 条目无需密钥，或它的 `api_key_env` 变量已设置 | 返回第一个可用条目 |

## Python 接口

下面的签名来自 `robonix_api.llm` 源码。该模块尚未纳入固定修订的 Python 公共 API 页，字段和示例以本页及源码 `pylib/robonix-api/robonix_api/llm/README.md` 为准。

| 符号 | 签名 | 返回值 | 异常 |
|---|---|---|---|
| `ModelCatalog.load` | `load(path: str \| Path)` | `ModelCatalog` | `OSError`、`ValueError`、YAML 解析错误 |
| `ModelCatalog.from_dict` | `from_dict(data: dict)` | `ModelCatalog` | `ValueError` |
| `ModelCatalog.model` | `model(model_id: str)` | `ModelSpec` | `ValueError`（未知 ID） |
| `ModelCatalog.resolve` | `resolve(requirements: Requirements)` | `list[ModelSpec]` | — |
| `Requirements` | `(features=(), input_tokens=0, max_output_tokens=0)` | 不可变数据类（dataclass） | — |
| `Requirements.default` | `default()` | `Requirements()` | — |
| `catalog_from_env` | `catalog_from_env()` | `ModelCatalog \| None` | 加载错误 |
| `resolve_from_env` | `resolve_from_env(requirements, *, env="ROBONIX_LLM_CATALOG")` | `ModelSpec \| None` | `RuntimeError`、加载错误 |
| `api_key_for` | `api_key_for(target: ModelSpec)` | 字符串；无需密钥或变量未设置时为空字符串 | — |
| `explicit_env_set` | `explicit_env_set(*names: str)` | `bool` | — |

`ModelSpec` 和 `Requirements` 都是不可变数据类（dataclass）。`ModelSpec` 的字段包括 `id`、`provider`、`base_url`、`model`、`api_key_env`、`features`、`context`、`latency_p95_ms`、`price_input_per_mtok`、`price_output_per_mtok` 和 `protocol`。

## 模型用量记录

记录写到 `$ROBONIX_LLM_METRICS_DIR/<consumer>-<pid>.jsonl`。变量未设置时不创建文件，第一次调用记录接口会在组件日志里打印 `off`。记录只写本地文件，不经过网络，也不包含提示词、回复或密钥。

一条成功的记录格式化后类似下面这样；JSONL 文件里一行一个这样的对象（字段顺序不固定；未上报的计数为 `null`）：

```json
{
  "v": 1,
  "consumer": "pilot",
  "operation": "plan",
  "host": "api.deepseek.com",
  "model": "deepseek-chat",
  "stream": true,
  "attempts": 1,
  "status": "ok",
  "http_status": 200,
  "provider_model": "deepseek-chat",
  "input_tokens": 1820,
  "output_tokens": 240,
  "cached_input_tokens": 1024,
  "reasoning_tokens": null,
  "latency_ms": 4200,
  "ts_ms": 1770000000000,
  "boot_id": "3f2c9a1e-7b4d-4c8e-9a51-2d6f0b8e4c17",
  "pid": 100
}
```

调用失败时记录结构相同，`status` 和 `http_status` 换成对应的值；`http_status` 在 `timeout`、`cancelled` 或 `transport_error` 时可能为 `null`。

`metrics.Call` 用于包裹一次 HTTP 请求：

| 成员 | 输入 | 作用 |
|---|---|---|
| `Call(consumer, operation, base_url, model, *, stream=False)` | 消费者、操作和端点信息 | 校验 `consumer` 是否匹配 `[a-z0-9_-]+`，不匹配时抛 `ValueError`；同时初始化 `v`、`host`、`stream` 和 `attempts=1` |
| `__enter__` | — | 记录调用开始时间 |
| `retry()` | — | 把 `attempts` 加一，不新增记录 |
| `finish(response=None)` | 带 `status_code` 和 `json()` 的对象 | 写入状态、HTTP 状态码、响应中的模型名和 token 数；错误响应也必须调用。未传 `response` 时按成功处理，token 记为 `null` |
| `__exit__` | — | 未调用 `finish` 时，根据异常记为 `timeout`、`cancelled` 或 `transport_error`，补上 `latency_ms` 后追加记录，并把异常继续向上抛出 |

`metrics.write(record)` 是模块级函数，不属于 `Call`。它把一条记录追加到 `<consumer>-<pid>.jsonl`，用一次 `os.write` 完成写入；写入失败只记录一次警告（进程内），不影响模型调用。

| 状态 | 出现条件 |
|---|---|
| `ok` | HTTP 2xx 且响应体是 JSON；或者调用 `finish` 时未传 response |
| `http_error` | HTTP 非 2xx |
| `transport_error` | HTTP 2xx 但响应体无法读取，或者出现超时和取消之外的异常；Rust 侧（Pilot）的流式响应被截断时也记为该状态 |
| `timeout` | 异常名称包含 `Timeout` |
| `cancelled` | 抛出 `asyncio.CancelledError` |

同一进程内的多个线程通过模块级锁串行写入；不同进程的文件名带 PID，互不影响。读取方可能读到尚未写完的最后一行，汇总时会跳过损坏的行并给出提示。

### 记录字段

| 字段 | 含义 |
|---|---|
| `v` | 记录格式版本，当前是 1 |
| `ts_ms`、`latency_ms` | 调用结束时间和耗时 |
| `boot_id`、`pid` | 本次部署的 `RBNX_BOOT_ID` 和写入进程的 PID；`boot_id` 缺失时为 `null` |
| `consumer`、`operation` | 消费者和操作名 |
| `host` | 端点的 host 和端口，不含路径、查询参数或凭据 |
| `model`、`provider_model` | 请求的模型名，以及响应里声明的模型名 |
| `stream` | 是否流式调用 |
| `status`、`http_status` | 上面的状态和 HTTP 状态码 |
| `attempts` | 本次调用发出的请求总数，包含重试 |
| `input_tokens`、`output_tokens`、`cached_input_tokens`、`reasoning_tokens` | 模型服务上报的计数；未上报时为 `null` |

## 用 rbnx llm-usage 汇总

在部署目录运行 `rbnx llm-usage`。不加 `-d` 时，先读 `ROBONIX_LLM_METRICS_DIR`；变量未设置或为空则读当前目录的 `rbnx-boot/llm-metrics/`，也就是 `<deploy>/rbnx-boot/llm-metrics/`。`--boot` 只统计指定的启动。`BOOT_ID` 可从 `rbnx-boot/state.json` 读取。`rbnx boot` 退出、`rbnx shutdown` 和 `rbnx clean -f` 都会删除该文件，因此只能在部署运行期间读取；停机后直接使用记录里的 `boot_id`。命令参数见[开发者指南第 15 节：常用命令行接口](../developer-guide.md#15-常用命令行接口)或 [rbnx 命令行](../components/rbnx.md)。

`--json` 每行输出一个分组：

| 字段 | 含义 |
|---|---|
| `consumer`、`operation`、`host`、`model` | 分组键 |
| `calls` | 记录数，也就是模型调用次数 |
| `attempts` | 请求数，包含重试 |
| `status` | 每个状态对应的调用数 |
| `input_tokens`、`output_tokens`、`cached_input_tokens`、`reasoning_tokens` | 已上报计数的合计 |
| `unreported` | 缺少输入或输出计数（任一项）的调用数 |
| `latency_samples` | 有延迟样本的调用数 |
| `latency_p50_ms`、`latency_p95_ms`、`latency_max_ms` | 最近秩（nearest-rank）分位数和最大值；没有样本时为 `null` |

损坏的行或文件会被跳过，并在 stderr 输出提示，其余记录照常汇总。记录中不含价格；计算成本时，用 token 合计按 `host` 和 `model` 对照目录中的 `price_*` 字段或运维价目表。

## Rust 实现

Rust 侧的实现在 `system/llm-catalog` 和 `system/llm-metrics`，解析规则与 Python 相同，两边共用 `pylib/robonix-api/robonix_api/llm/contract/` 下的 JSON 用例。Rust 的 `Call` 提供 `start`、`retry`、`observe` 和 `finish`，状态用 `Status` 枚举（`Ok`、`HttpError`、`Timeout`、`Cancelled`、`TransportError`）。字段与 Python 记录相同；有记录目录时，`Drop` 前没有 `finish` 的记录一律记为 `Cancelled`，Python 则按异常区分后三种状态。
