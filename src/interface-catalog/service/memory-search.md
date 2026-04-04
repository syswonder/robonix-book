# Memory Search 服务（gRPC 契约）

对齐 **`rust/contracts/sys/memory_search.v1.toml`**：一元 RPC，**`[io.srv]`** 绑定 **`lib/memory/srv/Search.srv`**（`query` / `results` 为 `std_msgs/String`）。与 **MCP 形态的 Memsearch 包**可并存（见下文）。

## 契约（与 `memory_search.v1.toml` 同步）

| 项 | 值 |
|----|-----|
| **契约 ID（`contract_id`）** | `robonix/sys/memory/search` |
| **版本** | `1` |
| **`kind`** | `service` |
| **`[io.srv]`** | `srv = "memory/srv/Search"` |
| **`[mode].type`** | `rpc` |
| **`[semantics]`** | `persistent = true`，`searchable = true`；`wire_profile = "mcp_or_future_idl"` |

TOML 注明：索引参数、embedding 模型、`top_k` 等为**部署侧**元数据，非契约字段。

## 相关契约：`save` / `compact`

| 契约 ID | 契约源码 |
|---------|----------|
| `robonix/sys/memory/save` | `rust/contracts/sys/memory_save.v1.toml` |
| `robonix/sys/memory/compact` | `rust/contracts/sys/memory_compact.v1.toml` |

二者 **`[io]`** 亦为 **`std_msgs/msg/String`**（与 **`search`** 相同线形状），具体语义见各 TOML 注释。

## MCP 形态的 Memsearch（示例包）

长期记忆示例服务基于 **`memsearch[onnx]`**、`milvus-lite`，节点如 **`com.robonix.services.memsearch`**，通过 **MCP** 暴露工具。**`search` / `save` / `compact`** 均在 **`rust/contracts/sys/`** 有对应 **`*.v1.toml`**（`memory_search`、`memory_save`、`memory_compact`），实现上应对齐 **`std_msgs/msg/String`**：使用 **`robonix-codegen --lang mcp`** 生成的 **`String` dataclass**，并用 **`@mcp_contract(..., input_cls=String, output_cls=String)`** 注册；MCP **`arguments`** 线格式为 **`{"data": "<UTF-8 文本>"}`**（与 `to_dict()` 一致），**不要**与裸 `@mcp.tool()` 的 Python 参数名混用。

常用工具（与契约 ID 对应，见包内 `DeclareInterface`）：

1. **`search_memory`** — `robonix/sys/memory/search`，向量检索记忆  
2. **`save_memory`** — `robonix/sys/memory/save`，写入并索引  
3. **`compact_memory`** — `robonix/sys/memory/compact`，归纳压缩  

启动示例：`START_MEMSEARCH=1 ./examples/run.sh`；关闭：`START_MEMSEARCH=0`。数据目录示例：**`rust/examples/packages/memsearch_service/agent_milvus.db`**、**`agent_memory/`**。

## 注册

gRPC 形态声明时使用 **`contract_id="robonix/sys/memory/search"`**（或 **`save` / `compact`** 对应契约 ID）；MCP 形态须遵守 [接口目录首页](../index.md) 中的 **MCP 线格式**与 **`mcp_contract`** 约定。
