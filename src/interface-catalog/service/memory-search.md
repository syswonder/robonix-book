# Memory Search 服务（gRPC 契约）

对齐 **`rust/contracts/sys/memory_search.v1.toml`**：一元 **字符串** 请求/响应（`std_msgs/msg/String`），用于检索类能力在 gRPC/IDL 路径上的统一形状。与 **MCP 形态的 Memsearch 包**可并存（见下文）。

## 契约（与 `memory_search.v1.toml` 同步）

| 项 | 值 |
|----|-----|
| **契约 ID（`contract_id`）** | `robonix/sys/memory/search` |
| **版本** | `1` |
| **`kind`** | `service` |
| **`[io].input`** | `std_msgs/msg/String` |
| **`[io].output`** | `std_msgs/msg/String` |
| **`[mode].type`** | `rpc` |
| **`[semantics]`** | `persistent = true`，`searchable = true`；`wire_profile = "mcp_or_future_idl"` |

TOML 注明：索引参数、embedding 模型、`top_k` 等为**部署侧**元数据，非契约字段。

## MCP 形态的 Memsearch（`robonix/sys/memory/memsearch`）

长期记忆示例服务基于 **`memsearch[onnx]`**、`milvus-lite`，节点如 **`com.robonix.services.memsearch`**，通过 **MCP** 暴露工具（**无**单独 `rust/contracts` 文件，与控制面目录中 MCP 路径可共存）。

常用工具：

1. **`search_memory(query)`** — 向量检索记忆  
2. **`save_memory(content)`** — 写入并索引  
3. **`compact_memory()`** — 归纳压缩  

启动示例：`START_MEMSEARCH=1 ./examples/run.sh`；关闭：`START_MEMSEARCH=0`。数据目录示例：**`rust/examples/packages/memsearch_service/agent_milvus.db`**、**`agent_memory/`**。

## 注册

gRPC 形态声明时使用 **`contract_id="robonix/sys/memory/search"`**；MCP 形态受 MCP 规则约束，不限于系统接口目录。
