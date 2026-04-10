# Memory Search 服务

长期记忆服务，支持向量检索、写入与压缩归纳。有两种接入形态，可单独使用或并存：

- **MCP 形态**（推荐，Agent 直接调用）：以 MCP 工具暴露 `search` / `save` / `compact`，无需 `NegotiateChannel`。
- **gRPC 形态**（系统内部或非 MCP 消费者）：一元 RPC，载荷均为 `std_msgs/String`。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/srv/memory/search` | `rpc` | `std_msgs/String` → `std_msgs/String` | `sys/memory_search.v1.toml` |
| `robonix/srv/memory/save` | `rpc` | `std_msgs/String` → `std_msgs/String` | `sys/memory_save.v1.toml` |
| `robonix/srv/memory/compact` | `rpc` | `std_msgs/String` → `std_msgs/String` | `sys/memory_compact.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。索引参数、embedding 模型、`top_k` 等为部署侧元数据，不在契约内。

## MCP 形态（示例包 `memsearch_service`）

示例包基于 `memsearch[onnx]` + `milvus-lite`，节点 ID `com.robonix.services.memsearch`。三个工具对应契约 ID：

| MCP 工具名 | 契约 ID |
|------------|---------|
| `search_memory` | `robonix/srv/memory/search` |
| `save_memory` | `robonix/srv/memory/save` |
| `compact_memory` | `robonix/srv/memory/compact` |

载荷用 `robonix-codegen --lang mcp` 生成的 `std_msgs_mcp.String`（线格式 `{"data": "<UTF-8 文本>"}`），通过 `@mcp_contract(mcp, contract_id=...)` 注册，与 `to_dict()` 一致。

启动示例：`START_MEMSEARCH=1 ./examples/run.sh`。

## 注册

gRPC 形态：`contract_id` 填对应契约 ID（`search` / `save` / `compact`）。MCP 形态须遵守 [接口目录首页](../index.md) 中的 MCP 线格式与 `mcp_contract` 约定。
