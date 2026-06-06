# 记忆 robonix/service/memory

记忆服务是长期记忆层：语义检索、写入、压缩归纳。载荷都是 `std_msgs/String`（检索 query、写入文本、归纳指令），底层 embedding / 向量库 / `top_k` 等是部署侧元数据，不进契约。该服务没有 `driver` 契约。

契约 TOML 在 `capabilities/service/memory/`，IDL 在 `capabilities/lib/memory/`。

## 接口

| 契约 ID | 模式 | 载荷（IDL） | 契约 TOML |
|---|---|---|---|
| `robonix/service/memory/search` | `rpc` | `memory/Search`（`std_msgs/String` → `std_msgs/String`） | `service/memory/search.v1.toml` |
| `robonix/service/memory/save` | `rpc` | `memory/Save` | `service/memory/save.v1.toml` |
| `robonix/service/memory/compact` | `rpc` | `memory/Compact` | `service/memory/compact.v1.toml` |

参考实现：`services/memsearch`（`memsearch[onnx]` + `milvus-lite`），三条契约都用 `@memory.mcp(...)` 以 MCP 工具暴露（工具名默认取契约 leaf：`search` / `save` / `compact`），Agent 可直接调。MCP 线格式见 [接口目录首页](../index.md)。
