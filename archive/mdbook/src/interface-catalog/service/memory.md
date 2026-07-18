# 记忆 robonix/service/memory

记忆服务是长期记忆层：语义检索、写入、压缩归纳。载荷都是 `std_msgs/String`（检索 query、写入文本、归纳指令），底层 embedding / 向量库 / `top_k` 等是部署侧元数据，不进能力约定。该服务没有 `driver` 能力约定。

能力约定 TOML 在 `capabilities/service/memory/`，IDL 在 `capabilities/lib/memory/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/service/memory/search` | `rpc` | [`memory/Search`](../../reference/idl.md#memory-srv-search-srv)（`std_msgs/String` → `std_msgs/String`） | `service/memory/search.v1.toml` |
| `robonix/service/memory/save` | `rpc` | [`memory/Save`](../../reference/idl.md#memory-srv-save-srv) | `service/memory/save.v1.toml` |
| `robonix/service/memory/compact` | `rpc` | [`memory/Compact`](../../reference/idl.md#memory-srv-compact-srv) | `service/memory/compact.v1.toml` |

参考实现：`services/memsearch`（`memsearch[onnx]` + `milvus-lite`），三条能力约定都用 `@memory.mcp(...)` 以 MCP 工具暴露（工具名默认取能力约定 leaf：`search` / `save` / `compact`），Agent 可直接调。MCP 线格式见 [接口目录首页](../index.md)。
