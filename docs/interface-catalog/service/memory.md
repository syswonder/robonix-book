<span id="记忆-robonixservicememory"></span>
# 记忆

记忆服务是长期记忆层：语义检索、写入、压缩归纳。`search` / `save` 的请求和响应使用 `std_msgs/String`；`compact` 接收空请求，返回 `std_msgs/String` 摘要。embedding、向量库和检索数量属于实现细节，不进能力约定。该服务没有 `driver` 能力约定。

能力约定 TOML 在 `capabilities/service/memory/`；直接 IDL 位于 `capabilities/lib/memory/`，字符串载荷复用 `std_msgs/String`。

## 接口

| 能力约定 ID | 模式 | 默认实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/memory/search` | `rpc` | MCP | [`memory/Search`](../../reference/idl.md#memory-srv-search-srv)（`std_msgs/String` → `std_msgs/String`） | `service/memory/search.v1.toml` |
| `robonix/service/memory/save` | `rpc` | MCP | [`memory/Save`](../../reference/idl.md#memory-srv-save-srv)（`std_msgs/String` → `std_msgs/String`） | `service/memory/save.v1.toml` |
| `robonix/service/memory/compact` | `rpc` | MCP | [`memory/Compact`](../../reference/idl.md#memory-srv-compact-srv)（空请求 → `std_msgs/String`） | `service/memory/compact.v1.toml` |

参考实现：Robonix [`8c2551ce`](https://github.com/syswonder/robonix/tree/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/memsearch) 中的 `services/memsearch`（`memsearch[onnx]` + `milvus-lite`）。三条能力约定都用 `@memory.mcp(...)` 暴露，不挂载 gRPC servicer；工具名默认取能力约定 leaf：`search`、`save`、`compact`。MCP 线格式见 [接口目录首页](../index.md)。

默认实现固定使用 ONNX embedding，`search` 固定取 `top_k=2`；部署可通过 `AGENT_MEMORY_DIR` 指定 Markdown 记忆目录，通过 `AGENT_MILVUS_URI` 指定 Milvus Lite 数据库。`compact` 还需要 `VLM_API_KEY` 或 `OPENAI_API_KEY`；端点和模型分别读取 `VLM_BASE_URL` / `VLM_MODEL`，并回退到对应的 `OPENAI_*` 变量。缺少密钥时，工具会返回不可用说明，不执行归纳。

> 实现依据：[package manifest](https://github.com/syswonder/robonix/blob/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/memsearch/package_manifest.yaml) · [MCP 注册与运行配置](https://github.com/syswonder/robonix/blob/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/memsearch/memsearch_service/service.py)
