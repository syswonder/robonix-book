---
title: 记忆
---
<span id="记忆-robonixservicememory"></span>
# 记忆

记忆服务是长期记忆层：语义检索、写入和压缩归纳。`search` / `save` 的请求和响应使用 `std_msgs/String`；`compact` 接收空请求，返回 `std_msgs/String`。embedding、向量库和检索数量属于实现细节，不进能力约定。

能力约定 TOML 在 `capabilities/service/memory/`；直接使用的接口定义语言（Interface Definition Language，IDL）文件位于 `capabilities/lib/memory/`，字符串载荷复用 `std_msgs/String`。

## 接口

| 能力约定 ID | 模式 | 默认实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/memory/driver` | `rpc` | gRPC（生命周期） | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `service/memory/driver.v1.toml` |
| `robonix/service/memory/search` | `rpc` | 模型上下文协议（Model Context Protocol，MCP） | [`memory/Search`](../../reference/idl.md#memory-srv-search-srv)（`std_msgs/String` → `std_msgs/String`） | `service/memory/search.v1.toml` |
| `robonix/service/memory/save` | `rpc` | MCP | [`memory/Save`](../../reference/idl.md#memory-srv-save-srv)（`std_msgs/String` → `std_msgs/String`） | `service/memory/save.v1.toml` |
| `robonix/service/memory/compact` | `rpc` | MCP | [`memory/Compact`](../../reference/idl.md#memory-srv-compact-srv)（空请求 → `std_msgs/String`） | `service/memory/compact.v1.toml` |

参考实现：Robonix 源码中的 [`services/memsearch`](https://github.com/syswonder/robonix/tree/436204a0aa5301dcf682a38ad29523023ad5a3a5/services/memsearch)（`memsearch[onnx]` + `milvus-lite`）。三个记忆操作都用 `@memory.mcp(...)` 暴露，不挂载业务 gRPC servicer；工具名默认取能力约定 leaf：`search`、`save`、`compact`。`driver` 由 Robonix API 以 gRPC 提供。

## 生命周期与运行行为

提供方注册后等待 `Driver(CMD_INIT)`。参考实现在 `on_init` 中解析实例配置、创建存储目录、配置 ONNX Runtime、构造 MemSearch 后端，并为 Markdown 语料建立初始索引。后端构造或首次索引失败时，`CMD_INIT` 返回错误，提供方进入 `ERROR`，不会继续进入 `ACTIVE`。相同配置的重复初始化是幂等操作；已初始化后再传入不同配置会返回错误。

初始化成功后，启动器发送 `CMD_ACTIVATE` 进入 `ACTIVE`。参考实现没有自定义激活或去激活处理，这两个命令使用框架的默认状态转换；关闭时由框架停止服务端。业务工具不会在后台补做初始化：未完成 `CMD_INIT` 时，后端访问会失败。

运行期间，`search` 在后端异常时返回“检索不可用”结果；`save` 先把内容追加到当日的 `YYYY-MM-DD_notes.md`，再尝试重新建索引。重建失败时文件内容仍已保存，当前工具响应仍返回成功，失败详情写入日志。

## 配置

实例 `config:` 通过 `Driver(CMD_INIT)` 传入并优先于环境变量；环境变量仅作为兼容回退。文件系统相对路径以运行时工作目录解析。需要跨构建或缓存清理保留数据时，应使用绝对路径或部署目录中的持久目录。

| `config:` 键 | 环境变量回退 | 默认值 | 用途 |
|---|---|---|---|
| `memory_dir` | `AGENT_MEMORY_DIR` | `./agent_memory` | Markdown 记忆文件目录 |
| `milvus_uri` | `AGENT_MILVUS_URI` | `./agent_milvus.db` | Milvus Lite 数据库路径，或远程 Milvus URI / `host:port` |
| `onnx_threads` | `MEMSEARCH_ONNX_THREADS` | `1` | 正整数；在 aarch64 上设置 ONNX Runtime 线程数 |

`MEMSEARCH_LOG_LEVEL` 控制日志等级，默认值为 `INFO`；它是进程环境变量，不属于实例 `config:`。

默认使用 ONNX embedding，`search` 固定取 `top_k=2`。`compact` 优先读取 `VLM_BASE_URL` / `VLM_API_KEY` / `VLM_MODEL`，再回退到对应的 `OPENAI_*` 变量；两组模型名都未设置时使用 `gpt-5.5`。缺少两组 API 密钥时，`compact` 返回不可用说明，不调用外部端点。
