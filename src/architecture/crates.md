# Crate 索引

Robonix 的 Rust workspace（`rust/`）包含五个 crate，各自职责清晰、边界明确。

## robonix-server

控制平面的核心进程。启动后在 `ROBONIX_META_GRPC_ADDR`（默认 `0.0.0.0:50051`）上提供 `RobonixRuntime` gRPC 服务。

内部维护一个 `MetaRuntimeRegistry`，存储所有已注册节点、接口和通道的状态。注册时服务端会校验 `node_id` 的 reverse-DNS 格式（至少三段，以 `com.` 开头），对 gRPC/ROS 2 传输的接口声明还会检查 `abstract_interface_id` 是否在系统接口目录（`ROBO_SYSTEM_INTERFACE_CATALOG`）中——不在目录里的路径会被拒绝。MCP 传输不受此约束，允许自由注册。

端口分配逻辑：对 gRPC/MCP 传输，如果 provider 指定了 `listen_port` 则直接使用，否则服务端从 50100 开始递增分配。同一节点上同类传输的多个接口会复用同一端口。ROS 2 和共享内存传输则生成基于 UUID 的唯一端点名。

构建与启动：

```bash
cargo build -p robonix-server
cargo run -p robonix-server
```

或通过 `make install` 安装 wrapper 到 `~/.cargo/bin/robonix-server`。

主要环境变量：

| 变量 | 说明 |
|------|------|
| `ROBONIX_META_GRPC_ADDR` | 监听地址，默认 `0.0.0.0:50051` |
| `ROBONIX_META_GRPC_ENDPOINT` | 客户端连接地址（写入 wrapper） |
| `ROBONIX_DATA_PLANE_HOST` | 数据面端点的 host 部分，默认 `localhost` |
| `RUST_LOG` | 日志级别，如 `robonix_server=info` |

日志中出现 `starting robonix runtime meta API (gRPC)` 表示服务就绪。运行时状态可通过 `InspectRuntime` RPC 或 `rbnx inspect` 命令查看。

## robonix-sdk

Rust 异步 gRPC 客户端库，封装对 `RobonixRuntime` 服务的调用。核心类型是 `RobonixClient`，提供 `connect`、`connect_with_retry`、`register_node`、`declare_interface`、`query_nodes`、`negotiate_channel`、`query_all_skills` 等方法。

其他 Rust crate（`robonix-agent`、`robonix-cli`）通过 `robonix-sdk` 与控制平面通信。Proto 类型由 `tonic-build` 在编译期从 `rust/proto/robonix_runtime.proto` 生成。

这个 crate 只是 library，没有可执行文件。

## robonix-agent

系统智能体，启动后注册为 `com.robonix.runtime.agent`，然后执行两步发现：通过 `QueryNodes` + `NegotiateChannel` 获取 VLM 服务的 gRPC 端点；扫描所有 `transport=mcp` 的接口，从 `metadata_json.endpoint` 连接 MCP 服务器，获取可用工具列表。

发现完成后进入 ReAct 交互循环（`react.rs`）：从 stdin 读取用户指令，将指令、系统 prompt（含 SKILL.md 和工具 schema）、对话历史发送给 VLM，解析 VLM 返回的 `tool_calls`，通过 MCP 协议调用对应工具，将结果追加到历史，再次调用 VLM——如此循环直到 VLM 判定任务完成。

Agent 的行为由几个关键机制保证持续性：`tool_persist_nudges` 在 VLM 返回无 tool_calls 时自动追加提示，要求继续链式调用；`move_base` 重复检测在发现连续相同的运动命令时强制插入感知步骤（`get_robot_pose` + `get_camera_image`）。

主要环境变量：

| 变量 | 说明 |
|------|------|
| `ROBONIX_SERVER` | 控制平面地址，默认 `localhost:50051` |
| `ROBONIX_AGENT_MAX_TOOL_ROUNDS` | 每轮对话最大工具调用次数，默认 64 |
| `ROBONIX_AGENT_TOOL_PERSIST_NUDGES` | 空回复时追加提示的最大次数 |

## robonix-cli (rbnx)

命令行工具，二进制名 `rbnx`。分为两类功能：

包管理命令操作本地文件系统和 `robonix_manifest.yaml`：

| 命令 | 作用 |
|------|------|
| `rbnx validate <path>` | 校验 manifest 格式 |
| `rbnx build -p <path>` | 执行 `build.script` |
| `rbnx start -p <pkg> -n <node_id>` | 启动节点进程 |
| `rbnx install --github <repo>` | 从 GitHub 安装包 |
| `rbnx list` / `rbnx info <name>` | 列出/查看已安装包 |

运行时检查命令通过 gRPC 查询 `robonix-server`：

| 命令 | 作用 |
|------|------|
| `rbnx nodes` | 列出已注册节点及接口 |
| `rbnx describe` | 查看节点的 SKILL.md |
| `rbnx tools` | 列出 Agent 可见的所有工具 |
| `rbnx channels` | 查看已协商的通道 |
| `rbnx inspect` | 导出完整运行时状态（JSON） |

## ridlc

ROS IDL 到 Proto 的代码生成器。读取 `robonix-interfaces/lib/` 下的 `.msg` 和 `.srv` 文件，按包名生成对应的 `.proto` 文件到 `robonix-interfaces/robonix_proto/`。

命名规则：ROS 包 `prm_base` 生成 `prm_base.proto`，包名空间 `robonix.prm_base`。每个 `.msg` 生成一个 `message`，每个 `.srv` 生成 `_Request`/`_Response` message 和对应的 `rpc`。

```bash
cd rust
cargo run -p ridlc -- --lang proto \
  -I robonix-interfaces/lib \
  -o robonix-interfaces/robonix_proto
```

目前只支持 `--lang proto`。生成 Python 桩文件（供 `tiago_bridge` 等 Python 代码使用）：

```bash
cd rust && ./examples/scripts/gen_proto_python.sh
```
