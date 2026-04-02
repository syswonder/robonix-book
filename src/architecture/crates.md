# Crate 索引

Robonix 的 Rust workspace（`rust/`）包含 atlas、sdk、pilot、executor、liaison、cli、ridlc、buffer 等 crate，各自职责清晰、边界明确。

## robonix-atlas

控制平面的核心进程。启动后在 `ROBONIX_META_GRPC_ADDR`（默认 `0.0.0.0:50051`）上提供 `RobonixRuntime` gRPC 服务。

内部维护一个 `MetaRuntimeRegistry`，存储所有已注册节点、接口和通道的状态。注册时服务端会校验 `node_id` 的 reverse-DNS 格式（至少三段，例如以 `com.` 、`org.` 、`cn.` 开头），对 gRPC/ROS 2 传输的接口声明还会检查 **`contract_id`**（`DeclareInterfaceRequest` 显式字段或由其推导）是否在系统接口目录（`ROBO_SYSTEM_INTERFACE_CATALOG`）中——不在目录里的路径会被拒绝。MCP 传输不受此约束，允许自由注册。

端口分配逻辑：对 gRPC/MCP 传输，如果 provider 指定了 `listen_port` 则直接使用，否则服务端从 50100 开始递增分配。同一节点上同类传输的多个接口会复用同一端口。ROS 2 和共享内存传输则生成基于 UUID 的唯一端点名。

构建与启动：

```bash
cd rust && make install
robonix-atlas
```

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

其他 Rust crate（如 `robonix-cli`、`robonix-pilot`）通过 `robonix-sdk` 与控制平面通信。Proto 类型由 `tonic-build` 在编译期从 `rust/proto/robonix_runtime.proto` 生成。

这个 crate 只是 library，没有可执行文件。

## robonix-pilot

推理与会话运行时（二进制 **`robonix-pilot`**）。在 Atlas 注册 **`contract_id = robonix/sys/runtime/pilot`**，数据面为 **`PilotService.HandleIntent`**：请求侧 **`Intent`**，响应为 **`PilotEvent`** 流（契约见 **`rust/contracts/sys/pilot.v1.toml`**、`lib/pilot/`）。

Pilot 内部编排 VLM、Executor 等；**`rbnx chat`** 经 Atlas 按 **`robonix/sys/runtime/pilot`** 发现端点后，用 **`PilotService`** 与用户流式对话——仓库**已移除** `rust/proto/agent_chat.proto` 与 **`AgentChat`** 控制面契约。

主要环境变量（节选）：

| 变量 | 说明 |
|------|------|
| `ROBONIX_ATLAS` / `ROBONIX_ATLAS_ENDPOINT` | 控制平面 |
| `ROBONIX_PILOT_PORT` | Pilot gRPC 端口，默认 `50071` |
| `ROBONIX_EXECUTOR_ENDPOINT` | Executor，默认 `localhost:50061` |

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

运行时检查命令通过 gRPC 查询 `robonix-atlas`：

| 命令 | 作用 |
|------|------|
| `rbnx nodes` | 列出已注册节点及接口 |
| `rbnx describe` | 查看节点的 SKILL.md |
| `rbnx tools` | 列出 Agent 可见的所有工具 |
| `rbnx channels` | 查看已协商的通道 |
| `rbnx inspect` | 导出完整运行时状态（JSON） |

交互和可视化命令：

| 命令 | 作用 |
|------|------|
| `rbnx chat` | 启动 TUI 客户端，连接 **Pilot**（`robonix/sys/runtime/pilot`） |
| `rbnx graph` | 生成系统拓扑图（内置渲染 PNG/SVG） |

### rbnx chat

`rbnx chat` 通过 Atlas **`QueryNodes(contract_id=robonix/sys/runtime/pilot)`** 发现 Pilot 端点，使用 **`PilotService.HandleIntent`** 流式会话，基于 `ratatui` 的终端 UI：

- 上方为滚动消息历史，按角色着色（用户、模型回复、工具调用等）
- 下方为文本输入区域
- PageUp/PageDown 滚动，Ctrl+C 退出

```bash
rbnx chat                           # 自动发现 Pilot
rbnx chat --server 192.168.1.5:50051  # 指定控制平面地址
```

### rbnx graph

`rbnx graph` 查询控制平面中所有已注册节点和已协商通道，用内置布局与 SVG 绘制拓扑，再按需输出 **PNG**（同图经 [resvg](https://github.com/linebender/resvg) 光栅化）或 **原始 SVG**。

```bash
rbnx graph -o topology.png                 # 默认 PNG
rbnx graph -o topology.svg --format svg    # SVG
rbnx graph -o out.png --format png       # 显式 PNG
```

图中每个节点为带接口子块的卡片（节点 ID、`kind`、namespace、各接口的 `contract_id` / 端点）。已协商的通道为贝塞尔有向边。`--test` 可不连 Atlas 生成随机示例图。

## robonix-buffer

系统级缓冲区管理库，负责 POSIX 共享内存的分配/映射、GPU DMA 页面锁定（`cudaHostRegister`）、CUDA IPC 跨进程显存共享，以及缓冲区生命周期管理。编译为 `librobonix_buffer.so`，通过 C FFI 供 Python（ctypes）和 C/C++ 使用。

缓冲区系统不限于图像——支持任何高带宽连续数据（点云、张量、大模型 embedding、体素网格等）。

核心类型是 `RobonixBufferManager`，提供：

| 方法 | 说明 |
|------|------|
| `allocate(shm_name, spec)` | 分配图像缓冲区（w×h×c），写入 `BufferHeader` |
| `allocate_raw(shm_name, size, format)` | 分配任意大小的缓冲区（张量、点云等） |
| `open(shm_name)` | 打开另一进程创建的 SHM，自动读取 header 获取元信息 |
| `attach(handle, pin_gpu)` | 附加消费者，可选 `cudaHostRegister` |
| `detach(handle)` | 分离消费者，自动 `cudaHostUnregister` |
| `release(handle)` | 释放缓冲区，`munmap` + `shm_unlink` |

`BufferFormat` 枚举覆盖图像格式（RGB8、BGR8、DEPTH_F32 等）和通用数据格式（FLOAT32、BF16、INT8、INT64、RAW_BYTES 等）。Shape 和 stride 等 N-D 元信息通过控制平面的 `metadata_json` 传递。

CUDA 功能通过动态加载 `libcudart.so` 实现，无 CUDA 环境时退化为纯 CPU 模式。CUDA IPC 相关函数（`ipc_get_mem_handle`、`ipc_open_mem_handle`、`ipc_close_mem_handle`）允许在不同进程间共享 GPU 显存。

详细设计和数据流见[零拷贝缓冲区系统](buffer-system.md)。

## ridlc

ROS IDL 到 Proto 的代码生成器。读取 `robonix-interfaces/lib/` 下的 `.msg` 和 `.srv` 文件，按包名生成对应的 `.proto` 文件到 `robonix-interfaces/robonix_proto/`。

命名规则：ROS 包 `prm_base` 生成 `prm_base.proto`，包名空间 `robonix.prm_base`。每个 `.msg` 生成一个 `message`，每个 `.srv` 生成 `_Request`/`_Response` message 和对应的 `rpc`。

```bash
cd rust
ridlc --lang proto \
  -I robonix-interfaces/lib \
  -o robonix-interfaces/robonix_proto
```

目前只支持 `--lang proto`。生成 Python 桩文件（供 `tiago_bridge` 等 Python 代码使用）：

```bash
cd rust && ./examples/scripts/gen_proto_python.sh
```
