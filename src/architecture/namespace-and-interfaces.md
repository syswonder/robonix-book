# 能力与能力接口

Robonix 把所有可发现的功能统一抽象成"**能力（capability）**"，能力包括原语、服务、技能。

## 命名空间

每个 capability 注册时声明一个 `namespace` 前缀，所有它的 interface 的 `contract_id` 必须从这个前缀开始。一级命名空间分四类：

| 前缀 | 含义 | 谁实现 |
|------|------|--------|
| `robonix/primitive/*` | 原语：低层硬件抽象（chassis、camera、lidar、gripper 等） | 设备驱动包 |
| `robonix/service/*`   | 服务：场景级算法/能力（slam、navigation、semantic_map 等） | Robonix 提供默认实现，可被替换，可自定义场景服务并自行实现 |
| `robonix/skill/*`     | 技能：封装了特定语义功能、调用原语和服务来完成用户特定任务的可复用功能单元 | 用户/算法开发者自行定义接口和进行实现 |
| `robonix/system/*`    | 系统（服务）：Robonix 自身的核心服务（pilot、executor、liaison、memory） | 仓库内置，不可替换 |

## 契约（contract）

contract 是 interface 的**身份证**——`contract_id` 在整个系统里全局唯一，能力提供方和消费方都按 `contract_id` 对话，跟具体进程、传输无关。每个 contract 由一份 TOML 描述：

```toml
# rust/contracts/primitive/chassis_move.v1.toml
[contract]
id      = "robonix/primitive/chassis/move"
version = "1"
kind    = "primitive"

[io.srv]
srv = "prm_chassis/srv/MoveCommand"

[mode]
type = "rpc"
```

字段含义：

- `[contract]`：身份。`id` 采用 Path-style 命名空间，如 `robonix/primitive/chassis/move`，version 为接口版本，kind 为这个能力所属于的大类。
- `[io.srv]` 或 `[io.msg]`：载荷形状。指向 ROS IDL（`std_msgs/Empty`、`prm_chassis/MoveCommand`）。**官方 contract**（在 robonix 源码仓库的 `capabilities/`）的 IDL 路径解析到 robonix 源码仓库的 `rust/crates/robonix-interfaces/lib/`；**包内 contract**（在 package 的 `capabilities/`）解析到包自己的 `capabilities/srv/`、`capabilities/msg/`。
- `[mode]`：交互形态：
  - `rpc`：一元 RPC，如 ROS2 service 或 grpc unary。
  - `rpc_server_stream`：服务器端流式 RPC，目前仅支持生成 grpc 对应的通信 stub，ROS2 暂不原生支持。
  - `rpc_client_stream`：客户端流式 RPC，目前仅支持生成 grpc 对应的通信 stub，ROS2 暂不原生支持。
  - `rpc_bidirectional_stream`：双向流式 RPC，目前仅支持生成 grpc 对应的通信 stub，ROS2 暂不原生支持。
  - `topic_out`：流式发布者，如 ROS2 topic 发布或 grpc server-stream（且 request 为空）。
  - `topic_in`：流式订阅者，如 ROS2 topic 订阅或 grpc client-stream（且 response 为空）。

contract 文件本身不包含传输信息——同一个 `robonix/primitive/camera/rgb` 既可以由一个 ROS 2 桥提供，也可以由一个独立的 gRPC 服务提供，也可以由一个 MCP 服务或共享内存提供。不过受限于不同具体通信模式的情况，某些 contract 只能在特定传输上使用（如 rpc_server_stream, rpc_client_stream, rpc_bidirectional_stream 当前仅支持 grpc，而无法原始翻译为 ROS2/MCP 的通信，ROS2 action codegen 暂未支持）。

## codegen：契约 → 代码

**`rbnx codegen`**（详见 [Build 与 Codegen](../integration-guide/build-and-codegen.md)）从包的 `capabilities:` 列表读 contract，生成 stub（通过参数开关）：

1. **proto / Python stubs**：把 ROS IDL 翻成 protobuf，再用 `grpc_tools.protoc` 生成 `proto_gen/*_pb2.py`。包代码 `from proto_gen.prm_chassis_pb2 import MoveCommand` 即可。
2. **MCP types**（`--mcp`）：把 contract 的 IO 类型生成为 pydantic-like 类，含 `.json_schema()`、`.model_validate(dict)`、`.model_dump()`。落到 `<pkg>/robonix_mcp_types/`。

> codegen 全部产出到 `<pkg>/rbnx-build/codegen/`（proto stubs）和 `<pkg>/robonix_mcp_types/`（MCP types）。`rbnx start` 在 spawn 子进程前会把这两个目录加进 `PYTHONPATH`。

## 通道（channel）

consumer 要使用某个 cap 的 interface 时，先调 Atlas 的 `ConnectCapability(consumer_id, namespace_filter, transport)`，Atlas 选一个匹配的 provider，记录这条 consumer→provider 的 channel，并把 endpoint 返回给 consumer。consumer 拿到 endpoint 自己 dial（gRPC）或订阅（ROS）或起 HTTP client（MCP）。channel 在 Atlas 侧只是一条记账记录，可以用 `rbnx channels` 查看。

不需要走 Connect 也能用：所有的只读发现都用 `QueryCapabilities` 直接拿 endpoint。Connect 主要给系统服务用——pilot / executor 启动时连一次 cognition 和 memory，channel 帮 atlas 跟踪谁在用谁，方便 `rbnx inspect` 时给运维一个完整图。
