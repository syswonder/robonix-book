# Atlas 能力目录

Atlas 是 Robonix 的**唯一控制平面**——所有进程（不管是 Robonix 自身的系统服务，还是用户提供的 primitive / service / skill 包）启动时第一件事就是连 Atlas，把自己暴露的 capability 登记上去；其他进程要找它们，也只能通过 Atlas。Atlas 本身不转发数据面流量，它只做"目录 + 通道协商"。

## 数据模型：能力提供者 + capability

**能力提供者**（统称，对应内部类型 `CapabilityProvider`）只有三种且仅三种：**primitive / service / skill**。每个能力提供者注册一次，提供 ≥1 个 **capability**。

能力提供者字段：

- `id`：能力提供者在 atlas 里的唯一 id（如 `tiago_chassis` / `mapping` / `explore`）。空值时 atlas 分配 `com.robonix.ephemeral.<uuid>`。
- `namespace`：归属命名空间前缀（如 `robonix/primitive/chassis`）。该 provider 提供的所有 capability 的 `contract_id` 必须在此前缀下，atlas 强制校验。
- `capability_md_path`：可选。指向包根 `CAPABILITY.md` 的绝对路径。Pilot 把 path 列入 system prompt，LLM 通过 executor `read_file` 按需懒加载。

**capability** 是能力提供者暴露的一条接口：`(contract_id, transport, endpoint, params, description)`。capability 没有自己的 id，反向通过 `(provider_id, contract_id, transport)` 唯一寻址。`provider_id` / `provider_kind` 直接拷在每条 capability 上，consumer 拿到 flat list 时不需要回查 provider 也能立刻定位。

典型注册流程：

```
RegisterPrimitive(id, namespace, capability_md_path)    # or RegisterService / RegisterSkill
  └─ DeclareCapability(provider_id, contract_id, transport, endpoint, params)
  └─ DeclareCapability(provider_id, contract_id, transport, endpoint, params)
  └─ ...
Heartbeat(id) ← 每 N 秒续约
```

同一个能力提供者下的多条 capability 可以走不同传输。例如 tiago_chassis 同时暴露 `robonix/primitive/chassis/move`（gRPC，给 service/skill 调用）和 `robonix/primitive/chassis/odom`（ROS 2，给 SLAM 订阅）。

## 三种传输

Atlas 不在乎传输细节，但它必须能把 provider 的 endpoint 完整告诉 consumer。`TransportParams` 是个 oneof，按传输各塞一份元数据：

| 传输 | `TransportParams.kind` | 典型 endpoint | 用途 |
|------|------------------------|---------------|------|
| `Grpc` | `GrpcParams { proto_file, service_name, method }` | `host:port` | 系统服务（pilot / executor / vlm）、PRM 的二进制流式接口 |
| `Ros2` | `Ros2Params { qos_profile }` | 冲突时采用 `/rbnx/ch/<uuid>`，其他情况直接用 provider 提供的 endpoint | 容器内 ROS 节点间通信 |
| `Mcp`  | `McpParams { description, input_schema_json }` | `host:port` (HTTP) 或 `stdio://cmd` | LLM 可调工具 |

## RPC 接口一览

Atlas 服务定义在 `rust/crates/robonix-atlas/proto/atlas.proto`，关键 RPC：

| RPC | 调用方 | 作用 |
|-----|--------|------|
| `RegisterPrimitive` / `RegisterService` / `RegisterSkill` | 能力提供者 | 登记 id + namespace + capability_md_path（按种类三个 typed RPC） |
| `DeclareCapability` | 能力提供者 | 把一个 contract_id 绑到一种 transport+endpoint |
| `Heartbeat` | 能力提供者 | 续约，超时（默认 30 s）后该 provider 被驱逐 |
| `Unregister` | 能力提供者 | 主动注销（也可以让心跳超时） |
| `SetLifecycleState` | 能力提供者 | 推送当前状态（REGISTERED / INACTIVE / ACTIVE / ERROR / TERMINATED） |
| `Query` | 消费者 | 按 `id` / `namespace_prefix` / `kind` / `contract_id` / `transport` 过滤检索 |
| `ConnectCapability` | 消费者 | 提交"我要用 provider X 的 contract Y 走 Z 传输"，atlas 记录通道并返回 endpoint |
| `DisconnectCapability` | 消费者 | 释放通道（atlas 仅做记账） |
| `QueryContract` / `ListContracts` | 消费者 | 拉契约 IDL（字段定义、Request/Response 形状） |
| `InspectAtlas` | 调试 | 一次性 dump 当前所有 providers + capabilities + channels（JSON） |

`Connect` / `Disconnect` 只是 atlas 侧的记账：真正的数据面连接（gRPC dial、ROS topic sub、MCP HTTP 客户端）由 consumer 自己用拿到的 endpoint 建。

## 注册流程示例：tiago_chassis driver

`examples/webots/primitives/tiago_chassis/chassis_driver/driver.py` 的注册顺序（伪代码）：

```python
ATLAS.register_primitive(
    id                 = "tiago_chassis",
    namespace          = "robonix/primitive/chassis",
    capability_md_path = "/abs/path/to/tiago_chassis/CAPABILITY.md",
)

# 声明一条 MCP capability
ATLAS.declare_capability(
    provider_id       = "tiago_chassis",
    contract_id       = "robonix/primitive/chassis/move",
    transport         = Transport.MCP,
    endpoint          = "127.0.0.1:50112",
    params            = McpParams(
        description       = "Drive the chassis at (linear, angular).",
        input_schema_json = MoveCommand.json_schema(),
    ),
)

# 心跳由 robonix_api Provider 框架后台维护
```

contract_id（`robonix/primitive/chassis/move`、`robonix/primitive/chassis/odom`）必须在 namespace `robonix/primitive/chassis` 前缀下，atlas 在 `DeclareCapability` 时会校验。

## capability 文档懒加载

每个包都鼓励在根目录写一份 `CAPABILITY.md`，描述：自己提供的工具、推荐使用模式（"先 snapshot 再 reason 再下指令"之类）、参数语义、典型陷阱（"navigate 是阻塞的，交互场景别用"）。

注册时通过 `capability_md_path` 字段把这份文档的**绝对路径**告诉 atlas。Pilot 在每个 turn 构造 system prompt 时拉一遍，把每条 provider 的 path 用一行列在 system prompt 末尾：

```
## Capability docs (lazy-load via `read_file`)
Each provider below ships a CAPABILITY.md ...

- `tiago_chassis` (robonix/primitive/chassis): `/.../tiago_chassis/CAPABILITY.md`
- `tiago_camera`  (robonix/primitive/camera):  `/.../tiago_camera/CAPABILITY.md`
- ...
```

具体内容**不进 system prompt**——LLM 只在确认要用某个 provider 时，通过 executor 的 `read_file` builtin 按需读。这种懒加载策略主要是为了控制 system prompt 大小（已经观察到 tool 描述本身就是大头 token 消耗）；同时让 provider 作者可以写很长的文档而不用担心污染所有 turn 的 prompt。
