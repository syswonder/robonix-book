# Atlas 能力目录

Atlas 是 Robonix 的**唯一控制平面**——所有进程（不管是 Robonix 自身的系统服务，还是用户提供的 primitive / service / skill 包）启动时第一件事就是连 Atlas，把自己的能力（capability）登记上去；其他进程要找它们，也只能通过 Atlas。Atlas 本身不转发数据面流量，它只做"目录 + 通道协商"。

## 数据模型：capability + interface

一个 **capability instance**（能力实例）是 Atlas 里的最小注册单位，对应一个进程或一台设备的某种能力。每个 capability 有：

- `capability_id`：实例的反向 DNS 名字，进程级唯一。例如 `com.robonix.primitive.tiago_chassis`。空值时 Atlas 分配 `com.robonix.ephemeral.<uuid>`。
- `namespace`：能力所属命名空间前缀，例如 `robonix/primitive/chassis`。后续声明的所有 interface 的 `contract_id` **必须**在这个前缀下，Atlas 强制校验。
- `capability_md_path`：可选。指向包根目录下的 `CAPABILITY.md` 绝对路径。Pilot 在构造 system prompt 时把这条路径列出来，由 LLM 通过 executor 的 `read_file` builtin 按需懒加载（详见下文"capability 文档懒加载"）。
- 0 到 N 个 **interface**：每个 interface 把一个契约（`contract_id`）绑到一种传输（`transport`）和一个端点（`endpoint`），同时携带传输专属参数（`TransportParams`）。

一个进程典型的注册流程：

```
RegisterCapability(capability_id, namespace, capability_md_path)
  └─ DeclareInterface(capability_id, contract_id, transport, endpoint, params)
  └─ DeclareInterface(capability_id, contract_id, transport, endpoint, params)
  └─ ...
Heartbeat(capability_id) ← 每 N 秒续约
```

同一个 capability 下的多个 interface 可以走不同传输。例如 tiago_chassis 既可以暴露 `robonix/primitive/chassis/state`（MCP，给 LLM 调）又暴露 `robonix/primitive/chassis/odom`（ROS 2，给 SLAM 订阅）。

## 三种传输

Atlas 不在乎传输细节，但它必须能把 provider 的 endpoint 完整告诉 consumer。`TransportParams` 是个 oneof，按传输各塞一份元数据：

| 传输 | `TransportParams.kind` | 典型 endpoint | 用途 |
|------|------------------------|---------------|------|
| `Grpc` | `GrpcParams { proto_file, service_name, method }` | `host:port` | 系统服务（pilot / executor / vlm）、PRM 的二进制流式接口 |
| `Ros2` | `Ros2Params { qos_profile }` | 冲突时采用`/rbnx/ch/<uuid>`，其他情况直接用能力发起人提供的 endpoint | 容器内 ROS 节点间通信 |
| `Mcp`  | `McpParams { description, input_schema_json }` | `host:port` (HTTP) 或 `stdio://cmd` | LLM 可调工具 |

## RPC 接口一览

Atlas 服务定义在 `rust/proto/atlas.proto`，关键 RPC：

| RPC | 调用方 | 作用 |
|-----|--------|------|
| `RegisterCapability` | Provider | 登记 capability_id + namespace + capability_md_path |
| `DeclareInterface` | Provider | 把一个 contract_id 绑到一种 transport+endpoint |
| `Heartbeat` | Provider | 续约，超时（默认 30 s）后该 capability 被驱逐 |
| `UnregisterCapability` | Provider | 主动注销（也可以让心跳超时） |
| `QueryCapabilities` | Consumer | 按 `capability_id` / `namespace` / `transport` 过滤检索 |
| `QueryCapabilityMd` | Consumer | 读取 capability_md_path 指向的 markdown 内容（Pilot 不用——LLM 用 read_file 懒加载更省 token） |
| `ConnectCapability` | Consumer | 提交"我要用 cap X 的 contract Y 走 Z 传输"，Atlas 记录通道并返回 endpoint |
| `DisconnectCapability` | Consumer | 释放通道（Atlas 仅做记账） |
| `InspectAtlas` | 调试 | 一次性 dump 当前所有 capabilities + interfaces + channels（JSON） |

`Connect` / `Disconnect` 只是 Atlas 侧的记账：真正的数据面连接（gRPC dial、ROS topic sub、MCP HTTP 客户端）由 consumer 自己用拿到的 endpoint 建。

## 注册流程示例：tiago_chassis driver

`examples/webots/primitives/tiago_chassis/chassis_driver/driver.py` 的注册顺序（伪代码）：

```python
stub = AtlasStub(channel)

# 1. 登记一个 capability instance
stub.RegisterCapability(RegisterCapabilityRequest(
    capability_id     = "com.robonix.primitive.tiago_chassis",
    namespace         = "robonix/primitive/chassis",
    capability_md_path = "/abs/path/to/tiago_chassis/CAPABILITY.md",
))

# 2. 登记两个 MCP interface
def state(msg: Empty) -> RobotState: ...     # @mcp_contract 装饰
def move(msg: MoveCommand) -> String: ...    # @mcp_contract 装饰

for fn, port in [(state, 50111), (move, 50112)]:
    stub.DeclareInterface(DeclareInterfaceRequest(
        capability_id = "com.robonix.primitive.tiago_chassis",
        contract_id   = fn._robonix_contract_id,    # mcp_contract 写入
        transport     = Transport.Mcp,
        endpoint      = f"127.0.0.1:{port}",
        params        = TransportParams(mcp=McpParams(
            description       = (fn.__doc__ or "").strip(),
            input_schema_json = fn._robonix_input_cls.json_schema(),
        )),
    ))

# 3. 后台心跳
asyncio.create_task(heartbeat_loop(stub, "com.robonix.primitive.tiago_chassis"))
```

contract_id（`robonix/primitive/chassis/state`、`robonix/primitive/chassis/move`）必须在 namespace `robonix/primitive/chassis` 前缀下，Atlas 在 DeclareInterface 时会校验。

## capability 文档懒加载

每个包都鼓励在根目录写一份 `CAPABILITY.md`，描述：自己提供的工具、推荐使用模式（"先 snapshot 再 reason 再下指令"之类）、参数语义、典型陷阱（"navigate 是阻塞的，交互场景别用"）。

注册时通过 `capability_md_path` 字段把这份文档的**绝对路径**告诉 Atlas。Pilot 在每个 turn 构造 system prompt 时调 `discovery::cap_md_index(atlas)` 拉一遍，把每条 cap 的 path 用一行列在 system prompt 末尾：

```
## Capability docs (lazy-load via `read_file`)
Each capability below ships a CAPABILITY.md ...

- `com.robonix.primitive.tiago_chassis` (robonix/primitive/chassis): `/.../tiago_chassis/CAPABILITY.md`
- `com.robonix.primitive.tiago_camera`   (robonix/primitive/camera):  `/.../tiago_camera/CAPABILITY.md`
- ...
```

具体内容**不进 system prompt**——LLM 只在确认要用某个 cap 时，通过 executor 的 `read_file` builtin 按需读。这种懒加载策略主要是为了控制 system prompt 大小（已经观察到 tool 描述本身就是大头 token 消耗）；同时让 cap 作者可以写很长的文档而不用担心污染所有 turn 的 prompt。
