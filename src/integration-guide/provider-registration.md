# Provider 注册

Provider 进程启动后需要完成两步注册：`RegisterNode` 声明"我是谁"，`DeclareInterface` 声明"我能提供什么"。之后启动数据面（MCP HTTP server、gRPC server 或 ROS 2 publisher），开始服务请求。

本节以两个真实实现为参照，展示完整的注册代码。

## 建立控制平面连接

所有 provider 都先通过 gRPC 连接 `robonix-atlas`：

```python
import grpc
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc

channel = grpc.insecure_channel(os.environ.get("ROBONIX_ATLAS", "localhost:50051"))
stub = pb_grpc.RobonixRuntimeStub(channel)
```

Python proto 桩文件建议放在 **package 内**（例如 `tiago_bridge/proto_gen/`），provider 代码通过 `_ensure_proto_gen()` 将该目录加入 `sys.path`。不要依赖全局共享的 `examples/proto_gen/`。

## RegisterNode

`RegisterNode` 告诉控制平面这个进程的身份和归属的命名空间。

示例 A — PRM provider（摘自 `tiago_bridge/node.py`）：

```python
stub.RegisterNode(pb.RegisterNodeRequest(
    node_id="com.robonix.prm.tiago",
    namespace="robonix/prm/camera",
    kind="primitive",
    skill_md="",
    distro="humble",
    container_id="",
))
```

示例 B — System service（摘自 `vlm_service/service.py`）：

```python
stub.RegisterNode(pb.RegisterNodeRequest(
    node_id="com.robonix.services.vlm",
    namespace="robonix/sys/model/vlm",
    kind="service",
    skill_md=_load_skill_md(),
))
```

`node_id` 使用 reverse-DNS 格式，至少三段。`namespace` 与 `DeclareInterface.name` 仅在未填 **`contract_id`** 时用于推导契约路径。`kind` 是 `primitive`、`service` 或 `skill`。`skill_md` 可以内嵌 SKILL.md 内容（VLM 使用），也可以留空由 `rbnx start` 从包的 `skills/` 目录加载。

建议在 `DeclareInterface` 中显式设置 **`contract_id`**，与 `rust/contracts` 里对应契约的 **`[contract] id`** 一致。

## DeclareInterface

`DeclareInterface` 声明这个节点能提供的具体接口。每个接口有一个叶子名 `name`、支持的传输列表和可选的元数据。

### MCP 工具接口

Tiago 桥接、ManiSkill demo、`memsearch_service` 等将 **每个契约** 映射为一个 MCP 工具。Python 侧 **必须** 使用 **`robonix-codegen --lang mcp`** 生成的 **`robonix_mcp_types/`**（`*_mcp.py` dataclass），且 **`[io]` 与 `rust/contracts` 中对应契约一致**。

#### 契约对齐的装饰器 `mcp_contract`（推荐）

仓库提供共享包 **`rust/examples/packages/robonix_mcp_contract`**（import 名 **`robonix_mcp_contract`**，装饰器 **`mcp_contract`**）。示例 provider 应 **`from robonix_mcp_contract import mcp_contract`**，用 **`@mcp_contract(...)`** 注册工具，**不要**再用裸 **`@mcp.tool()`** 手写同一套逻辑（FastMCP 按 Python 参数名映射 JSON，容易与 ROS 线格式不一致，例如 `std_msgs/String` 的 wire 键是 **`data`** 而不是 `req`）。

约定（与实现一致）：

| 项 | 说明 |
|----|------|
| **`contract_id`** | 与 `rust/contracts/**/*.toml` 里 **`[contract] id`** 相同，用于报错与追溯。 |
| **`input_cls` / `output_cls`** | codegen 生成的消息类（如 `std_msgs_mcp.String`、`geometry_msgs_mcp.PoseStamped`、`sensor_msgs_mcp.Image`）。 |
| **Handler 签名** | 单参数：`def tool(msg: <input_cls>) -> <output_cls>`（可为 **sync 或 async**）。参数名一般为 `msg`。 |
| **MCP 线格式** | 工具 **`arguments`** 的 JSON 对象 = **`input_cls.to_dict()`** / **`from_dict()`**（顶层键 = ROS 消息字段）。装饰器生成 **shim**，按 `json_schema()` 展开为与线格式一致的参数，再组装成 `msg` 调用你的函数。 |
| **返回值** | Handler 返回 codegen 实例即可；发往 MCP 客户端前会 **`to_dict()`**（如 `sensor_msgs/Image.data` 为 **base64 字符串**），以便 JSON 传输。 |

**依赖路径**：在构建或运行环境中把 **`packages/robonix_mcp_contract`** 加入 **`PYTHONPATH`**（示例 **`scripts/build.sh`** 已写入 `rbnx-build/ws/install/setup.bash`）；或在包内用 **`pip install -e ../robonix_mcp_contract`**。进程内也可在 import 前向上查找目录，将 `.../packages/robonix_mcp_contract` 插入 **`sys.path`**（与示例 `service.py` / `node.py` 一致）。

构建脚本在 `docker compose build` 之前会先运行 codegen；若本机 `rustup`/`cargo` 异常，可设置 **`CARGO=/usr/bin/cargo`** 或保证 `scripts/build.sh` 能调用到可用的 `cargo`。

> **关键顺序（MCP port）**  
> 当你要声明 MCP 接口（`supported_transports=["mcp"]` 并设置 `listen_port`）时，必须先完成 codegen（至少 **`robonix_mcp_types/`**，以及 package 内的 **`proto_gen/`**），并保证能 **`import robonix_mcp_contract`**。不要跳过 **`rbnx build`** 直接 **`rbnx start`**，否则常见故障是镜像构建阶段 `COPY .../proto_gen ... not found`，或运行时参数 schema 与契约不一致。

对每个契约分别 `DeclareInterface`（`contract_id` 与 `[contract] id` 一致），同一端口上可挂多个 MCP 接口。**`metadata_json.tools[].input_schema`** 应与 **MCP 线格式**一致：优先使用 **整段 `InputCls.json_schema()`**（与 `to_dict` 字段一致），例如 **`Empty.json_schema()`**、`**PoseStamped.json_schema()`**、**`String.json_schema()`**；不要误写成与 wire 不符的嵌套（例如把 `PoseStamped` 再包一层 `"pose"` 键，若契约本身就是 `geometry_msgs/PoseStamped` 顶层）。

```python
mcp_port = _pick_mcp_listen_port()

stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id=node_id,
    name="base_navigate",
    supported_transports=["mcp"],
    metadata_json=_single_tool_meta(
        "base_navigate",
        "Navigate to pose. Arguments: geometry_msgs/PoseStamped wire JSON.",
        PoseStamped.json_schema(),   # 与 mcp_contract 线格式一致
    ),
    listen_port=mcp_port,
    contract_id="robonix/prm/base/navigate",
))
```

`listen_port` 告诉控制平面"我已经绑定了这个端口"，控制平面会返回 `allocated_endpoint`（格式 `host:port`），与传入的端口一致。如果设为 0，控制平面自行分配端口（但此时 provider 需要绑定到分配的端口上）。

`metadata_json.tools[].input_schema` 应与 **`mcp_contract` 使用的 `input_cls.json_schema()`** 一致（便于发现与调用对齐）。

### gRPC 数据面接口

同一节点可以为同一个逻辑端口注册多种传输。Tiago 桥接为 camera `rgb` 同时注册了 gRPC 和 ROS 2：

```python
prm_grpc_port = _pick_grpc_listen_port()

# gRPC 声明
stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id=node_id,
    name="rgb",
    supported_transports=["grpc"],
    metadata_json="{}",
    listen_port=prm_grpc_port,
    contract_id="robonix/prm/camera/rgb",
))

# ROS 2 声明
resp_ros2 = stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id=node_id,
    name="rgb",
    supported_transports=["ros2"],
    metadata_json="{}",
    listen_port=0,  # 控制平面分配 topic 名
    contract_id="robonix/prm/camera/rgb",
))
ros2_topic = resp_ros2.allocated_endpoint   # 如 "/rbnx/ch/n<uuid>"
```

对 gRPC/ROS 2 传输，控制平面会校验 **`contract_id`**（显式填写或由上推导）是否落在系统接口目录中。例如 `robonix/prm/camera/rgb`。若拼写错误或路径未入库，`DeclareInterface` 返回 `INVALID_ARGUMENT`。契约定义见 `rust/contracts/`，目录与契约应对齐维护。

### VLM gRPC 服务接口

VLM 的数据面在同一个 `VlmService` 上提供**一元** `Chat` 与 **server-streaming** `ChatStream`（外加 `Describe`）。控制平面通过 `name="chat"` 与 **`contract_id="robonix/sys/model/vlm/chat"`**（或与之一致的推导值）声明业务能力；流式与一元共用同一监听端口，客户端协商到端点后按需选择方法。

`metadata_json.contract` 里建议同时写清一元与流式方法名，便于工具链与人工查阅（字段名由示例约定，服务端校验不依赖这些扩展键）：

```python
server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
vlm_pb2_grpc.add_VlmServiceServicer_to_server(VlmHandler(), server)

bound_port = server.add_insecure_port("127.0.0.1:0")  # 系统分配端口

resp = stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id="com.robonix.services.vlm",
    name="chat",
    supported_transports=["grpc"],
    contract_id="robonix/sys/model/vlm/chat",
    metadata_json=json.dumps({
        "transport": "grpc",
        "contract": {
            "proto_file": "crates/robonix-interfaces/robonix_proto/vlm.proto",
            "service": "robonix.vlm.VlmService",
            "rpc_method": "/robonix.vlm.VlmService/Chat",
            "request_type": "Chat_Request",
            "response_type": "Chat_Response",
            "streaming_rpc_method": "/robonix.vlm.VlmService/ChatStream",
            "stream_request_type": "ChatStream_Request",
            "stream_event_type": "ChatStreamEvent",
        },
    }),
    listen_port=bound_port,
))

server.start()
```

注意顺序：先绑定端口获取 `bound_port`，再 `DeclareInterface` 把端口告诉控制平面，最后 `server.start()` 开始接受请求。

## 启动数据面

### MCP HTTP server

Tiago 桥接等使用 `FastMCP` 的 `streamable_http_app()` 通过 `uvicorn` 提供 MCP HTTP 服务；工具用 **`mcp_contract`** 注册（见上文），例如：

```python
import uvicorn
from mcp.server.fastmcp import FastMCP
from robonix_mcp_contract import mcp_contract
from std_msgs_mcp import Empty
from sensor_msgs_mcp import Image

mcp = FastMCP("tiago-node")

@mcp_contract(
    mcp,
    contract_id="robonix/prm/camera/snapshot",
    input_cls=Empty,
    output_cls=Image,
)
def camera_snapshot(msg: Empty) -> Image:
    ...
    return image

app = mcp.streamable_http_app()
uvicorn.run(app, host="0.0.0.0", port=mcp_port, log_level="warning")
```

### Pilot / VLM 与工具结果中的图像

**`robonix-executor`** 将 MCP 工具结果以 **JSON 字符串**回传给 **`robonix-pilot`**。若解析为 **`sensor_msgs/Image` 线对象**（含 **`width`、`height`、`encoding`** 与 base64 **`data`**），或顶层遗留字段 **`image_base64`**，Pilot 会把像素以 **多模态**形式送入 **`vlm_service`**（`ChatMessage.image_base64` → OpenAI 兼容 **`image_url`**）。因此 **`camera_snapshot` 等返回 Image 线格式时，VLM 能真正“看到”图**，而不是仅把整段 JSON 当文本。

### `rbnx chat` 与中断

在 **`rbnx chat`** TUI 中按 **Esc** 会向 Pilot 发送 **`AbortSession`**（gRPC），用于中断当前一轮推理与工具执行，无需杀进程。

### gRPC server

PRM 相机等流式能力须与 **`rust/contracts`** 中对应契约及 **`robonix-codegen` 生成的 `robonix_proto`** 一致（例如契约 `robonix/prm/camera/rgb` 对应 `robonix.contracts.PrmCameraRgb` 门面与 `sensor_msgs.Image` 载荷；Python 侧使用 `examples/scripts/gen_proto_python.sh` 生成桩）。具体 servicer 实现可参考 `tiago_bridge` 等示例中对 **`robonix_contracts`/`sensor_msgs`** 的注册方式。

## 心跳

Provider 应该定期发送心跳，让控制平面知道自己还活着。15 秒间隔是个合理的选择：

```python
import threading

def _heartbeat_loop(stub, node_id):
    while True:
        time.sleep(15.0)
        try:
            stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
        except Exception as e:
            print(f"heartbeat failed: {e}")

threading.Thread(target=_heartbeat_loop, args=(stub, node_id), daemon=True).start()
```

## 小结

整个注册流程用伪代码表示：

```
connect to robonix-atlas
RegisterNode(node_id, namespace, kind)
for each interface:
    bind data-plane port
    DeclareInterface(node_id, name, transports, listen_port)
start heartbeat thread
start data-plane servers (MCP / gRPC / ROS 2)
block forever
```

`tiago_bridge/node.py` 的 `main()` 函数（约 80 行）完整展示了这个流程，可以直接作为新 provider 的模板。
