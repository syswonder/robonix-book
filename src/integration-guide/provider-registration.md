# Provider 注册

Provider 进程启动后须完成两步注册：`RegisterNode` 声明节点身份，`DeclareInterface` 声明所提供的接口。完成注册后启动数据面（MCP HTTP server、gRPC server 或 ROS 2 publisher）并保持运行。

## 建立控制平面连接

```python
import grpc
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc

channel = grpc.insecure_channel(os.environ.get("ROBONIX_ATLAS", "localhost:50051"))
stub = pb_grpc.RobonixRuntimeStub(channel)
```

Python proto 桩文件须放在 **package 内**（如 `tiago_bridge/proto_gen/`），通过 `_ensure_proto_gen()` 将该目录加入 `sys.path`。不要依赖全局共享的 `examples/proto_gen/`。

## RegisterNode

```python
# PRM provider（摘自 tiago_bridge/node.py）
stub.RegisterNode(pb.RegisterNodeRequest(
    node_id="com.robonix.prm.tiago",
    namespace="robonix/prm/camera",
    kind="primitive",
    skill_md="",
    distro="humble",
    container_id="",
))

# System service（摘自 vlm_service/service.py）
stub.RegisterNode(pb.RegisterNodeRequest(
    node_id="com.robonix.services.vlm",
    namespace="robonix/sys/model/vlm",
    kind="service",
    skill_md=_load_skill_md(),
))
```

| 字段 | 说明 |
|------|------|
| `node_id` | reverse-DNS 格式，至少三段（如 `com.robonix.prm.tiago`） |
| `namespace` | 仅在 `DeclareInterface` 未填 `contract_id` 时用于推导契约路径 |
| `kind` | `primitive`、`service` 或 `skill` |
| `skill_md` | 内嵌 SKILL.md 内容（供 VLM 使用）；留空则由 `rbnx start` 从包的 `skills/` 目录加载 |

## DeclareInterface

每个接口有叶子名 `name`、传输列表和可选元数据。建议在所有接口声明中显式填写 `contract_id`，与 `rust/contracts` 中对应契约的 `[contract] id` 一致。

### MCP 工具接口

MCP 工具的 Python 侧须使用 `robonix-codegen --lang mcp` 生成的 `robonix_mcp_types/`（`*_mcp.py` dataclass），并通过 `mcp_contract` 装饰器注册，确保线格式与契约对齐。

**不要**用裸 `@mcp.tool()` 手写工具参数——FastMCP 按 Python 参数名映射 JSON，与 ROS 线格式不一致（例如 `std_msgs/String` 的 wire 键为 `data`，而非 `req`）。

#### `mcp_contract` 装饰器

共享包 `rust/examples/packages/robonix_mcp_contract`（import 名 `robonix_mcp_contract`）提供 `mcp_contract` 装饰器。**`input_cls` 和 `output_cls` 从 handler 的类型注解推断，无需显式传入**：

```python
from robonix_mcp_contract import mcp_contract
from std_msgs_mcp import Empty
from sensor_msgs_mcp import Image

@mcp_contract(mcp, contract_id="robonix/prm/camera/snapshot")
def camera_snapshot(msg: Empty) -> Image:
    """Take a single RGB snapshot."""
    ...
    return image
```

装饰器签名：`mcp_contract(mcp, *, contract_id, name=None, structured_output=None)`

| 项 | 说明 |
|------|------|
| `contract_id` | 与 `rust/contracts/**/*.toml` 中 `[contract] id` 一致 |
| `name` | 可选，覆盖工具名（默认取函数名） |
| Handler 签名 | 恰好一个参数，参数与返回值均须有 codegen 类型注解（支持 sync 和 async） |
| Tool description | 取 handler 的 docstring |
| MCP 线格式 | `arguments` JSON = `input_cls.to_dict()` / `from_dict()`（顶层键 = ROS 消息字段） |
| 返回值 | 返回 codegen 实例，装饰器自动 `to_dict()` 后发往客户端（`Image.data` 为 base64） |

**依赖配置**：将 `packages/robonix_mcp_contract` 加入 `PYTHONPATH`，或在包内执行 `pip install -e ../robonix_mcp_contract`。

> **重要：MCP 接口声明的顺序**  
> 须先完成 codegen（`robonix_mcp_types/` 和 `proto_gen/`），再声明 MCP 接口并启动进程。不要跳过 `rbnx build` 直接 `rbnx start`——常见故障为镜像构建时 `COPY .../proto_gen ... not found`，或运行时参数 schema 与契约不一致。

#### DeclareInterface 示例（MCP）

每个契约单独调用一次 `DeclareInterface`，同一端口可挂多个 MCP 接口。`metadata_json.tools[].input_schema` 使用 `InputCls.json_schema()`，与 `mcp_contract` 线格式保持一致：

```python
mcp_port = _pick_mcp_listen_port()

stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id=node_id,
    name="base_navigate",
    supported_transports=["mcp"],
    metadata_json=_single_tool_meta(
        "base_navigate",
        "Navigate to pose. Arguments: geometry_msgs/PoseStamped wire JSON.",
        PoseStamped.json_schema(),
    ),
    listen_port=mcp_port,
    contract_id="robonix/prm/base/navigate",
))
```

`listen_port` 为 provider 已绑定的端口，控制平面返回的 `allocated_endpoint` 与之一致。设为 0 时由控制平面分配，但此时 provider 须绑定到分配的端口。

### gRPC 数据面接口

同一逻辑接口可在多种传输上分别声明。以 `rgb` 为例，同时注册 gRPC 和 ROS 2：

```python
prm_grpc_port = _pick_grpc_listen_port()

stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id=node_id,
    name="rgb",
    supported_transports=["grpc"],
    metadata_json="{}",
    listen_port=prm_grpc_port,
    contract_id="robonix/prm/camera/rgb",
))

resp_ros2 = stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id=node_id,
    name="rgb",
    supported_transports=["ros2"],
    metadata_json="{}",
    listen_port=0,
    contract_id="robonix/prm/camera/rgb",
))
ros2_topic = resp_ros2.allocated_endpoint   # 如 "/rbnx/ch/n<uuid>"
```

对 gRPC/ROS 2 传输，控制平面会校验 `contract_id` 是否在系统接口目录中，路径不合法时返回 `INVALID_ARGUMENT`。MCP 传输无此约束。

### VLM gRPC 服务接口

VLM 服务在同一端口上提供一元 `Chat`、streaming `ChatStream` 和 `Describe`，共享同一 `contract_id`。须先绑定端口再声明，最后启动 server：

```python
server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
vlm_pb2_grpc.add_VlmServiceServicer_to_server(VlmHandler(), server)
bound_port = server.add_insecure_port("127.0.0.1:0")

stub.DeclareInterface(pb.DeclareInterfaceRequest(
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

## 启动数据面

### MCP HTTP server

```python
from mcp.server.fastmcp import FastMCP
from robonix_mcp_contract import mcp_contract
from std_msgs_mcp import Empty
from sensor_msgs_mcp import Image
import uvicorn

mcp = FastMCP("tiago-node")

@mcp_contract(mcp, contract_id="robonix/prm/camera/snapshot")
def camera_snapshot(msg: Empty) -> Image:
    """Take a single RGB snapshot."""
    ...
    return image

app = mcp.streamable_http_app()
uvicorn.run(app, host="0.0.0.0", port=mcp_port, log_level="warning")
```

### gRPC server

PRM 等 gRPC 流式接口须与 `rust/contracts` 中的契约及 codegen 生成的 `robonix_proto` 保持一致。Python 侧通过 `examples/scripts/gen_proto_python.sh` 生成桩文件。具体实现参考 `tiago_bridge` 中的 servicer 注册方式。

### 工具结果与图像

`robonix-executor` 将 MCP 工具结果以 JSON 字符串回传给 `robonix-pilot`。若结果解析为 `sensor_msgs/Image` 线对象（含 `width`、`height`、`encoding` 和 base64 `data`），或含顶层字段 `image_base64`，Pilot 会将图像以多模态形式送入 VLM（`ChatMessage.image_base64` → OpenAI 兼容 `image_url`）。

## 心跳

Provider 须定期发送心跳。建议间隔 15 秒：

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

## 完整注册流程

```
connect to robonix-atlas
RegisterNode(node_id, namespace, kind)
for each interface:
    bind data-plane port
    DeclareInterface(node_id, name, transports, listen_port, contract_id)
start heartbeat thread
start data-plane servers (MCP / gRPC / ROS 2)
block forever
```

参考实现：`tiago_bridge/node.py` 的 `main()` 函数，可直接作为新 provider 的模板。
