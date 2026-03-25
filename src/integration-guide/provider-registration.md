# Provider 注册

Provider 进程启动后需要完成两步注册：`RegisterNode` 声明"我是谁"，`DeclareInterface` 声明"我能提供什么"。之后启动数据面（MCP HTTP server、gRPC server 或 ROS 2 publisher），开始服务请求。

本节以两个真实实现为参照，展示完整的注册代码。

## 建立控制平面连接

所有 provider 都先通过 gRPC 连接 `robonix-server`：

```python
import grpc
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc

channel = grpc.insecure_channel(os.environ.get("ROBONIX_SERVER", "localhost:50051"))
stub = pb_grpc.RobonixRuntimeStub(channel)
```

Python proto 桩文件由 `rust/examples/scripts/gen_proto_python.sh` 生成到 `rust/examples/proto_gen/`。provider 代码中通过 `_ensure_proto_gen()` 将该目录加入 `sys.path`。

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

`node_id` 使用 reverse-DNS 格式，至少三段。`namespace` 决定了后续 `DeclareInterface` 时 `abstract_interface_id` 的前缀。`kind` 是 `primitive`、`service` 或 `skill`。`skill_md` 可以内嵌 SKILL.md 内容（VLM 使用），也可以留空由 `rbnx start` 从包的 `skills/` 目录加载。

## DeclareInterface

`DeclareInterface` 声明这个节点能提供的具体接口。每个接口有一个叶子名 `name`、支持的传输列表和可选的元数据。

### MCP 工具接口

Tiago 桥接暴露 MCP 工具（`get_camera_image`、`navigate_to` 等）供 Agent 调用：

```python
mcp_port = _pick_mcp_listen_port()   # 选一个空闲 TCP 端口

resp = stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id=node_id,
    name="mcp_tools",
    supported_transports=["mcp"],
    metadata_json=json.dumps({"tools": _mcp_tools_list()}),
    listen_port=mcp_port,
))
```

`listen_port` 告诉控制平面"我已经绑定了这个端口"，控制平面会返回 `allocated_endpoint`（格式 `host:port`），与传入的端口一致。如果设为 0，控制平面自行分配端口（但此时 provider 需要绑定到分配的端口上）。

MCP 传输不受系统接口目录约束，`name` 可以自由定义。`metadata_json` 中包含工具 schema，Agent 发现节点后从中提取工具列表。

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
))

# ROS 2 声明
resp_ros2 = stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id=node_id,
    name="rgb",
    supported_transports=["ros2"],
    metadata_json="{}",
    listen_port=0,  # 控制平面分配 topic 名
))
ros2_topic = resp_ros2.allocated_endpoint   # 如 "/rbnx/ch/n<uuid>"
```

对 gRPC/ROS 2 传输，控制平面会校验 `abstract_interface_id`（= `namespace` + `/` + `name`）是否在系统接口目录中。`robonix/prm/camera/rgb` 在目录中，所以通过。如果拼写错误或路径不存在，`DeclareInterface` 会返回 `INVALID_ARGUMENT` 错误。

### VLM gRPC 服务接口

VLM service 的接口声明更简单——只有一个 gRPC RPC：

```python
server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
vlm_pb2_grpc.add_VlmServiceServicer_to_server(VlmHandler(), server)

bound_port = server.add_insecure_port("127.0.0.1:0")  # 系统分配端口

resp = stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id="com.robonix.services.vlm",
    name="chat",
    supported_transports=["grpc"],
    metadata_json=json.dumps({
        "transport": "grpc",
        "contract": {
            "proto_file": "robonix-interfaces/robonix_proto/vlm.proto",
            "service": "robonix.vlm.VlmService",
            "rpc_method": "/robonix.vlm.VlmService/Chat",
        },
    }),
    listen_port=bound_port,
))

server.start()
```

注意顺序：先绑定端口获取 `bound_port`，再 `DeclareInterface` 把端口告诉控制平面，最后 `server.start()` 开始接受请求。

## 启动数据面

### MCP HTTP server

Tiago 桥接使用 `FastMCP` 的 `streamable_http_app()` 通过 `uvicorn` 提供 MCP HTTP 服务：

```python
import uvicorn
from mcp.server.fastmcp import FastMCP

mcp = FastMCP("tiago-node")

@mcp.tool()
def get_camera_image() -> str:
    ...

app = mcp.streamable_http_app()
uvicorn.run(app, host="0.0.0.0", port=mcp_port, log_level="warning")
```

每个 `@mcp.tool()` 装饰的函数都会成为 Agent 可调用的工具。

### gRPC server

PRM camera 的 gRPC streaming 使用 `grpc.server` + 自定义 servicer：

```python
class PrmCameraServicer(prm_camera_pb2_grpc.PrmCameraServiceServicer):
    def SubscribeRgb(self, request, context):
        while context.is_active():
            jpg = get_latest_rgb_frame()
            if jpg:
                yield jpeg_to_sensor_image_proto(jpg)
            time.sleep(0.5)

server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
prm_camera_pb2_grpc.add_PrmCameraServiceServicer_to_server(PrmCameraServicer(), server)
server.add_insecure_port(f"0.0.0.0:{prm_grpc_port}")
server.start()
```

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
connect to robonix-server
RegisterNode(node_id, namespace, kind)
for each interface:
    bind data-plane port
    DeclareInterface(node_id, name, transports, listen_port)
start heartbeat thread
start data-plane servers (MCP / gRPC / ROS 2)
block forever
```

`tiago_bridge/node.py` 的 `main()` 函数（约 80 行）完整展示了这个流程，可以直接作为新 provider 的模板。
