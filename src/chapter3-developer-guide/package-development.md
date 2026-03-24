# 节点开发指南

本指南说明如何编写一个 Robonix 节点，注册到控制平面，声明接口，并编写 SKILL.md 供 agent 发现和调用。


## 1. 概念

| 概念 | 含义 |
|------|------|
| Node | 独立进程，通过 gRPC API 注册到 `robonix-server` |
| Interface | 节点声明的通信接口，包含支持的传输类型 |
| Channel | 系统分配的通信端点（topic 名、shm key 等） |
| SKILL.md | 描述节点技能的文档，供 LLM/agent 理解和调用 |


## 2. 开发流程

### 2.1 注册节点

通过 gRPC 调用 `RegisterNode`：

```python
stub.RegisterNode(pb.RegisterNodeRequest(
    node_id="my-camera",
    namespace="robonix/prm/camera",
    kind="primitive",
    skill_md=open("SKILL.md").read()  # 可选
))
```

### 2.2 声明接口

通过 `DeclareInterface` 声明节点对外提供的接口及支持的传输类型：

```python
stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id="my-camera",
    name="rgb",
    supported_transports=["ros2", "shared_memory"],
    metadata_json='{"format": "bgr8", "width": 640, "height": 480}'
))
```

### 2.3 消费者协商通道

消费者通过 `NegotiateChannel` 获取系统分配的端点：

```python
resp = stub.NegotiateChannel(pb.NegotiateChannelRequest(
    consumer_id="my-slam",
    provider_node_id="my-camera",
    interface_name="rgb",
    transport="ros2"
))
topic = resp.endpoint  # 系统分配的 ROS2 topic，如 /rbnx/ch/n...
```

协商后用原生 API（`rclpy`、`grpc`、`posix_ipc` 等）在该端点上通信。

### 2.4 编写 SKILL.md

可调用的 service/skill 应附带 SKILL.md（格式见 [RFC005](../rfc/RFC005-SKILL-Format.md)）：

```markdown
# navigate_to

导航到指定位置。

## Parameters

​```json
{
  "type": "object",
  "properties": {
    "target": { "type": "string", "description": "目标位置" }
  },
  "required": ["target"]
}
​```

## Returns

成功或失败状态。

## Example

​```json
{"target": "kitchen"}
​```
```


## 3. 节点目录结构

```
my_node/
├── main.py          # 入口：注册、声明接口、主循环
├── SKILL.md         # 技能描述（可选）
└── requirements.txt # Python 依赖（可选）
```


## 4. Rust 节点

Rust 节点使用 `robonix-sdk` crate：

```rust
use robonix_sdk::RobonixClient;

let mut client = RobonixClient::connect("http://localhost:50051").await?;
let node_id = client.register_node("my-node", "robonix/prm/camera", "primitive", None).await?;
client.declare_interface(&node_id, "rgb", vec!["ros2".into()], "{}").await?;
```


## 5. 参考

- 示例代码：`rust/examples/nodes/`
- SKILL.md 示例：`rust/examples/skills/`
- 传输示例：`rust/examples/transports/`
- [RFC003: 控制平面设计](../rfc/RFC003-Control-Plane.md)
- [RFC005: SKILL.md 格式](../rfc/RFC005-SKILL-Format.md)
- [RFC006: 多传输支持](../rfc/RFC006-Multi-Transport.md)
