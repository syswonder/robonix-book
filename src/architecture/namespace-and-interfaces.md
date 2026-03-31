# 命名空间与接口模型

Robonix 用一棵树形命名空间管理所有硬件能力和系统服务。每个注册节点占据树上一个位置，每个接口对应一个确定的抽象路径。这套命名体系与具体的 Linux 发行版和 ROS 2 消息名称无关——ROS topic 和 action 是实现细节，不出现在命名空间路径中。

完整设计稿见 `rust/docs/NAMESPACE.md`。

## 命名空间树

所有节点的 namespace 都以 `robonix/` 开头，第二段标识域：

| 域 | 用途 |
|----|------|
| `prm` | 物理机器人、仿真、虚拟硬件的能力抽象（底盘、相机、机械臂等） |
| `sys` | 框架服务：VLM/LLM、地图、规划、技能/任务管理、调试工具 |

`prm` 下按硬件类别细分：`base`、`camera`、`sensor`、`arm`、`gripper`、`force_torque`。`sys` 下按功能域细分：`model`、`map`、`planning`、`manager`、`debug`、`runtime`。

## abstract_interface_id

abstract_interface_id 是接口发现的关键标识。它由 `RegisterNode.namespace` 和 `DeclareInterface.name` 拼接而成：

```
namespace = "robonix/sys/model/vlm"
name      = "chat"
→ abstract_interface_id = "robonix/sys/model/vlm/chat"
```

消费者通过 `QueryNodes` 的 `abstract_interface_id` 字段做精确匹配来发现 provider。也可以用 `namespace` 前缀做模糊匹配。

proto 中的相关定义（摘自 `rust/proto/robonix_runtime.proto`）：

```protobuf
message DeclareInterfaceRequest {
  string node_id = 1;
  string name = 2;                      // 叶子名，如 "rgb"、"chat"
  repeated string supported_transports = 3;
  string metadata_json = 4;
  uint32 listen_port = 5;
  string abstract_interface_id = 6;     // 留空则服务端自动拼接
}

message QueryNodesRequest {
  string namespace = 1;
  string name = 2;
  string transport = 3;
  string abstract_interface_id = 6;     // 非空时忽略 namespace/name 过滤
}
```

## 多传输

同一个逻辑接口可以在不同传输上分别声明。例如 `tiago_bridge` 为 `rgb` 端口同时注册了 gRPC（server-stream）和 ROS 2（topic republish）两种实现：

```python
# gRPC 声明
stub.DeclareInterface(DeclareInterfaceRequest(
    node_id=node_id, name="rgb",
    supported_transports=["grpc"],
    listen_port=prm_grpc_port,
))

# ROS 2 声明
stub.DeclareInterface(DeclareInterfaceRequest(
    node_id=node_id, name="rgb",
    supported_transports=["ros2"],
    listen_port=0,
))
```

服务端允许同一节点上同名接口存在多条记录，只要传输类型不同。消费者在 `NegotiateChannel` 时通过 `transport` 字段选择使用哪种。

## 系统接口目录

对于 gRPC 和 ROS 2 传输，服务端维护一份系统接口目录（`ROBO_SYSTEM_INTERFACE_CATALOG`），只有目录中列出的 abstract_interface_id 才允许注册。这保证了 `robonix/` 命名空间下的接口路径是受控的，不会出现拼写错误或未经设计的路径。

MCP 传输不受此约束——它主要用于工具暴露，路径可以自由定义。

目前目录中包含的路径：

```
robonix/prm/base/{cancel_nav, goal_status, joint_state, move, nav_status, navigate, odom, pose_cov, stop}
robonix/prm/camera/{depth, intrinsics, ir, rgb, rgbd}
robonix/prm/sensor/{imu, lidar, pointcloud}
robonix/sys/model/vlm/{chat, describe}
```

## node_id

每个注册节点必须使用 reverse-DNS 格式的 `node_id`，至少三段，各段只含 ASCII 字母数字和下划线。例如 `com.robonix.prm.tiago`、`org.example.vlm_service`、`cn.robonix.runtime.agent`。空 `node_id` 由服务端分配为 `com.robonix.ephemeral.<uuid>`。同一时刻两个不同进程不能使用相同的 `node_id`。

## NegotiateChannel 流程

消费者获取数据面端点的完整步骤：

1. 调用 `QueryNodes`，按 `abstract_interface_id` 或 `namespace` + `transport` 找到 provider
2. 从返回的 `NodeInfo.interfaces` 中选择目标接口和传输
3. 调用 `NegotiateChannel`，传入 `consumer_id`、`provider_node_id`、`interface_name`、`transport`
4. 服务端返回 `channel_id`、`endpoint` 和 `metadata_json`（包含 provider 的缓冲区元数据，如几何信息、CUDA IPC handle 等）
5. 消费者用 `endpoint` 直接建立数据面连接，用 `metadata_json` 配置缓冲区参数

通道使用完毕后调用 `ReleaseChannel` 释放。节点注销（`UnregisterNode`）时关联的所有通道自动释放。
