# 硬件/服务厂商接入指南

本指南面向需要将硬件驱动或服务接入 Robonix 的厂商。


## 1. 接入模式

厂商节点是一个独立进程，通过 gRPC 注册到 `robonix-server`：

1. 调用 `RegisterNode` 注册自身（`namespace`、`kind`）。
2. 调用 `DeclareInterface` 声明提供的接口及支持的传输类型。
3. 在主循环中通过原生 API 提供数据或服务。

厂商只需实现与本机硬件能力匹配的接口子集。


## 2. 相机厂商

典型接口：

| 接口名 | 传输类型 | 说明 |
|--------|----------|------|
| `rgb` | `ros2`, `shared_memory` | RGB 图像 |
| `depth` | `ros2`, `shared_memory` | 深度图 |
| `intrinsics` | `grpc` | 相机内参 |

```python
stub.RegisterNode(pb.RegisterNodeRequest(
    node_id="realsense-d435", namespace="robonix/prm/camera", kind="primitive"))
stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id="realsense-d435", name="rgb",
    supported_transports=["ros2", "shared_memory"]))
stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id="realsense-d435", name="depth",
    supported_transports=["ros2", "shared_memory"]))
```


## 3. 机械臂 / 夹爪厂商

典型接口：`move_joint`、`move_ee`、`state_joint`、`open`、`close`。

按硬件能力选择需要实现的子集。


## 4. 底盘厂商

典型接口：`cmd_vel`（接收速度指令）、`odom`（发布里程计）。


## 5. 服务（语义地图、VLM 等）

注册为 `kind: "service"`，声明 `grpc` 传输接口，并附带 SKILL.md。

```python
stub.RegisterNode(pb.RegisterNodeRequest(
    node_id="vlm-service", namespace="robonix/system/vlm",
    kind="service", skill_md=open("SKILL.md").read()))
stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id="vlm-service", name="describe",
    supported_transports=["grpc"]))
```


## 6. 参考

- 接口清单：[抽象硬件原语](primitives/index.md)
- 开发流程：[节点开发指南](package-development.md)
- [RFC003: 控制平面设计](../rfc/RFC003-Control-Plane.md)
