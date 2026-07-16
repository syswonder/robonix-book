---
title: 相机
---
<span id="相机-robonixprimitivecamera"></span>
# 相机

相机原语覆盖 RGB 与深度图像，两种取图方式并存：**流式**（`rgb` / `depth`）给场景融合、建图等高频消费者使用；**快照**（`snapshot` / `depth_snapshot`，一元 RPC）供大模型智能体按需取一帧。`topic_out` 只描述单向输出流，不绑定具体传输方式；当前 Webots 提供方通过 ROS 2 发布，Scene 当前也只接入 ROS 2 数据面。当前 Scene 优先组合 Atlas 发现的地图位姿与 `camera/extrinsics`，只在该路径不可用时回退到 tf2。

能力约定 TOML 在 `capabilities/primitive/camera/`，接口定义语言（Interface Definition Language，IDL）文件在 `capabilities/lib/camera/` 与 `capabilities/lib/common_interfaces/`。

## 接口

| 能力约定 ID（`contract_id`） | 模式 | 载荷（IDL） | 能力约定 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/primitive/camera/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/camera/driver.v1.toml` |
| `robonix/primitive/camera/rgb` | `topic_out` | [`sensor_msgs/Image`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-image-msg) | `primitive/camera/rgb.v1.toml` |
| `robonix/primitive/camera/depth` | `topic_out` | [`sensor_msgs/Image`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-image-msg) | `primitive/camera/depth.v1.toml` |
| `robonix/primitive/camera/extrinsics` | `topic_out` | [`geometry_msgs/TransformStamped`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-transformstamped-msg) | `primitive/camera/extrinsics.v1.toml` |
| `robonix/primitive/camera/intrinsics` | `topic_out` | [`sensor_msgs/CameraInfo`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-camerainfo-msg) | `primitive/camera/intrinsics.v1.toml` |
| `robonix/primitive/camera/snapshot` | `rpc` | [`camera/GetCameraImage`](../../reference/idl.md#camera-srv-getcameraimage-srv) | `primitive/camera/snapshot.v1.toml` |
| `robonix/primitive/camera/depth_snapshot` | `rpc` | [`camera/GetCameraImage`](../../reference/idl.md#camera-srv-getcameraimage-srv) | `primitive/camera/depth_snapshot.v1.toml` |

`snapshot` / `depth_snapshot` 共用 `camera/GetCameraImage` 线协议结构，但使用两个不同的能力约定 ID 选择 RGB 或深度图；两者的请求均为空，应答为 `sensor_msgs/Image`。`rgb` / `depth` 是给 Scene、Mapping 等系统消费者的高频数据面。当前 Scene 选择 ROS 2 传输；其它消费者或提供方也可以为同一 `topic_out` 模式注册 gRPC 流。

`intrinsics` 提供相机内参。提供方应使用“瞬态本地（`TRANSIENT_LOCAL`）+ 可靠（`RELIABLE`）”的服务质量（Quality of Service，QoS）配置发布，并在启动和重标定后发送；消息必须描述与 RGB 和深度数据相同的光学坐标系。这样晚启动的 Scene 才能取得标定，并把深度像素放进世界坐标系。

`extrinsics` 使用相同的持久可靠 QoS。当前实现把它作为主路径：Scene 组合 `T(world ← base)` 与该能力约定提供的 `T(base ← camera_optical)`，当 `extrinsics` 未声明时才查询 tf2。[Issue #156](https://github.com/syswonder/robonix/issues/156) 提议未来反转顺序：由完整 URDF 发布的 TF 作为权威来源，`extrinsics` 只作兼容回退，并删除旧 Soma 外参接口。该目标尚未实现，接入时不能把它当作当前运行时行为。

参考实现：`examples/webots/primitives/tiago_camera`。其软件包清单列出表中全部 7 条能力约定，驱动在初始化后注册 RGB、深度和两项标定数据面。

该实现的 `snapshot` 把 JPEG 字节放进 `sensor_msgs/Image.data`，`depth_snapshot` 还会把深度归一化为灰度 JPEG；这是 Tiago 示例的编码特例，不是能力约定保证，也不保留米制深度。需要原始像素布局或度量深度的消费者应使用 `rgb` / `depth` 数据面，并检查 `Image.encoding`。
