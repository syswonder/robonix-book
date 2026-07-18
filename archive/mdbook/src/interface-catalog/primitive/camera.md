# 相机 robonix/primitive/camera

相机原语覆盖 RGB 与深度图像，两种取图方式并存：**流式**（`rgb` / `depth`，ROS 2 topic 持续发布）给场景融合、建图等高频消费者用；**快照**（`snapshot` / `depth_snapshot`，一元 RPC）给 LLM agent 按需取一帧用。`extrinsics` 发布相机相对本体的外参，供深度反投影到世界系。

能力约定 TOML 在 `capabilities/primitive/camera/`，IDL 在 `capabilities/lib/camera/` 与 `capabilities/lib/common_interfaces/`。

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

`snapshot` / `depth_snapshot` 共用 `camera/GetCameraImage` 服务（请求选通道，应答回 `sensor_msgs/Image`），是 agent 按需取一帧的入口（一元 RPC）。`rgb` / `depth` 是给 scene、mapping 等系统消费者的高频数据面，走 ROS 2。`extrinsics` / `intrinsics` 是相机几何标定面，scene 用它们把深度像素反投影并放进 map/world 坐标系。

参考实现：`examples/webots/primitives/tiago_camera`（订阅 Webots `/head_front_camera/*`，桥成上面这些能力约定）。
