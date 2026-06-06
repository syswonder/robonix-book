# 相机 robonix/primitive/camera

相机原语覆盖 RGB 与深度图像，两种取图方式并存：**流式**（`rgb` / `depth`，ROS 2 topic 持续发布）给场景融合、建图等高频消费者用；**快照**（`snapshot` / `depth_snapshot`，一元 RPC）给 LLM agent 按需取一帧用。`extrinsics` 发布相机相对本体的外参，供深度反投影到世界系。

契约 TOML 在 `capabilities/primitive/camera/`，IDL 在 `capabilities/lib/camera/` 与 `capabilities/lib/common_interfaces/`。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/primitive/camera/driver` | `rpc` | `lifecycle/Driver` | `primitive/camera/driver.v1.toml` |
| `robonix/primitive/camera/rgb` | `topic_out` | `sensor_msgs/Image` | `primitive/camera/rgb.v1.toml` |
| `robonix/primitive/camera/depth` | `topic_out` | `sensor_msgs/Image` | `primitive/camera/depth.v1.toml` |
| `robonix/primitive/camera/extrinsics` | `topic_out` | `geometry_msgs/TransformStamped` | `primitive/camera/extrinsics.v1.toml` |
| `robonix/primitive/camera/snapshot` | `rpc` | `camera/GetCameraImage` | `primitive/camera/snapshot.v1.toml` |
| `robonix/primitive/camera/depth_snapshot` | `rpc` | `camera/GetCameraImage` | `primitive/camera/depth_snapshot.v1.toml` |

`snapshot` / `depth_snapshot` 共用 `camera/GetCameraImage` 服务（请求选通道，应答回 `sensor_msgs/Image`），是 agent 按需取一帧的入口（一元 RPC）。`rgb` / `depth` 是给 scene、mapping 等系统消费者的高频数据面，走 ROS 2。

参考实现：`examples/webots/primitives/tiago_camera`（订阅 Webots `/head_front_camera/*`，桥成上面五条契约）。
