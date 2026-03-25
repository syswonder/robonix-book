# 相机 robonix/prm/camera

相机原语定义 RGB、深度、红外图像流和内参输出。图像数据的 payload 类型直接使用 `sensor_msgs/msg/Image.msg`（来自 `common_interfaces`），streaming RPC 指令定义在 `rust/robonix-interfaces/lib/prm_camera/srv/`。

## 接口

| abstract_interface_id | 模式 | 方向 | ROS IDL | gRPC 映射 |
|-----------------------|------|------|---------|-----------|
| `robonix/prm/camera/rgb` | pub-sub | output | `sensor_msgs/msg/Image.msg` | server-streaming `PrmCameraService/SubscribeRgb` |
| `robonix/prm/camera/depth` | pub-sub | output | `sensor_msgs/msg/Image.msg` | 同上模式 |
| `robonix/prm/camera/ir` | pub-sub | output | `sensor_msgs/msg/Image.msg` | 同上模式 |
| `robonix/prm/camera/intrinsics` | pub-sub | output | `sensor_msgs/msg/CameraInfo.msg` | `message CameraInfo` |
| `robonix/prm/camera/rgbd` | pub-sub | output | `robonix_msg/msg/RGBD.msg` | `robonix_msg.RGBD` |

gRPC streaming RPC 通过 `.srv` 文件首行的 `# @robonix.grpc stream_server sensor_msgs/msg/Image` 指令生成。ROS 2 端使用普通 topic。

## 典型组合

最常见的配置是 `rgb`（大多数 VLM 需要 RGB 图像）。带深度传感器的相机再加上 `depth`，立体相机或 RGB-D 传感器可以用 `rgbd` 一次性输出。`intrinsics` 对需要 3D 重建的下游算法有用。

## Tiago 桥接实现

`tiago_bridge/node.py` 为 `rgb` 端口同时注册了 gRPC（server-stream）和 ROS 2（topic republish）两种传输。gRPC 端实现 `PrmCameraServicer.SubscribeRgb`，以指定帧率从 ROS 2 缓存的最新帧中推送 `sensor_msgs.Image`。ROS 2 端将控制平面分配的 topic 名作为 republish 目标。
