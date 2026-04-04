# 相机 robonix/prm/camera

相机原语定义图像流和快照接口。图像载荷使用 `sensor_msgs/msg/Image.msg`，streaming RPC 定义在 `rust/crates/robonix-interfaces/lib/prm_camera/srv/`。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/prm/camera/rgb` | `pub-sub (out)` / `rpc_server_stream` | `sensor_msgs/Image` | `prm/camera_rgb.v1.toml` |
| `robonix/prm/camera/depth` | `pub-sub (out)` / `rpc_server_stream` | `sensor_msgs/Image` | `prm/camera_depth.v1.toml` |
| `robonix/prm/camera/ir` | `pub-sub (out)` | `sensor_msgs/Image` | — |
| `robonix/prm/camera/intrinsics` | `pub-sub (out)` | `sensor_msgs/CameraInfo` | — |
| `robonix/prm/camera/rgbd` | `pub-sub (out)` | `robonix_msg/RGBD` | — |
| `robonix/prm/camera/snapshot` | `rpc` | `std_msgs/Empty` → `sensor_msgs/Image` | `prm/camera_snapshot.v1.toml` |
| `robonix/prm/camera/depth_snapshot` | `rpc` | `std_msgs/Empty` → `sensor_msgs/Image` | `prm/camera_depth_snapshot.v1.toml` |
| `robonix/prm/perception/detect` | `rpc` | `sensor_msgs/Image` → `std_msgs/String` | `prm/perception_detect.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。`rgb`/`depth` 同时支持 ROS 2 topic（`pub-sub`）和 gRPC server-stream 两种传输，可在 `DeclareInterface` 时分别声明。

`snapshot` 类接口在 MCP 工具中最常用：以 `std_msgs/Empty` 为输入触发单帧采集，返回 `sensor_msgs/Image`（JSON 中 `data` 为 base64）。当工具结果含 `width`/`height`/`encoding`/`data` 字段时，Pilot 会将图像以多模态形式送入 VLM。

## Tiago 桥接实现

`tiago_bridge/node.py` 为 `rgb` 同时注册 gRPC（server-stream `SubscribeRgb`）和 ROS 2（topic republish）两种传输；`camera_snapshot` 等 MCP 工具通过 `mcp_contract` 与表中契约 ID 对齐。

## 典型组合

最常见的配置是 `rgb`（VLM 视觉推理必需）。带深度传感器的相机加 `depth`，RGB-D 传感器可用 `rgbd`。`intrinsics` 供需要 3D 重建的下游算法使用。
