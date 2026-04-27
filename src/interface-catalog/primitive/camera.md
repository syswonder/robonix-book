# 相机 robonix/primitive/camera

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/primitive/camera/rgb` | `pub-sub (out)` / `rpc_server_stream` | `sensor_msgs/Image` | `primitive/camera_rgb.v1.toml` |
| `robonix/primitive/camera/depth` | `pub-sub (out)` / `rpc_server_stream` | `sensor_msgs/Image` | `primitive/camera_depth.v1.toml` |
| `robonix/primitive/camera/ir` | `pub-sub (out)` | `sensor_msgs/Image` | — |
| `robonix/primitive/camera/intrinsics` | `pub-sub (out)` | `sensor_msgs/CameraInfo` | — |
| `robonix/primitive/camera/rgbd` | `pub-sub (out)` | `robonix_msg/RGBD` | — |
| `robonix/primitive/camera/snapshot` | `rpc` | `std_msgs/Empty` → `sensor_msgs/Image` | `primitive/camera_snapshot.v1.toml` |
| `robonix/primitive/camera/depth_snapshot` | `rpc` | `std_msgs/Empty` → `sensor_msgs/Image` | `primitive/camera_depth_snapshot.v1.toml` |
