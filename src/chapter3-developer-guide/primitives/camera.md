# 相机 (Camera)

命名空间：`robonix/prm/camera`

视觉传感器对外暴露的数据接口。厂商根据硬件支持的模式选择实现子集。

## 接口列表

| 接口 | 方向 | 载荷 | 说明 |
|------|------|------|------|
| `rgb` | 输出 | `sensor_msgs/msg/Image` | RGB 图像 |
| `depth` | 输出 | `sensor_msgs/msg/Image` | 深度图（16-bit 或 32-bit） |
| `rgbd` | 输出 | `robonix_msg/msg/RGBD` | RGB + 深度同步输出 |
| `ir` | 输出 | `sensor_msgs/msg/Image` | 红外图像 |
| `intrinsics` | 输出 | `sensor_msgs/msg/CameraInfo` | 相机内参 |

## 典型组合

| 硬件类型 | 建议实现 |
|----------|----------|
| 仅 RGB 相机 | rgb, intrinsics |
| 纯深度相机 | depth（+ 可选 intrinsics） |
| RGB-D 相机 | rgb, depth, rgbd, intrinsics |
| 带红外相机 | rgb, ir, intrinsics（可选 depth） |
