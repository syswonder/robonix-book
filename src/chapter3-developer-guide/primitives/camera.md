# 相机 (Camera)

命名空间：`robonix/prm/camera`

## 概述

相机原语描述一类**视觉传感器**对外暴露的数据流：既可以是单独的 RGB 或深度图，也可以是同步的 RGB-D，以及红外与内参等辅助流。厂商只需根据自己硬件实际支持的模式，选择实现其中一部分接口；例如纯 RGB 设备通常实现 `rgb` 与 `intrinsics`，而 RGB-D 设备往往还需要 `depth` 与 `rgbd`。

## 接口列表

| 接口 | 通信语义 | 载荷 | 说明 |
|------|----------|------|------|
| `rgb` | stream | `sensor_msgs/msg/Image` | RGB 图像流 |
| `depth` | stream | `sensor_msgs/msg/Image` | 深度图（16-bit 或 32-bit） |
| `rgbd` | stream | `robonix_msg/msg/RGBD` | RGB + 深度同步输出 |
| `ir` | stream | `sensor_msgs/msg/Image` | 红外图像 |
| `intrinsics` | stream | `sensor_msgs/msg/CameraInfo` | 相机内参（K、D、分辨率等） |

## 典型组合

| 硬件类型 | 建议实现 |
|----------|----------|
| 仅 RGB 相机 | rgb, intrinsics |
| 纯深度相机 | depth（+ 可选 intrinsics） |
| RGB-D 相机 | rgb, depth, rgbd, intrinsics |
| 带红外相机 | rgb, ir, intrinsics（可选 depth） |

## 参考系

相机输出通常以相机光学中心为原点。若需与地图/机器人坐标系对齐，应在接口或上层服务中声明 `@frame` 注解，或由调用方根据 TF 转换。
