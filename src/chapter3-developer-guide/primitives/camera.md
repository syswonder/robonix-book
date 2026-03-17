# 相机 (Camera)

命名空间：`robonix/prm/camera`

## 概述

相机原语抽象视觉传感器，支持 RGB、深度、RGB-D、红外及内参等接口。厂商根据硬件能力实现子集：仅 RGB 相机实现 `rgb`、`intrinsics`；纯深度相机只实现 `depth`（可选 `intrinsics`）；RGB-D 相机实现 `rgb`、`depth`、`rgbd`、`intrinsics`。

## 接口列表

| 接口 | 原语类型 | 载荷 | 说明 |
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
