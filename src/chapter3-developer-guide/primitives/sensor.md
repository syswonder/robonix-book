# 通用传感器 (Sensor)

命名空间：`robonix/prm/sensor`

## 概述

通用传感器原语用于承载**非相机类**、且不便归入底盘运动学的传感数据，例如三维点云、二维激光扫描与 IMU。它与相机、底盘等命名空间相互独立，便于厂商按传感器类型拆分驱动，并在上层组合使用。

## 接口列表

| 接口 | 通信语义 | 载荷 | 说明 |
|------|----------|------|------|
| `pointcloud` | stream | `sensor_msgs/msg/PointCloud2` | 3D 点云 |
| `lidar` | stream | `sensor_msgs/msg/LaserScan` | 2D 激光扫描 |
| `imu` | stream | `sensor_msgs/msg/Imu` | 惯性测量单元（加速度、角速度、姿态） |

## 典型组合

| 硬件 | 建议实现 |
|------|----------|
| 3D 激光雷达 | pointcloud |
| 2D 激光雷达 | lidar |
| IMU | imu |
| 深度相机 + 点云 | 由 camera 提供 depth，或单独实现 pointcloud |
