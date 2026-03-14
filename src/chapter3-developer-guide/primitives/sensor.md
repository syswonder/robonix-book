# 通用传感器 (Sensor)

命名空间：`robonix/prm/sensor`

## 概述

通用传感器原语涵盖点云、激光雷达、IMU 等，与相机、底盘分离，便于厂商按传感器类型独立实现。

## 接口列表

| 接口 | 原语类型 | 载荷 | 说明 |
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
