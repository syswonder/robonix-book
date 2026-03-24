# 通用传感器 (Sensor)

命名空间：`robonix/prm/sensor`

非相机类传感器数据接口。

## 接口列表

| 接口 | 方向 | 载荷 | 说明 |
|------|------|------|------|
| `pointcloud` | 输出 | `sensor_msgs/msg/PointCloud2` | 3D 点云 |
| `lidar` | 输出 | `sensor_msgs/msg/LaserScan` | 2D 激光扫描 |
| `imu` | 输出 | `sensor_msgs/msg/Imu` | 惯性测量单元 |

## 典型组合

| 硬件 | 建议实现 |
|------|----------|
| 3D 激光雷达 | pointcloud |
| 2D 激光雷达 | lidar |
| IMU | imu |
