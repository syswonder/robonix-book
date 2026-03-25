# 传感器 robonix/prm/sensor

通用传感器原语覆盖点云、激光雷达和 IMU。这些接口的 payload 类型均来自 `common_interfaces` 中的 `sensor_msgs`。

## 接口

| abstract_interface_id | 模式 | 方向 | ROS IDL | gRPC 类型 |
|-----------------------|------|------|---------|-----------|
| `robonix/prm/sensor/pointcloud` | pub-sub | output | `sensor_msgs/msg/PointCloud2.msg` | `message PointCloud2` |
| `robonix/prm/sensor/lidar` | pub-sub | output | `sensor_msgs/msg/LaserScan.msg` | `message LaserScan` |
| `robonix/prm/sensor/imu` | pub-sub | output | `sensor_msgs/msg/Imu.msg` | `message Imu` |

## 典型组合

带激光雷达的移动机器人通常实现 `lidar`，配合 SLAM 和导航使用。工业场景可能额外提供 `pointcloud`（如 3D LiDAR 或深度相机点云）。`imu` 在有惯性导航需求时实现。
