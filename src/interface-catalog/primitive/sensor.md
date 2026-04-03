# 传感器 robonix/prm/sensor

通用传感器原语覆盖点云、激光雷达和 IMU。这些接口的 payload 类型均来自 `common_interfaces` 中的 `sensor_msgs`。

## 接口

| 契约 ID（`contract_id`） | 模式 | 方向 | ROS IDL | gRPC 类型 | 契约源码（TOML） |
|-----------------------|------|------|---------|-----------|------------------|
| `robonix/prm/sensor/pointcloud` | pub-sub | output | `sensor_msgs/msg/PointCloud2.msg` | `message PointCloud2` | — |
| `robonix/prm/sensor/lidar` | pub-sub | output | `sensor_msgs/msg/LaserScan.msg` | `message LaserScan` | `rust/contracts/prm/sensor_lidar.v1.toml` |
| `robonix/prm/sensor/imu` | pub-sub | output | `sensor_msgs/msg/Imu.msg` | `message Imu` | `rust/contracts/prm/sensor_imu.v1.toml` |
| `robonix/prm/sensor/lidar_snapshot` | rpc | — | `std_msgs/Empty` → `sensor_msgs/LaserScan` | 单次扫描（与连续 `lidar` 流互补；MCP 常用） | `rust/contracts/prm/sensor_lidar_snapshot.v1.toml` |

## 典型组合

带激光雷达的移动机器人通常实现 `lidar`，配合 SLAM 和导航使用。工业场景可能额外提供 `pointcloud`（如 3D LiDAR 或深度相机点云）。`imu` 在有惯性导航需求时实现。
