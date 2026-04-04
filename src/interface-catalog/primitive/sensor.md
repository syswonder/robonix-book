# 传感器 robonix/prm/sensor

通用传感器原语覆盖激光雷达、点云和 IMU。载荷类型来自 `common_interfaces` 中的 `sensor_msgs`。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/prm/sensor/lidar` | `pub-sub (out)` | `sensor_msgs/LaserScan` | `prm/sensor_lidar.v1.toml` |
| `robonix/prm/sensor/pointcloud` | `pub-sub (out)` | `sensor_msgs/PointCloud2` | — |
| `robonix/prm/sensor/imu` | `pub-sub (out)` | `sensor_msgs/Imu` | `prm/sensor_imu.v1.toml` |
| `robonix/prm/sensor/lidar_snapshot` | `rpc` | `std_msgs/Empty` → `sensor_msgs/LaserScan` | `prm/sensor_lidar_snapshot.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。

## 典型组合

带激光雷达的移动机器人通常实现 `lidar`，配合 SLAM 和导航使用。需要按需查询单次扫描时用 `lidar_snapshot`（MCP 友好）。工业场景可额外提供 `pointcloud`（3D LiDAR 或深度相机点云）。`imu` 在有惯性导航需求时实现。
