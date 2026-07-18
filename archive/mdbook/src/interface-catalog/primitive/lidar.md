# 激光雷达 robonix/primitive/lidar

激光雷达原语覆盖 2D 扫描与 3D 点云。`lidar`（2D `LaserScan`）和 `lidar3d`（`PointCloud2`）都是 `topic_out` 数据面，给建图（`service/map`）、避障、场景融合用；`snapshot` 给 LLM agent 按需取一帧 2D 扫描。

能力约定 TOML 在 `capabilities/primitive/lidar/`，IDL 在 `capabilities/lib/lidar/` 与 `capabilities/lib/common_interfaces/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/primitive/lidar/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/lidar/driver.v1.toml` |
| `robonix/primitive/lidar/lidar` | `topic_out` | [`sensor_msgs/LaserScan`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-laserscan-msg) | `primitive/lidar/lidar.v1.toml` |
| `robonix/primitive/lidar/lidar3d` | `topic_out` | [`sensor_msgs/PointCloud2`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-pointcloud2-msg) | `primitive/lidar/lidar3d.v1.toml` |
| `robonix/primitive/lidar/snapshot` | `rpc` | [`lidar/GetLaserScan`](../../reference/idl.md#lidar-srv-getlaserscan-srv) | `primitive/lidar/lidar_snapshot.v1.toml` |

一个雷达包按硬件实现 `lidar`（2D 雷达）、`lidar3d`（3D 雷达，如 Livox / Velodyne）或两者。**vendor 中立**：原语只暴露标准 `LaserScan` / `PointCloud2`，厂家 SDK 的私有点云格式应在驱动内转成标准消息，不外泄到能力约定。

参考实现：`examples/webots/primitives/tiago_lidar`（`/scanner` → `lidar` + `snapshot`，输出归一化的 `/scanner_normalized`）。
