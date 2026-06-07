# 空间地图 robonix/service/map

`map` 服务是 SLAM / 定位的输出面：把建图算法（FAST-LIO2、cartographer、AMCL 等）产生的占据栅格、map 帧位姿、全局点云、融合里程统一成 `robonix/service/map/*` 能力约定。导航、scene、可视化都从这里取定位结果。

能力约定 TOML 在 `capabilities/service/map/`，IDL 在 `capabilities/lib/common_interfaces/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/service/map/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `service/map/driver.v1.toml` |
| `robonix/service/map/occupancy_grid` | `topic_out` | [`nav_msgs/OccupancyGrid`](../../reference/idl.md#common-interfaces-nav-msgs-msg-occupancygrid-msg) | `service/map/occupancy_grid.v1.toml` |
| `robonix/service/map/pose` | `topic_out` | [`geometry_msgs/PoseWithCovarianceStamped`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-posewithcovariancestamped-msg) | `service/map/pose.v1.toml` |
| `robonix/service/map/pointcloud` | `topic_out` | [`sensor_msgs/PointCloud2`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-pointcloud2-msg) | `service/map/pointcloud.v1.toml` |
| `robonix/service/map/odom` | `topic_out` | [`nav_msgs/Odometry`](../../reference/idl.md#common-interfaces-nav-msgs-msg-odometry-msg) | `service/map/odom.v1.toml` |

`pose` 是 **map 帧**的机器人位姿（融合定位结果）——和底盘原语的 `odom`（odom 帧、未消除漂移）分工不同：要 map 帧位姿找 `service/map/pose`，要瞬时里程找 `primitive/chassis/odom`（见 [底盘](../primitive/chassis.md)）。

输入侧：map 服务消费 `primitive/lidar/lidar3d` + `primitive/imu/imu`（或 2D `lidar` + `chassis/odom`），通过 atlas 按能力约定发现，不硬编码 topic 名。
