# 空间地图 robonix/service/map

`map` 服务是 SLAM / 定位的输出面：把建图算法（FAST-LIO2、cartographer、AMCL 等）产生的占据栅格、map 帧位姿、全局点云、融合里程统一成 `robonix/service/map/*` 能力约定。导航、scene、可视化都从这里取定位结果。

能力约定 TOML 在 `capabilities/service/map/`，IDL 在 `capabilities/lib/common_interfaces/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/service/map/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `service/map/driver.v1.toml` |
| `robonix/service/map/occupancy_grid` | `topic_out` | [`nav_msgs/OccupancyGrid`](../../reference/idl.md#common-interfaces-nav-msgs-msg-occupancygrid-msg) | `service/map/occupancy_grid.v1.toml` |
| `robonix/service/map/pose` | `topic_out` | [`geometry_msgs/PoseWithCovarianceStamped`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-posewithcovariancestamped-msg) | `service/map/pose.v1.toml` |
| `robonix/service/map/odom` | `topic_out` | [`nav_msgs/Odometry`](../../reference/idl.md#common-interfaces-nav-msgs-msg-odometry-msg) | `service/map/odom.v1.toml` |
| `robonix/service/map/pointcloud` | `topic_out` | [`sensor_msgs/PointCloud2`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-pointcloud2-msg) | `service/map/pointcloud.v1.toml` |
| `robonix/service/map/save_map` | `rpc` | [`map/SaveMap`](../../reference/idl.md#map-srv-savemap-srv) | `service/map/save_map.v1.toml` |
| `robonix/service/map/load_map` | `rpc` | [`map/LoadMap`](../../reference/idl.md#map-srv-loadmap-srv) | `service/map/load_map.v1.toml` |
| `robonix/service/map/delete_map` | `rpc` | [`map/DeleteMap`](../../reference/idl.md#map-srv-deletemap-srv) | `service/map/delete_map.v1.toml` |
| `robonix/service/map/reset_map` | `rpc` | [`map/ResetMap`](../../reference/idl.md#map-srv-resetmap-srv) | `service/map/reset_map.v1.toml` |
| `robonix/service/map/switch_mode` | `rpc` | [`map/SwitchMode`](../../reference/idl.md#map-srv-switchmode-srv) | `service/map/switch_mode.v1.toml` |
| `robonix/service/map/get_mode` | `rpc` | [`map/GetMode`](../../reference/idl.md#map-srv-getmode-srv) | `service/map/get_mode.v1.toml` |
| `robonix/service/map/get_pose` | `rpc` | [`map/GetPose`](../../reference/idl.md#map-srv-getpose-srv) | `service/map/get_pose.v1.toml` |
| `robonix/service/map/pose_estimate` | `rpc` | [`map/PoseEstimate`](../../reference/idl.md#map-srv-poseestimate-srv) | `service/map/pose_estimate.v1.toml` |

`pose` 是 **map 帧**的机器人位姿（融合定位结果）——和底盘原语的 `odom`（odom 帧、未消除漂移）分工不同：要 map 帧位姿找 `service/map/pose`，要瞬时里程找 `primitive/chassis/odom`（见 [底盘](../primitive/chassis.md)）。

输入侧：map 服务消费 `primitive/lidar/lidar3d` + `primitive/imu/imu`（或 2D `lidar` + `chassis/odom`，可选 `camera/rgb` + `camera/depth` 做视觉融合），通过 atlas 按能力约定发现，不硬编码 topic 名。

参考实现：[`service-map-rbnx`](https://github.com/syswonder/service-map-rbnx)（上游仓库）默认运行 RTAB-Map 图优化 SLAM。Robot deployment 保存完整 `config/rtabmap_params.yaml`，在 Mapping 条目的 `config.params_file` 中引用；共享仓库只提供 `config/rtabmap_params.template.yaml`。`sensor_providers` 按角色绑定 `lidar2d`/`lidar3d`/`rgb`/`depth`/`imu`/`odom` provider，避免多传感器本体选错实例。`map_id` 开启命名地图持久化，`map_mode: localization` 加载已存地图重定位并保持 map 帧原点跨重启稳定。

> 文档参考：[README（部署、`params_file`、传感器绑定与兼容迁移）](https://github.com/syswonder/service-map-rbnx/blob/main/README.md) · [`config.spec`（全部 instance 配置、类型、默认值与约束）](https://github.com/syswonder/service-map-rbnx/blob/main/config.spec) · [CAPABILITY.md（能力面与地图持久化）](https://github.com/syswonder/service-map-rbnx/blob/main/CAPABILITY.md)

RTAB-Map 参数名、默认值和上游说明以 [`Parameters.h`](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h) 为准。以 service 提供的 template 为起点，在 deployment 的 YAML 中增加或修改本体所需参数；运行时只读取这份 deployment 文件，不隐式叠加上游配置。
