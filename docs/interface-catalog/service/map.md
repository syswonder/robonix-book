<span id="空间地图-robonixservicemap"></span>
# 空间地图

`map` 服务是算法无关的 SLAM / 定位输出面：把占据栅格、map 帧位姿、全局点云、融合里程和地图生命周期统一成 `robonix/service/map/*` 能力约定。导航、scene 和可视化按能力约定消费这些结果，不依赖具体 SLAM 引擎。

能力约定 TOML 在 `capabilities/service/map/`；直接 IDL 位于 `capabilities/lib/{map,lifecycle,common_interfaces}/`。

## 接口

| 能力约定 ID | 模式 | `main@ce4092a` 注册 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/map/driver` | `rpc` | gRPC（生命周期） | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `service/map/driver.v1.toml` |
| `robonix/service/map/lifecycle` | `topic_out` | ROS 2 topic（transient-local） | [`map/MapLifecycle`](../../reference/idl.md#map-msg-maplifecycle-msg) | `service/map/lifecycle.v1.toml` |
| `robonix/service/map/occupancy_grid` | `topic_out` | ROS 2 topic | [`nav_msgs/OccupancyGrid`](../../reference/idl.md#common-interfaces-nav-msgs-msg-occupancygrid-msg) | `service/map/occupancy_grid.v1.toml` |
| `robonix/service/map/pose` | `topic_out` | ROS 2 topic | [`geometry_msgs/PoseWithCovarianceStamped`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-posewithcovariancestamped-msg) | `service/map/pose.v1.toml` |
| `robonix/service/map/odom` | `topic_out` | ROS 2 topic | [`nav_msgs/Odometry`](../../reference/idl.md#common-interfaces-nav-msgs-msg-odometry-msg) | `service/map/odom.v1.toml` |
| `robonix/service/map/pointcloud` | `topic_out` | ROS 2 topic | [`sensor_msgs/PointCloud2`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-pointcloud2-msg) | `service/map/pointcloud.v1.toml` |
| `robonix/service/map/save_map` | `rpc` | gRPC + MCP | [`map/SaveMap`](../../reference/idl.md#map-srv-savemap-srv) | `service/map/save_map.v1.toml` |
| `robonix/service/map/load_map` | `rpc` | gRPC + MCP | [`map/LoadMap`](../../reference/idl.md#map-srv-loadmap-srv) | `service/map/load_map.v1.toml` |
| `robonix/service/map/delete_map` | `rpc` | gRPC + MCP（manifest 漏列） | [`map/DeleteMap`](../../reference/idl.md#map-srv-deletemap-srv) | `service/map/delete_map.v1.toml` |
| `robonix/service/map/reset_map` | `rpc` | gRPC + MCP（manifest 漏列） | [`map/ResetMap`](../../reference/idl.md#map-srv-resetmap-srv) | `service/map/reset_map.v1.toml` |
| `robonix/service/map/switch_mode` | `rpc` | gRPC + MCP | [`map/SwitchMode`](../../reference/idl.md#map-srv-switchmode-srv) | `service/map/switch_mode.v1.toml` |
| `robonix/service/map/get_mode` | `rpc` | gRPC + MCP（manifest 漏列） | [`map/GetMode`](../../reference/idl.md#map-srv-getmode-srv) | `service/map/get_mode.v1.toml` |
| `robonix/service/map/get_pose` | `rpc` | gRPC + MCP（manifest 漏列） | [`map/GetPose`](../../reference/idl.md#map-srv-getpose-srv) | `service/map/get_pose.v1.toml` |
| `robonix/service/map/pose_estimate` | `rpc` | gRPC + MCP | [`map/PoseEstimate`](../../reference/idl.md#map-srv-poseestimate-srv) | `service/map/pose_estimate.v1.toml` |
| `robonix/service/map/list_maps` | `rpc` | gRPC + MCP | [`map/ListMaps`](../../reference/idl.md#map-srv-listmaps-srv) | `service/map/list_maps.v1.toml` |

`pose` 是 **map 帧**的机器人位姿（融合定位结果）——和底盘原语的 `odom`（odom 帧、未消除漂移）分工不同：要 map 帧位姿找 `service/map/pose`，要瞬时里程找 `primitive/chassis/odom`（见 [底盘](../primitive/chassis.md)）。

输入侧由 `sensor_providers` 选择并绑定 `lidar2d`、`lidar3d`、`rgb`、`depth`、`imu`、`odom` 角色；只解析实际绑定的 Atlas 提供方。默认 RTAB-Map 可使用二维或三维激光雷达、RGB-D 和可选外部里程计；DLIO 路径需要三维激光雷达和 IMU。`fastlio2` 在固定快照中明确标为漂移故障，仅保留用于复现。

参考实现固定为 [`service-map-rbnx main@ce4092a`](https://github.com/syswonder/service-map-rbnx/tree/ce4092a1bee8847d6314af957f0225c8371d9aa6)。`algo` 默认并推荐 `rtabmap`，也可选择 `dlio`；机器人部署保存完整 `config/rtabmap_params.yaml`，并在建图服务的 `config.params_file` 中引用。共享仓库模板不会在运行时隐式加载。命名地图持久化目前只支持 RTAB-Map：`map_mode: mapping` 建图后调用 `save_map(map_id)`，`map_mode: localization` 配合已有 `map_id` 加载不可变副本并保持地图坐标系稳定。

输出 topic 的适配面覆盖各个 `algo`，但地图数据库、`pose_estimate`、模式切换和 reset 的实现直接调用 RTAB-Map 数据库或 ROS service。选择 DLIO 时不能假设这些管理 RPC 可用，应按响应中的 `ok` / `detail` 处理失败。

> 固定快照文档：[README（部署、`params_file`、传感器绑定与兼容迁移）](https://github.com/syswonder/service-map-rbnx/blob/ce4092a1bee8847d6314af957f0225c8371d9aa6/README.md) · [`config.spec`（全部 instance 配置、类型、默认值与约束）](https://github.com/syswonder/service-map-rbnx/blob/ce4092a1bee8847d6314af957f0225c8371d9aa6/config.spec) · [实现注册](https://github.com/syswonder/service-map-rbnx/blob/ce4092a1bee8847d6314af957f0225c8371d9aa6/src/mapping_rbnx/atlas_bridge.py)

RTAB-Map 参数名、默认值和上游说明以 [`Parameters.h`](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h) 为准。以 service 提供的 template 为起点，在 deployment 的 YAML 中增加或修改本体所需参数；运行时只读取这份 deployment 文件，不隐式叠加上游配置。

> 上游已知缺陷：三个 package manifest 都漏列 `get_mode`、`get_pose`、`delete_map`、`reset_map`，但实现已同时注册其 gRPC servicer 和 MCP 工具。依赖 manifest 枚举能力的工具看不到这四项；调用方应先从 Atlas 确认实际注册状态。
