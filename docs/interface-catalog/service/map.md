---
title: 空间地图
---
<span id="空间地图-robonixservicemap"></span>
# 空间地图

`map` 服务是算法无关的同步定位与建图（Simultaneous Localization and Mapping，SLAM）及定位输出面：把占据栅格、地图坐标系位姿、全局点云、融合里程和地图生命周期统一成 `robonix/service/map/*` 能力约定。导航、Scene 和可视化按能力约定消费这些结果，不依赖具体 SLAM 引擎。

能力约定 TOML 在 `capabilities/service/map/`；直接使用的接口定义语言（Interface Definition Language，IDL）文件位于 `capabilities/lib/{map,lifecycle,common_interfaces}/`。

> 表中的命名空间 Driver 是已有软件包的兼容接口。新软件包省略 Driver 条目时由框架自动使用共享的 `robonix/lifecycle/driver`；显式共享仍受支持，两种 Driver 只能选择一条。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。

## 接口

| 能力约定 ID | 模式 | 参考实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/map/driver` | `rpc` | gRPC（生命周期） | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `service/map/driver.v1.toml` |
| `robonix/service/map/lifecycle` | `topic_out` | ROS 2 话题（瞬态本地） | [`map/MapLifecycle`](../../reference/idl.md#map-msg-maplifecycle-msg) | `service/map/lifecycle.v1.toml` |
| `robonix/service/map/occupancy_grid` | `topic_out` | ROS 2 话题 | [`nav_msgs/OccupancyGrid`](../../reference/idl.md#common-interfaces-nav-msgs-msg-occupancygrid-msg) | `service/map/occupancy_grid.v1.toml` |
| `robonix/service/map/pose` | `topic_out` | ROS 2 话题 | [`geometry_msgs/PoseWithCovarianceStamped`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-posewithcovariancestamped-msg) | `service/map/pose.v1.toml` |
| `robonix/service/map/odom` | `topic_out` | ROS 2 话题（未绑定外部里程计时） | [`nav_msgs/Odometry`](../../reference/idl.md#common-interfaces-nav-msgs-msg-odometry-msg) | `service/map/odom.v1.toml` |
| `robonix/service/map/pointcloud` | `topic_out` | ROS 2 话题 | [`sensor_msgs/PointCloud2`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-pointcloud2-msg) | `service/map/pointcloud.v1.toml` |
| `robonix/service/map/save_map` | `rpc` | gRPC + 模型上下文协议（Model Context Protocol，MCP） | [`map/SaveMap`](../../reference/idl.md#map-srv-savemap-srv) | `service/map/save_map.v1.toml` |
| `robonix/service/map/load_map` | `rpc` | gRPC + MCP | [`map/LoadMap`](../../reference/idl.md#map-srv-loadmap-srv) | `service/map/load_map.v1.toml` |
| `robonix/service/map/delete_map` | `rpc` | gRPC + MCP（软件包清单漏列） | [`map/DeleteMap`](../../reference/idl.md#map-srv-deletemap-srv) | `service/map/delete_map.v1.toml` |
| `robonix/service/map/reset_map` | `rpc` | gRPC + MCP（软件包清单漏列） | [`map/ResetMap`](../../reference/idl.md#map-srv-resetmap-srv) | `service/map/reset_map.v1.toml` |
| `robonix/service/map/switch_mode` | `rpc` | gRPC + MCP | [`map/SwitchMode`](../../reference/idl.md#map-srv-switchmode-srv) | `service/map/switch_mode.v1.toml` |
| `robonix/service/map/get_mode` | `rpc` | gRPC + MCP（软件包清单漏列） | [`map/GetMode`](../../reference/idl.md#map-srv-getmode-srv) | `service/map/get_mode.v1.toml` |
| `robonix/service/map/get_pose` | `rpc` | gRPC + MCP（软件包清单漏列） | [`map/GetPose`](../../reference/idl.md#map-srv-getpose-srv) | `service/map/get_pose.v1.toml` |
| `robonix/service/map/pose_estimate` | `rpc` | gRPC + MCP | [`map/PoseEstimate`](../../reference/idl.md#map-srv-poseestimate-srv) | `service/map/pose_estimate.v1.toml` |
| `robonix/service/map/list_maps` | `rpc` | gRPC + MCP | [`map/ListMaps`](../../reference/idl.md#map-srv-listmaps-srv) | `service/map/list_maps.v1.toml` |

`robonix/service/map/pose` 是 **map 帧**的机器人位姿（融合定位结果），与底盘原语 `robonix/primitive/chassis/odom`（odom 帧、未消除漂移）分工不同。

`robonix/service/map/odom` 是建图算法自带里程计的可选输出。部署通过 `sensor_providers.odom` 绑定外部底盘里程计时，参考实现会刻意跳过这条能力声明，避免同一机器人出现两个 odom 提供方；消费者应改用绑定的底盘原语，而不是等待 Mapping 重发同一数据。

输入侧由 `sensor_providers` 选择并绑定 `lidar2d`、`lidar3d`、`rgb`、`depth`、`imu`、`odom` 角色；只解析实际绑定的 Atlas 提供方。默认 RTAB-Map 可使用二维或三维激光雷达、RGB-D 和可选外部里程计；DLIO 路径需要三维激光雷达和 IMU。`fastlio2` 在当前参考实现中标为漂移故障，仅保留用于复现。

参考实现为 [`service-map-rbnx`](https://github.com/syswonder/service-map-rbnx/tree/ce4092a1bee8847d6314af957f0225c8371d9aa6)。`algo` 默认并推荐 `rtabmap`，也可选择 `dlio`；机器人部署保存完整 `config/rtabmap_params.yaml`，并在建图服务的 `config.params_file` 中引用。共享仓库模板不会在运行时隐式加载。命名地图持久化目前只支持 RTAB-Map：`map_mode: mapping` 建图后调用 `save_map(map_id)`，`map_mode: localization` 配合已有 `map_id` 加载不可变副本并保持地图坐标系稳定。

输出话题的适配面覆盖各个 `algo`，但地图数据库、`pose_estimate`、模式切换和重置的实现直接调用 RTAB-Map 数据库或 ROS 服务。选择 DLIO 时不能假设这些管理 RPC 可用，应按响应中的 `ok` 和 `detail` 处理失败。

> 实现文档：[README（部署、`params_file`、传感器绑定与兼容迁移）](https://github.com/syswonder/service-map-rbnx/blob/ce4092a1bee8847d6314af957f0225c8371d9aa6/README.md) · [`config.spec`（全部 instance 配置、类型、默认值与约束）](https://github.com/syswonder/service-map-rbnx/blob/ce4092a1bee8847d6314af957f0225c8371d9aa6/config.spec) · [实现注册](https://github.com/syswonder/service-map-rbnx/blob/ce4092a1bee8847d6314af957f0225c8371d9aa6/src/mapping_rbnx/atlas_bridge.py)

RTAB-Map 参数名、默认值和上游说明以 [`Parameters.h`](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h) 为准。以 service 提供的 template 为起点，在 deployment 的 YAML 中增加或修改本体所需参数；运行时只读取这份 deployment 文件，不隐式叠加上游配置。

> 上游已知缺陷：三份软件包清单都漏列 `get_mode`、`get_pose`、`delete_map`、`reset_map`，但实现已同时注册其 gRPC 服务端和 MCP 工具。依赖软件包清单枚举能力的工具看不到这四项；调用方应先从 Atlas 确认实际注册状态。
