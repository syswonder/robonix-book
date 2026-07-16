<span id="导航-robonixservicenavigation"></span>
# 导航

导航服务承担**目标式**运动：消费方给出地图坐标系中的目标，服务内部完成路径规划与避障。固定参考实现由 Nav2 产生速度，并经最终速度保护器发布 ROS `/cmd_vel`；底盘如何接收该话题由机器人部署完成。它不解析 `primitive/chassis/twist_in` 能力约定。智能体和规划器应调用导航 MCP 工具，不直接生成瞬时速度命令。

能力约定 TOML 在 `capabilities/service/navigation/`；直接 IDL 位于 `capabilities/lib/{navigation,lifecycle,common_interfaces}/`。

## 接口

| 能力约定 ID | 模式 | `main@b1a923a` 注册 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/navigation/driver` | `rpc` | gRPC（生命周期） | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `service/navigation/driver.v1.toml` |
| `robonix/service/navigation/navigate` | `rpc` | gRPC + MCP | [`navigation/Navigate`](../../reference/idl.md#navigation-srv-navigate-srv) | `service/navigation/navigate.v1.toml` |
| `robonix/service/navigation/navigate/status` | `rpc` | gRPC + MCP | [`navigation/GetNavigationStatus`](../../reference/idl.md#navigation-srv-getnavigationstatus-srv) | `service/navigation/navigate/status.v1.toml` |
| `robonix/service/navigation/navigate/cancel` | `rpc` | gRPC + MCP | [`navigation/CancelNavigation`](../../reference/idl.md#navigation-srv-cancelnavigation-srv) | `service/navigation/navigate/cancel.v1.toml` |

`navigate(goal: geometry_msgs/PoseStamped)` 返回提供方分配的 `run_id`，消费方用它通过 `navigate/status` / `navigate/cancel` 寻址同一个目标。`detail` 只是一句人类可读的说明（接受或拒绝的原因），不是结构化数据，消费方不应解析其内容。

参考实现固定为 [`service-navigation-rbnx main@b1a923a`](https://github.com/syswonder/service-navigation-rbnx/tree/b1a923a25cb3bf75554b861fceb605a190ae641b)。它把 `navigate`、`navigate/status`、`navigate/cancel` 接到 Nav2 `navigate_to_pose` action；`provider_ids` 必须绑定 map、chassis odom 和 lidar。使用 3D lidar 时，改绑 `scan_cloud` 并显式配置 `scan_projection`。Robot deployment 必须保存完整 `config/nav2_params.yaml` 并通过 `config.params_file` 引用；可用 `bt_xml_file` 指定本体 BehaviorTree。共享仓库的 `config/nav2_params.example.yml` 只是中性模板，不是机器人 profile。

缺少所选提供方时，`Driver(CMD_INIT)` 返回 `Deferred`；配置无效或 Nav2 启动失败时返回错误并清理子进程。部署程序应区分这两类结果，不能把 `Deferred` 当成已可导航。

> 固定快照文档：[README（部署目标、`params_file`、3D lidar 与兼容迁移）](https://github.com/syswonder/service-navigation-rbnx/blob/b1a923a25cb3bf75554b861fceb605a190ae641b/README.md) · [`config.spec`（全部 instance 配置、类型、默认值与约束）](https://github.com/syswonder/service-navigation-rbnx/blob/b1a923a25cb3bf75554b861fceb605a190ae641b/config.spec) · [实现注册](https://github.com/syswonder/service-navigation-rbnx/blob/b1a923a25cb3bf75554b861fceb605a190ae641b/nav2_wrapper/atlas_bridge.py)

各 server/plugin 参数见 Nav2 官方 [Configuration Guide](https://docs.nav2.org/configuration/index.html)，本体尺寸、footprint、costmap、planner/controller 等实机调优见 [Tuning Guide](https://docs.nav2.org/tuning/index.html)。
