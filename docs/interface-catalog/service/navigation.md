---
title: 导航
---
<span id="导航-robonixservicenavigation"></span>
# 导航

导航服务承担**目标式**运动：消费方给出地图坐标系中的目标，服务内部完成路径规划与避障。当前参考实现由 Nav2 产生速度，并经最终速度保护器发布 ROS 2 话题 `/cmd_vel`；底盘如何接收该话题由机器人部署完成。该话题名不是 `robonix/primitive/chassis/twist_in` 能力约定 ID，当前参考实现也不通过 Atlas 解析这条契约。智能体和规划器应调用导航的 MCP 能力，不直接生成瞬时速度命令。

能力约定 TOML 在 `capabilities/service/navigation/`；直接使用的接口定义语言（Interface Definition Language，IDL）文件位于 `capabilities/lib/{navigation,lifecycle,common_interfaces}/`。

> 表中的命名空间 Driver 是已有软件包的兼容接口。新软件包省略 Driver 条目时由框架自动使用共享的 `robonix/lifecycle/driver`；显式共享仍受支持，两种 Driver 只能选择一条。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。

## 接口

| 能力约定 ID | 模式 | 参考实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/navigation/driver` | `rpc` | gRPC（生命周期） | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `service/navigation/driver.v1.toml` |
| `robonix/service/navigation/navigate` | `rpc` | gRPC + MCP | [`navigation/Navigate`](../../reference/idl.md#navigation-srv-navigate-srv) | `service/navigation/navigate.v1.toml` |
| `robonix/service/navigation/navigate/status` | `rpc` | gRPC + MCP | [`navigation/GetNavigationStatus`](../../reference/idl.md#navigation-srv-getnavigationstatus-srv) | `service/navigation/navigate/status.v1.toml` |
| `robonix/service/navigation/navigate/cancel` | `rpc` | gRPC + MCP | [`navigation/CancelNavigation`](../../reference/idl.md#navigation-srv-cancelnavigation-srv) | `service/navigation/navigate/cancel.v1.toml` |

`navigate(goal: geometry_msgs/PoseStamped)` 返回提供方分配的 `run_id`，消费方用它通过 `navigate/status` / `navigate/cancel` 寻址同一个目标。`detail` 只是一句人类可读的说明（接受或拒绝的原因），不是结构化数据，消费方不应解析其内容。

参考实现为 [`service-navigation-rbnx`](https://github.com/syswonder/service-navigation-rbnx/tree/b1a923a25cb3bf75554b861fceb605a190ae641b)。它把 `navigate`、`navigate/status`、`navigate/cancel` 接到 Nav2 的 `navigate_to_pose` 动作；`provider_ids` 必须绑定地图、底盘里程计和激光雷达。使用三维激光雷达时，改绑 `scan_cloud` 并显式配置 `scan_projection`。机器人部署仓库必须保存完整 `config/nav2_params.yaml` 并通过 `config.params_file` 引用；可用 `bt_xml_file` 指定本体行为树。共享仓库的 `config/nav2_params.example.yml` 只是中性模板，不是机器人配置组合。

缺少所选提供方时，`Driver(CMD_INIT)` 返回 `Deferred`；配置无效或 Nav2 启动失败时返回错误并清理子进程。部署程序应区分这两类结果，不能把 `Deferred` 当成已可导航。

> 实现文档：[README（部署目标、`params_file`、3D lidar 与兼容迁移）](https://github.com/syswonder/service-navigation-rbnx/blob/b1a923a25cb3bf75554b861fceb605a190ae641b/README.md) · [`config.spec`（全部 instance 配置、类型、默认值与约束）](https://github.com/syswonder/service-navigation-rbnx/blob/b1a923a25cb3bf75554b861fceb605a190ae641b/config.spec) · [实现注册](https://github.com/syswonder/service-navigation-rbnx/blob/b1a923a25cb3bf75554b861fceb605a190ae641b/nav2_wrapper/atlas_bridge.py)

各 server/plugin 参数见 Nav2 官方 [Configuration Guide](https://docs.nav2.org/configuration/index.html)，本体尺寸、footprint、costmap、planner/controller 等实机调优见 [Tuning Guide](https://docs.nav2.org/tuning/index.html)。
