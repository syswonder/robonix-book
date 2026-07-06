# 导航 robonix/service/navigation

导航服务承担**目标式**运动：消费方给一个 map 帧目标，服务内部做路径规划 + 避障，并通过底盘原语的 `primitive/chassis/twist_in` 连续发布速度命令把机器人送到位。`primitive/chassis/move` 是低层离散/调试控制面，不是导航服务的主路径。和底盘原语的瞬时速度严格分层——LLM 永远走导航服务，不直接控速。

能力约定 TOML 在 `capabilities/service/navigation/`，IDL 在 `capabilities/lib/navigation/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/service/navigation/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `service/navigation/driver.v1.toml` |
| `robonix/service/navigation/navigate` | `rpc` | [`navigation/Navigate`](../../reference/idl.md#navigation-srv-navigate-srv) | `service/navigation/navigate.v1.toml` |
| `robonix/service/navigation/navigate/status` | `rpc` | [`navigation/GetNavigationStatus`](../../reference/idl.md#navigation-srv-getnavigationstatus-srv) | `service/navigation/navigate/status.v1.toml` |
| `robonix/service/navigation/navigate/cancel` | `rpc` | [`navigation/CancelNavigation`](../../reference/idl.md#navigation-srv-cancelnavigation-srv) | `service/navigation/navigate/cancel.v1.toml` |

`navigate(goal: geometry_msgs/PoseStamped)` 返回 provider 分配的 `run_id`，消费方拿它去 `navigate/status` / `navigate/cancel` 寻址同一个目标。`detail` 只是一句人类可读的说明（接受/拒绝的原因），不是结构化数据，消费方别去解析它的内容。

参考实现：[`service-navigation-rbnx`](https://github.com/syswonder/service-navigation-rbnx)（上游仓库）——封装标准 [Nav2](https://navigation.ros.org/) 栈，把 `navigate`/`navigate/status`/`navigate/cancel` 接到 Nav2 的 `navigate_to_pose` action；输入话题（`/map`、`/odom`、`/scan`）全部按能力约定经 atlas 发现，不硬编码本体。`sim` 参数 profile 在 odom 帧用滚动窗口 costmap 导航，只需 odom + 2D 雷达、不依赖 SLAM 的 `map→odom` TF，因此能直接在 webots 跑通；带 SLAM 定位的部署用 `slam`/`default` profile（map 帧 + AMCL）。

> 文档参考：[README（完整部署指南：3 种部署目标 / 生命周期 / params profile / DDS·rmem 调优）](https://github.com/syswonder/service-navigation-rbnx/blob/main/README.md) · [CAPABILITY.md（能力面 + `config:` 字段）](https://github.com/syswonder/service-navigation-rbnx/blob/main/CAPABILITY.md)
