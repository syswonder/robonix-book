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

参考实现：[`service-navigation-rbnx`](https://github.com/syswonder/service-navigation-rbnx)（上游仓库）封装标准 [Nav2](https://navigation.ros.org/) 栈，把 `navigate`/`navigate/status`/`navigate/cancel` 接到 Nav2 的 `navigate_to_pose` action；输入（map、odom、scan）按能力约定和 `provider_ids` 解析。Robot deployment 保存完整 `config/nav2_params.yaml`，通过 `config.params_file` 引用，并可用 `bt_xml_file` 指定本体自己的 BehaviorTree。共享仓库只保留 `config/nav2_params.example.yml`，不保存机器人型号 profile。

> 文档参考：[README（部署目标、`params_file`、BehaviorTree 与兼容迁移）](https://github.com/syswonder/service-navigation-rbnx/blob/main/README.md) · [`config.spec`（全部 instance 配置、类型、默认值与约束）](https://github.com/syswonder/service-navigation-rbnx/blob/main/config.spec) · [CAPABILITY.md（能力面与工具接口）](https://github.com/syswonder/service-navigation-rbnx/blob/main/CAPABILITY.md)

各 server/plugin 参数见 Nav2 官方 [Configuration Guide](https://docs.nav2.org/configuration/index.html)，本体尺寸、footprint、costmap、planner/controller 等实机调优见 [Tuning Guide](https://docs.nav2.org/tuning/index.html)。
