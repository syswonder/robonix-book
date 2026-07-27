# 四足底盘

四足原语提供机体运动、姿态和本地里程计等低层设备接口。它不负责全局路径规划；需要“到某个地图位置”的任务仍应使用[导航服务](../service/navigation.md)。

能力约定 TOML 位于 `capabilities/primitive/quadruped/`。通用速度和里程计消息来自 `capabilities/lib/common_interfaces/`，姿态服务位于 `capabilities/lib/quadruped/`。

新软件包省略 Driver 条目，由框架自动注册共享的 `robonix/lifecycle/driver`；显式选择共享 Driver 的行为相同。

:::warning[后向兼容：四足命名空间 Driver]
`robonix/primitive/quadruped/driver`、`lifecycle/Driver` 和 `primitive/quadruped/driver.v1.toml` 只用于仍由软件包自行维护 Driver TOML 的旧实现。目前仍可使用，但计划迁移到共享 Driver。同一个提供方必须且只能注册一条 Driver。
:::

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 语义 |
|---|---|---|---|
| `robonix/primitive/quadruped/move` | `rpc` | [`chassis/ExecuteMoveCommand`](../../reference/idl.md#chassis-srv-executemovecommand-srv) | 执行一次有边界的平移、旋转或速度动作，完成后停止 |
| `robonix/primitive/quadruped/twist_in` | `topic_in` | [`geometry_msgs/Twist`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-twist-msg) | 持续接收控制器速度指令 |
| `robonix/primitive/quadruped/odom` | `topic_out` | [`nav_msgs/Odometry`](../../reference/idl.md#common-interfaces-nav-msgs-msg-odometry-msg) | 发布局部 `odom` 坐标系中的原始里程计 |
| `robonix/primitive/quadruped/posture` | `rpc` | [`quadruped/SetPosture`](../../reference/idl.md#quadruped-srv-setposture-srv) | 切换到驱动定义的具名机体姿态 |

`move` 是一次性动作，不是连续控制流；遥操作或上层控制器需要持续刷新速度时使用 `twist_in`。`posture_name` 的可选值由具体机器人驱动定义和记录，例如 `stand`、`sit` 或 `crouch`；驱动必须拒绝硬件不支持的名称。
