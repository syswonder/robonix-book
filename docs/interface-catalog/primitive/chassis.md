---
title: 底盘
---
<span id="底盘-robonixprimitivechassis"></span>
# 底盘


底盘原语覆盖移动机器人的**低层**运动控制和反馈。能力约定定义在 Robonix 源码树下：IDL 在 `capabilities/lib/chassis/`，能力约定 TOML 在 `capabilities/primitive/chassis/`（绝对路径见 `rbnx path capabilities`）。

> **注意**：目标式导航不属于底盘原语，由 `robonix/service/navigation/navigate`、`robonix/service/navigation/navigate/status` 和 `robonix/service/navigation/navigate/cancel` 承担（通常 Nav2）——详见 [导航服务](../service/navigation.md)。map 帧位姿查询也不在这里，而是 `robonix/service/map/pose` 的职责。底盘原语只负责下发 `robonix/primitive/chassis/move` / `robonix/primitive/chassis/twist_in` 瞬时运动输入，以及通过 `robonix/primitive/chassis/odom` 反馈底盘自身的运动事实。

新软件包省略 Driver 条目，由框架自动注册共享的 `robonix/lifecycle/driver`；显式选择共享 Driver 的行为相同。未实现生命周期回调时，框架记录警告并执行空操作。

:::warning[后向兼容：底盘命名空间 Driver]
`robonix/primitive/chassis/driver`、`lifecycle/Driver` 和 `primitive/chassis/driver.v1.toml` 只用于仍由软件包自行维护 Driver TOML 的旧实现。目前仍可使用，但计划迁移到共享 Driver；两种 Driver 不能同时注册。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

## 接口

| 能力约定 ID（`contract_id`） | 模式 | Tiago 参考实现传输 | 载荷（IDL） | 能力约定 TOML |
|--------------------------|------|-------------|-------------|-----------|
| `robonix/primitive/chassis/move` | `rpc` | gRPC | [`chassis/ExecuteMoveCommand`](../../reference/idl.md#chassis-srv-executemovecommand-srv)（`MoveCommand` → `std_msgs/String`） | `primitive/chassis/move.v1.toml` |
| `robonix/primitive/chassis/twist_in` | `topic_in` | ROS 2 | [`geometry_msgs/Twist`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-twist-msg) | `primitive/chassis/twist_in.v1.toml` |
| `robonix/primitive/chassis/odom` | `topic_out` | ROS 2 | [`nav_msgs/Odometry`](../../reference/idl.md#common-interfaces-nav-msgs-msg-odometry-msg) | `primitive/chassis/odom.v1.toml` |

底盘有两个运动入口，分工明确：

- `robonix/primitive/chassis/move`（单发离散命令）：一次 `forward_m` / `rotate_deg`，或带 `duration_sec` 封顶的速度命令。当前 Tiago 参考实现通过 gRPC 接收请求，按配置速度换算持续时间，在 ROS 2 端点 `/cmd_vel` 上开环发布一小段 `Twist` 再停；`forward_m` / `rotate_deg` 是近似量，不提供 odom 闭环精度保证。该实现不暴露 MCP 传输；需要避障或精确到达时使用 `robonix/service/navigation/navigate`。
- `robonix/primitive/chassis/twist_in`（`geometry_msgs/Twist`）：连续速度流入口。`topic_in` 模式不绑定具体传输方式；当前 Webots Tiago 实现把它绑定到 ROS 2 话题 `/cmd_vel`，两个字符串不是同一层的名称。导航不通过 gRPC 调用 `robonix/primitive/chassis/move`。

需要 odom 坐标系下的位姿时订阅 `robonix/primitive/chassis/odom`；需要 map 坐标系下的定位结果时使用 `robonix/service/map/pose`。底盘原语不合并上层定位结果。

## 当前 Webots Tiago 组合

当前 Tiago 参考实现注册 `driver` + `move` + `odom` + `twist_in`：

- `driver`：gRPC，rbnx boot 通过 `Driver(CMD_INIT)` 把底盘启起来
- `move`：gRPC（不暴露为 MCP），离散单发运动命令（前进 / 旋转 / 限时速度）
- `odom`：ROS 2，给 `service/map/*` 做融合定位用
- `twist_in`：ROS 2，Nav2 控制器把 `cmd_vel` 发到这里
