---
title: 底盘
---
<span id="底盘-robonixprimitivechassis"></span>
# 底盘


底盘原语覆盖移动机器人的**低层**运动控制和反馈。能力约定定义在 Robonix 源码树下：IDL 在 `capabilities/lib/chassis/`，能力约定 TOML 在 `capabilities/primitive/chassis/`（绝对路径见 `rbnx path capabilities`）。

> **注意**：目标式导航（`navigate` / `status` / `cancel`）不属于底盘原语，由 `robonix/service/navigation/*` 服务承担（通常 Nav2）——详见 [导航服务](../service/navigation.md)。位姿在 map 帧的查询也不在这里——那是定位服务（`service/map/pose`）的职责。底盘原语只负责下发瞬时速度（`move` / `twist_in`）和反馈底盘自身的运动事实（`odom`）。

## 接口

| 能力约定 ID（`contract_id`） | 模式 | 载荷（IDL） | 能力约定 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/primitive/chassis/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/chassis/driver.v1.toml` |
| `robonix/primitive/chassis/move` | `rpc` | [`chassis/ExecuteMoveCommand`](../../reference/idl.md#chassis-srv-executemovecommand-srv)（`MoveCommand` → `std_msgs/String`） | `primitive/chassis/move.v1.toml` |
| `robonix/primitive/chassis/twist_in` | `topic_in` | [`geometry_msgs/Twist`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-twist-msg) | `primitive/chassis/twist_in.v1.toml` |
| `robonix/primitive/chassis/odom` | `topic_out` | [`nav_msgs/Odometry`](../../reference/idl.md#common-interfaces-nav-msgs-msg-odometry-msg) | `primitive/chassis/odom.v1.toml` |

底盘有两个运动入口，分工明确：

- `move`（单发离散命令）：一次 `forward_m` / `rotate_deg`，或带 `duration_sec` 封顶的速度命令。当前 Tiago 参考实现通过 gRPC 接收请求，按配置速度换算持续时间，在 `cmd_vel` 上开环发布一小段 `Twist` 再停；`forward_m` / `rotate_deg` 是近似量，不提供 odom 闭环精度保证。该实现不暴露 MCP；需要避障或精确到达时使用 `service/navigation/navigate`。
- `twist_in`（`geometry_msgs/Twist`）：连续速度流入口。`topic_in` 模式不绑定具体传输方式；当前 Webots Tiago 实现通过 ROS 2 接收 Nav2 控制器和遥控端的 `cmd_vel`。导航不通过 gRPC 调用 `move`。

需要 odom 坐标系下的位姿时订阅 `primitive/chassis/odom`；需要 map 坐标系下的定位结果时使用 `service/map/pose`。底盘原语不合并上层定位结果。

## 当前 Webots Tiago 组合

当前 Tiago 参考实现注册 `driver` + `move` + `odom` + `twist_in`：

- `driver`：gRPC，rbnx boot 通过 `Driver(CMD_INIT)` 把底盘启起来
- `move`：gRPC（不暴露为 MCP），离散单发运动命令（前进 / 旋转 / 限时速度）
- `odom`：ROS 2，给 `service/map/*` 做融合定位用
- `twist_in`：ROS 2，Nav2 控制器把 `cmd_vel` 发到这里
