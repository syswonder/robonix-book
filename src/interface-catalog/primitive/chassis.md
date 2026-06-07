# 底盘 robonix/primitive/chassis

[toc]

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

- `move`（gRPC，单发离散命令）：一次 `forward_m` / `rotate_deg`，或带 `duration_sec` 封顶的速度命令。驱动收到后在 `cmd_vel` 上发一小段 `Twist` 再停——适合"前进 1 m""转 30°"这种离散动作（snapshot → reason → move → snapshot）。`move` 刻意不暴露为 MCP：它下发的是未经避障的瞬时速度，不挂到大模型工具列表上；要带路径规划的运动，走 `service/navigation/navigate`。
- `twist_in`（ROS 2 topic，`geometry_msgs/Twist`）：连续速度流入口。导航控制器（`simple_nav` 的 nav 节点、Nav2 controller）和 teleop 把 `cmd_vel` 发到这里，底盘连续跟随。注意导航**不是**通过 gRPC 调 `move`，而是往 `twist_in` 发 `Twist`。

> **历史变更**：早期版本有 `robonix/primitive/chassis/state` 能力约定，返回一个 `RobotState` 巨型消息（`base_pose + joint_state + tcp_pose + gripper`）。`base_pose` 字段实际上是 AMCL 的输出，让底盘原语去依赖一个上层定位服务——这是分层倒置（原语应该是叶子节点）。该能力约定已删除，消费者改为：
>
> - 想拿 odom-frame 位姿 → 订阅 `primitive/chassis/odom`
> - 想拿 map-frame 位姿 → 订阅 `service/map/pose`

## 典型组合

基础移动底盘实现 `driver` + `move` + `odom` + `twist_in`：

- `driver`：gRPC，rbnx boot 通过 `Driver(CMD_INIT)` 把底盘启起来
- `move`：gRPC（不暴露为 MCP），离散单发运动命令（前进 / 旋转 / 限时速度）
- `odom`：ROS 2，给 `service/map/*` 做融合定位用
- `twist_in`：ROS 2，Nav2 控制器把 `cmd_vel` 发到这里
