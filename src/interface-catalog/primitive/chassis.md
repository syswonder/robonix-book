# 底盘 robonix/primitive/chassis

底盘原语覆盖移动机器人的**低层**运动控制和位姿反馈。IDL 定义在 `rust/crates/robonix-interfaces/lib/prm_chassis/`，契约 TOML 在 `capabilities/primitive/chassis/`。

> **注意**：目标式导航（`navigate` / `status` / `cancel`）已经不属于底盘原语。它们是规划/决策层能力，由 `robonix/service/navigation/*` 服务承担，通常由 Nav2 等导航栈提供——详见 [导航服务](../service/navigation.md)。原语层只负责下发瞬时速度（`move` / `twist_in`）和反馈底盘状态（`state` / `odom`）。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/primitive/chassis/move` | `rpc` | `prm_chassis/MoveCommand` → `std_msgs/String` | `primitive/chassis/move.v1.toml` |
| `robonix/primitive/chassis/state` | `rpc` | `std_msgs/Empty` → `prm_chassis/RobotState` | `primitive/chassis/state.v1.toml` |
| `robonix/primitive/chassis/twist_in` | `pub-sub (in)` | `geometry_msgs/Twist` | `primitive/chassis/twist_in.v1.toml` |
| `robonix/primitive/chassis/odom` | `pub-sub (out)` | `nav_msgs/Odometry` | `primitive/chassis/odom.v1.toml` |

`move` 是底盘的唯一运动入口——既支持瞬时速度（linear/angular 一次写入直接下发到 `cmd_vel`），也支持封顶执行时长（`duration_sec`）让一次"转 30°"或"前进 20 cm"这种短动作能完整跑完。LLM 通过 MCP 直接调它；用法见 [`tiago_chassis/CAPABILITY.md`](https://github.com/syswonder/robonix/blob/main/examples/webots/primitives/tiago_chassis/CAPABILITY.md) 里的 burst pattern。

## 典型组合

基础移动底盘实现 `move` + `state`：

- `move`：MCP，LLM 直接下发瞬时速度命令
- `state`：MCP，LLM 拿位姿确认动作执行结果（"snapshot → reason → cmd → snapshot"）

用作导航底盘时，把 `odom` 作为输出给 `robonix/service/map/*` 做融合定位、`twist_in` 作为 Nav2 控制器的输入（`cmd_vel`）。
