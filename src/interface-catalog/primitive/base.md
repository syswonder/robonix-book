# 底盘 robonix/prm/base

底盘原语覆盖移动机器人的**低层**运动控制和位姿反馈。IDL 定义在 `rust/crates/robonix-interfaces/lib/prm_base/`，生成的 proto 在 `robonix_proto/prm_base.proto`。

> **注意**：目标式导航（`navigate` / `nav_status` / `nav_cancel`）已经不属于底盘原语。它们是规划/决策层能力，由 `robonix/srv/navigation/*` 服务承担，通常由 Nav2 等导航栈提供——详见
> [导航服务](../service/navigation.md)。原语层只负责下发瞬时速度（`cmd`/`twist_in`）和反馈底盘状态（`odom`/`pose_cov` 等）。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/prm/base/cmd` | `rpc` | `prm_base/MoveCommand` → `std_msgs/String` | `prm/base_cmd.v1.toml` |
| `robonix/prm/base/stop` | `rpc` | `std_msgs/Empty` → `std_msgs/String` | — |
| `robonix/prm/robot/state` | `rpc` | `std_msgs/Empty` → `prm_base/RobotState` | `prm/robot_state.v1.toml` |
| `robonix/prm/base/move` | `rpc_client_stream` | `prm_base/action/StreamMove` | `prm/base_move.v1.toml` |
| `robonix/prm/base/twist_in` | `pub-sub (in)` | `geometry_msgs/Twist` | `prm/base_twist_in.v1.toml` |
| `robonix/prm/base/odom` | `pub-sub (out)` | `nav_msgs/Odometry` | `prm/base_odom.v1.toml` |
| `robonix/prm/base/pose_cov` | `pub-sub (out)` | `geometry_msgs/PoseWithCovarianceStamped` | — |
| `robonix/prm/base/joint_state` | `pub-sub (out)` | `sensor_msgs/JointState` | — |
| `robonix/prm/base/goal_status` | `pub-sub (out)` | `std_msgs/String`（JSON） | — |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。

## 典型组合

基础移动底盘实现 `cmd` + `stop` + `odom`。用作导航底盘时，把 `odom` 作为输出给 `robonix/srv/slam/*` 做融合定位、`twist_in` 作为 Nav2 控制器的输入（`cmd_vel`）。
