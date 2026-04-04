# 底盘 robonix/prm/base

底盘原语覆盖移动机器人的导航、运动控制和位姿反馈。IDL 定义在 `rust/crates/robonix-interfaces/lib/prm_base/`，生成的 proto 在 `robonix_proto/prm_base.proto`。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/prm/base/navigate` | `rpc` | `geometry_msgs/PoseStamped` → `std_msgs/String` | `prm/base_navigate.v1.toml` |
| `robonix/prm/base/nav_status` | `rpc` | `std_msgs/String` → `std_msgs/String` | `prm/base_nav_status.v1.toml` |
| `robonix/prm/base/nav_cancel` | `rpc` | `std_msgs/String` → `std_msgs/String` | `prm/base_nav_cancel.v1.toml` |
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

基础移动底盘实现 `navigate` + `stop` + `cmd`。带完整导航栈的底盘还应实现 `nav_status` 和 `nav_cancel`。`odom` 和 `pose_cov` 为可选输出，供 Agent 获取空间感知数据。
