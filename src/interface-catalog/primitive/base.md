# 底盘 robonix/prm/base

底盘原语覆盖移动机器人的导航、运动控制、位姿反馈等核心能力。IDL 定义在 `rust/crates/robonix-interfaces/lib/prm_base/`，生成的 proto 在 `robonix_proto/prm_base.proto`。

## RPC 接口

| 契约 ID（`contract_id`） | `[io]`（摘要） | gRPC 映射（IDL 包） | 契约源码（TOML） |
|-----------------------|----------------|---------------------|------------------|
| `robonix/prm/base/navigate` | `geometry_msgs/PoseStamped` → `std_msgs/String` | `PrmBaseService/Navigate`（与 `.srv` 一致） | `rust/contracts/prm/base_navigate.v1.toml` |
| `robonix/prm/base/nav_status` | `std_msgs/String` → `std_msgs/String`（JSON 载荷见 TOML 注释） | `PrmBaseService/GetNavigationStatus` | `rust/contracts/prm/base_nav_status.v1.toml` |
| `robonix/prm/base/nav_cancel` | `std_msgs/String` → `std_msgs/String` | `PrmBaseService/CancelNavigation` | `rust/contracts/prm/base_nav_cancel.v1.toml` |
| `robonix/prm/base/cmd` | `prm_base/MoveCommand` → `std_msgs/String`（单步指令，常与 MCP 对齐） | —（契约 `[mode].type = rpc`，门面见 `robonix_contracts.proto`） | `rust/contracts/prm/base_cmd.v1.toml` |
| `robonix/prm/base/stop` | `srv/Stop.srv` | `PrmBaseService/Stop` | — |
| `robonix/prm/robot/state` | `std_msgs/Empty` → `prm_base/RobotState` | — | `rust/contracts/prm/robot_state.v1.toml` |

## Pub-sub 接口（契约层：`topic_in` / `topic_out`）

| 契约 ID（`contract_id`） | 方向 | `[mode].type` | IDL（`[io.msg].msg`） | 契约源码（TOML） |
|-----------------------|------|---------------|---------------------------|------------------|
| `robonix/prm/base/twist_in` | input | `topic_in` | `geometry_msgs/msg/Twist`（如 cmd_vel） | `rust/contracts/prm/base_twist_in.v1.toml` |
| `robonix/prm/base/move` | input | `rpc_client_stream` | `prm_base/action/StreamMove` | `rust/contracts/prm/base_move.v1.toml` |
| `robonix/prm/base/odom` | output | `topic_out` | `nav_msgs/msg/Odometry` | `rust/contracts/prm/base_odom.v1.toml` |
| `robonix/prm/base/pose_cov` | output | — | `geometry_msgs/msg/PoseWithCovarianceStamped` | — |
| `robonix/prm/base/joint_state` | output | — | `sensor_msgs/msg/JointState` | — |
| `robonix/prm/base/goal_status` | output | — | `srv/GetNavigationStatus`（也可作 topic） | — |

纯 **topic** 形状见 **`rust/contracts/README.md`**（`topic_in` / `topic_out` 与 `robonix_contracts.proto` 中 `Stream` 的对应关系）；ROS 2 上仍由 topic 名与 QoS 约定实现。

## 典型组合

移动底盘通常至少实现 `navigate` + `stop` + `move`。带导航栈的底盘还应实现 `nav_status` 和 `nav_cancel`，让消费者能查询和取消正在执行的导航目标。位姿和里程计输出（`odom`、`pose_cov`）是可选但推荐的——它们为 Agent 提供空间感知所需的数据。
