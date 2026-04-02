# 底盘 robonix/prm/base

底盘原语覆盖移动机器人的导航、运动控制、位姿反馈等核心能力。IDL 定义在 `rust/robonix-interfaces/lib/prm_base/`，生成的 proto 在 `robonix_proto/prm_base.proto`。

## RPC 接口

| 契约 ID（`contract_id`） | IDL | gRPC 映射 | 契约源码（TOML） |
|-----------------------|-----|-----------|------------------|
| `robonix/prm/base/navigate` | `srv/Navigate.srv` | `PrmBaseService/Navigate` | — |
| `robonix/prm/base/nav_status` | `srv/GetNavigationStatus.srv` | `PrmBaseService/GetNavigationStatus` | — |
| `robonix/prm/base/cancel_nav` | `srv/CancelNavigation.srv` | `PrmBaseService/CancelNavigation` | — |
| `robonix/prm/base/stop` | `srv/Stop.srv` | `PrmBaseService/Stop` | — |

## Pub-sub 接口

| 契约 ID（`contract_id`） | 方向 | IDL | gRPC 类型 | 契约源码（TOML） |
|-----------------------|------|-----|-----------|------------------|
| `robonix/prm/base/move` | input | `msg/MoveCommand.msg` | `message MoveCommand` | `rust/contracts/prm/base_move.v1.toml` |
| `robonix/prm/base/odom` | output | `nav_msgs/msg/Odometry.msg` | `nav_msgs.Odometry` | `rust/contracts/prm/base_odom.v1.toml` |
| `robonix/prm/base/pose_cov` | output | `geometry_msgs/msg/PoseWithCovarianceStamped.msg` | `geometry_msgs.PoseWithCovarianceStamped` | — |
| `robonix/prm/base/joint_state` | output | `sensor_msgs/msg/JointState.msg` | `sensor_msgs.JointState` | — |
| `robonix/prm/base/goal_status` | output | `srv/GetNavigationStatus.srv`（也可作 topic） | `PrmBaseService/GetNavigationStatus` | — |

Pub-sub 接口在 gRPC 端只有 `message` 定义（没有 RPC）；具体的数据传输通过 ROS 2 topic 或自定义 gRPC stream shim 完成。

## 典型组合

移动底盘通常至少实现 `navigate` + `stop` + `move`。带导航栈的底盘还应实现 `nav_status` 和 `cancel_nav`，让消费者能查询和取消正在执行的导航目标。位姿和里程计输出（`odom`、`pose_cov`）是可选但推荐的——它们为 Agent 提供空间感知所需的数据。
