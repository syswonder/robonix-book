# 机械臂 robonix/primitive/arm

机械臂原语用统一的关节名称和单位提供指令、反馈与末端位姿。夹爪若属于机械臂，也作为同一 `JointState` 中的具名关节暴露。

能力约定 TOML 在 `capabilities/primitive/arm/`，IDL 在 `capabilities/lib/common_interfaces/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/primitive/arm/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/arm/driver.v1.toml` |
| `robonix/primitive/arm/end_pose` | `topic_out` | [`geometry_msgs/Pose`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-pose-msg) | `primitive/arm/end_pose.v1.toml` |
| `robonix/primitive/arm/joint_command` | `topic_in` | [`sensor_msgs/JointState`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-jointstate-msg) | `primitive/arm/joint_command.v1.toml` |
| `robonix/primitive/arm/joint_states` | `topic_out` | [`sensor_msgs/JointState`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-jointstate-msg) | `primitive/arm/joint_states.v1.toml` |

`end_pose` 的参考坐标系由具体 provider 的能力文档声明；关节指令与反馈使用相同的关节命名和单位。
