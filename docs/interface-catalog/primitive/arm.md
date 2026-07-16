# 机械臂

机械臂原语用统一的关节名称和单位提供指令、反馈与末端位姿。夹爪若属于机械臂，也作为同一 `JointState` 中的具名关节暴露。

能力约定 TOML 在 `capabilities/primitive/arm/`，接口定义语言（Interface Definition Language，IDL）文件在 `capabilities/lib/common_interfaces/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/primitive/arm/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/arm/driver.v1.toml` |
| `robonix/primitive/arm/end_pose` | `topic_out` | [`geometry_msgs/Pose`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-pose-msg) | `primitive/arm/end_pose.v1.toml` |
| `robonix/primitive/arm/joint_command` | `topic_in` | [`sensor_msgs/JointState`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-jointstate-msg) | `primitive/arm/joint_command.v1.toml` |
| `robonix/primitive/arm/joint_states` | `topic_out` | [`sensor_msgs/JointState`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-jointstate-msg) | `primitive/arm/joint_states.v1.toml` |

`end_pose` 不携带坐标系，具体提供方必须在能力文档中声明参考坐标系。`joint_command` 提供方可以只支持 `JointState` 的部分字段，因此同一文档还必须列出支持的 `position`、`velocity`、`effort` 字段；关节指令与反馈使用相同的关节命名和单位。

当前 Webots Tiago Lite 部署没有机械臂或夹爪 primitive；本页描述的是标准能力约定，不表示示例部署已经注册这些接口。
