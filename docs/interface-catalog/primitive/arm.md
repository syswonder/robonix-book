# 机械臂

机械臂原语用统一的关节名称和单位提供指令、反馈与末端位姿。夹爪若属于机械臂，也作为同一 `JointState` 中的具名关节暴露。

能力约定 TOML 在 `capabilities/primitive/arm/`。机械臂指令与反馈使用的接口定义语言（Interface Definition Language，IDL）文件位于 `capabilities/lib/common_interfaces/`；后向兼容 Driver 使用 `capabilities/lib/lifecycle/srv/Driver.srv`。

新软件包省略 Driver 条目，由框架自动注册共享的 `robonix/lifecycle/driver`；显式选择共享 Driver 的行为相同。未实现生命周期回调时，框架记录警告并执行空操作。

:::warning[后向兼容：机械臂命名空间 Driver]
`robonix/primitive/arm/driver`、`lifecycle/Driver` 和 `primitive/arm/driver.v1.toml` 只用于仍由软件包自行维护 Driver TOML 的旧实现。目前仍可使用，但计划迁移到共享 Driver。同一个提供方必须且只能注册一条生命周期 Driver，不能同时注册共享 Driver 和机械臂命名空间 Driver。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/primitive/arm/end_pose` | `topic_out` | [`geometry_msgs/Pose`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-pose-msg) | `primitive/arm/end_pose.v1.toml` |
| `robonix/primitive/arm/pos_command` | `topic_in` | [`geometry_msgs/Pose`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-pose-msg) | `primitive/arm/pos_command.v1.toml` |
| `robonix/primitive/arm/joint_command` | `topic_in` | [`sensor_msgs/JointState`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-jointstate-msg) | `primitive/arm/joint_command.v1.toml` |
| `robonix/primitive/arm/joint_states` | `topic_out` | [`sensor_msgs/JointState`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-jointstate-msg) | `primitive/arm/joint_states.v1.toml` |

`end_pose` 与 `pos_command` 都使用不带时间戳和 frame ID 的 `geometry_msgs/Pose`，具体提供方必须在能力文档中声明两者共同使用的机械臂基准坐标系。`pos_command` 表示末端执行器笛卡尔目标位姿，逆运动学或笛卡尔控制由提供方负责；它不携带夹爪开度。夹爪仍作为具名关节通过 `joint_command` 控制。

`joint_command` 提供方可以只支持 `JointState` 的部分字段，因此同一文档还必须列出支持的 `position`、`velocity`、`effort` 字段；关节指令与反馈使用相同的关节命名和单位。

### 消息字段

`geometry_msgs/Pose` 包含以下字段：

- `position.x`、`position.y`、`position.z`：末端执行器的位置。
- `orientation.x`、`orientation.y`、`orientation.z`、`orientation.w`：末端执行器姿态四元数。

该消息没有时间戳或坐标系字段。因此，提供方文档必须明确 `end_pose` 和 `pos_command` 共同使用的基准坐标系；调用方不能从消息本身推断坐标系。

`sensor_msgs/JointState` 包含 `header`、`name`、`position`、`velocity` 和 `effort`。其中：

- `header` 包含采样时间和可选的 `frame_id`。
- `name` 按顺序给出关节名称；机械臂所属夹爪也使用具名关节。
- `position`、`velocity`、`effort` 可以为空；非空数组必须与 `name` 一一对应并保持相同长度。

能力约定允许不同提供方支持 `joint_command` 的不同字段组合。软件包文档应明确可写字段、单位、关节名称和不支持字段的处理方式；这是提供方接入要求，不是 `JointState` 消息在运行时自动协商的能力。

### 通信模式

- `topic_out` 是提供方持续输出的数据流。代码生成后，调用方以服务端流方式订阅 `end_pose` 或 `joint_states`。
- `topic_in` 是调用方向提供方持续写入的输入流。代码生成后，调用方以客户端流方式发送 `pos_command` 或 `joint_command`。

具体通信库调用方式见[运行时通信](../../architecture/runtime-communication.md)和[开发者指南中的 gRPC 示例](../../developer-guide.md#145-mcp-与-grpc)。

### 消息示例

下面只展示消息字段，不是软件包实例配置。实际坐标、关节名称和数值必须来自当前机械臂提供方，并满足设备限位。

```yaml title="pos_command / end_pose"
position:
  x: 0.35
  y: 0.00
  z: 0.25
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
```

```yaml title="joint_command / joint_states"
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ""
name: [joint_1, joint_2, gripper_joint]
position: [0.0, -0.4, 0.025]
velocity: []
effort: []
```

当前 Webots Tiago Lite 部署没有机械臂或夹爪 primitive；本页描述的是标准能力约定，不表示示例部署已经注册这些接口。
