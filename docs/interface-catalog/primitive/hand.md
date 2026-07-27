# 灵巧手

灵巧手原语把不同硬件的手指运动统一为两层接口：优先使用与硬件无关的“手指 + 运动类型”，需要精确控制时再使用驱动公开的轴名称。能力约定 TOML 位于 `capabilities/primitive/hand/`，消息与服务定义位于 `capabilities/lib/hand/`。

新软件包省略 Driver 条目，由框架自动注册共享的 `robonix/lifecycle/driver`；显式选择共享 Driver 的行为相同。

:::warning[后向兼容：灵巧手命名空间 Driver]
`robonix/primitive/hand/driver`、`lifecycle/Driver` 和 `primitive/hand/driver.v1.toml` 只用于仍由软件包自行维护 Driver TOML 的旧实现。目前仍可使用，但计划迁移到共享 Driver。同一个提供方必须且只能注册一条 Driver。
:::

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 语义 |
|---|---|---|---|
| `robonix/primitive/hand/info` | `rpc` | [`hand/GetHandInfo`](../../reference/idl.md#hand-srv-gethandinfo-srv) | 查询该手的全部控制轴及其含义 |
| `robonix/primitive/hand/move_finger` | `rpc` | [`hand/MoveFinger`](../../reference/idl.md#hand-srv-movefinger-srv) | 按 `(finger, type)` 下发与硬件无关的位置目标 |
| `robonix/primitive/hand/move_joint` | `rpc` | [`hand/MoveJoint`](../../reference/idl.md#hand-srv-movejoint-srv) | 按 `JointInfo.name` 精确控制单个轴 |
| `robonix/primitive/hand/set_finger_speed_limits` | `rpc` | [`hand/SetFingerSpeedLimits`](../../reference/idl.md#hand-srv-setfingerspeedlimits-srv) | 按手指运动设置速度上限 |
| `robonix/primitive/hand/set_finger_torque_limits` | `rpc` | [`hand/SetFingerTorqueLimits`](../../reference/idl.md#hand-srv-setfingertorquelimits-srv) | 按手指运动设置力矩上限 |
| `robonix/primitive/hand/set_joint_speed_limits` | `rpc` | [`hand/SetJointSpeedLimits`](../../reference/idl.md#hand-srv-setjointspeedlimits-srv) | 按轴设置速度上限 |
| `robonix/primitive/hand/set_joint_torque_limits` | `rpc` | [`hand/SetJointTorqueLimits`](../../reference/idl.md#hand-srv-setjointtorquelimits-srv) | 按轴设置力矩上限 |
| `robonix/primitive/hand/state_finger` | `topic_out` | [`hand/FingerState`](../../reference/idl.md#hand-msg-fingerstate-msg) | 按驱动原生频率发布手指级当前位置 |
| `robonix/primitive/hand/state_joint` | `topic_out` | [`hand/JointState`](../../reference/idl.md#hand-msg-jointstate-msg) | 按驱动原生频率发布轴级当前位置 |

## 控制模型

调用方应先调用 `info`，再根据返回的 `JointInfo[]` 选择控制层级：

- `move_finger` 使用 `thumb`、`index`、`middle`、`ring`、`pinky` 和 `bend`、`sway`、`oppose`、`other` 的组合。一个组合可以映射到多个硬件轴，映射与联动方式由驱动负责，适合跨型号复用。
- `move_joint` 使用驱动返回的、不透明且唯一的 `JointInfo.name`。不要从名称解析关节含义；轴的手指归属、运动类型、顺序和端点含义都由 `JointInfo` 描述。
- 无法映射到五指模型的轴使用空 `finger`，只能通过轴名称控制。`type=other` 时，驱动必须在 `description` 中说明归一化端点。

位置、速度上限和力矩上限都使用闭区间 `[0, 1]` 的归一化值，不表示公制距离、弧度或牛顿米。越界值、非有限值、空请求、未知目标和重复目标都应被拒绝，不能静默截断。未列出的手指运动或轴保持当前目标。

运动调用返回 `ok=true` 只表示命令已经接受并下发，不表示已经到达目标。需要等待动作完成的调用方必须观察 `state_finger` 或 `state_joint`，并按任务容差和超时判断。
