# 机械臂 robonix/prm/arm

## 说明

`robonix/prm/arm` 预留给**基础、可标准化的机械臂能力**，例如末端位姿控制、关节控制、轨迹跟踪、状态反馈等。

**不属于 `prm/arm` 的内容：**

- 自然语言指令执行
- 任务级 pick / place / handover 等策略语义
- 带显式 planning / policy / workflow 含义的开放接口

上述能力应放在 Skill Node / SKILL.md 中，或在确属平台运行时的一部分时放到 `robonix/sys/...` 服务层。

## 规划中的接口（尚无 TOML）

以下接口为设计意图，IDL 包 `prm_arm` 尚未完整入库，暂无对应 TOML。新增时按 `rust/contracts/README.md` 规范创建版本化文件（如 `arm_move_ee.v1.toml`）。

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/prm/arm/move_ee` | `rpc` | `geometry_msgs/PoseStamped` → `std_msgs/String` | — |
| `robonix/prm/arm/move_joint` | `rpc` | `prm_arm/JointTarget` → `std_msgs/String` | — |
| `robonix/prm/arm/joint_trajectory` | `rpc` | `trajectory_msgs/JointTrajectory` → `std_msgs/String` | — |
| `robonix/prm/arm/state_joint` | `pub-sub (out)` | `sensor_msgs/JointState` | — |

## 典型组合

完整关节控制的 provider 通常至少实现 `move_ee`（或 `move_joint`）+ `state_joint`；有轨迹跟踪需求时再补 `joint_trajectory`。面向任务的自然语言操作通常由上层 Skill 组合这些基础能力来完成。
