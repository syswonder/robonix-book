# 机械臂 robonix/prm/arm

机械臂原语覆盖末端执行器运动、关节运动、轨迹执行和关节状态反馈。IDL 包 `prm_arm` 尚未完整入库，当前以设计意图为准。

## 预定义接口

| abstract_interface_id | 模式 | 说明 |
|-----------------------|------|------|
| `robonix/prm/arm/move_ee` | RPC | 将末端执行器移动到目标 pose |
| `robonix/prm/arm/move_joint` | RPC | 将指定关节移动到目标角度 |
| `robonix/prm/arm/joint_trajectory` | RPC | 执行一段关节轨迹 |
| `robonix/prm/arm/state_joint` | pub-sub (output) | 实时关节状态反馈 |

## 典型组合

机械臂通常至少实现 `move_ee`（或 `move_joint`）+ `state_joint`。需要轨迹跟踪的场景加上 `joint_trajectory`。provider 按自身硬件能力选择实现子集。
