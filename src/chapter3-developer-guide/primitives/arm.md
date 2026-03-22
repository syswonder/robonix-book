# 机械臂 (Arm)

命名空间：`robonix/prm/arm`

## 概述

机械臂原语覆盖从**笛卡尔空间**到**关节空间**的常见控制需求：既可以下发末端位姿或关节位置目标，也可以执行整段关节轨迹，同时通过 `state_joint` 持续反馈当前关节状态。集成方可以只实现与本机控制器能力相符的子集，例如只做关节空间控制而不暴露笛卡尔 `move_ee`。

## 接口列表

| 接口 | 通信语义 | 载荷 | 说明 |
|------|----------|------|------|
| `move_ee` | command | pose → status | 末端执行器位姿控制 |
| `move_joint` | command | joint_positions → status | 关节位置控制 |
| `joint_trajectory` | command | trajectory → status | 关节轨迹执行 |
| `state_joint` | stream | `sensor_msgs/msg/JointState` | 关节状态流 |

## 典型组合

| 场景 | 建议实现 |
|------|----------|
| 仅笛卡尔控制 | move_ee, state_joint |
| 仅关节控制 | move_joint, state_joint |
| 轨迹控制 | joint_trajectory, state_joint |
| 全功能 | 以上全部 |
