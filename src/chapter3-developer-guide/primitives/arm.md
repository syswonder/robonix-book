# 机械臂 (Arm)

命名空间：`robonix/prm/arm`

## 概述

机械臂原语抽象多自由度机械臂，支持末端执行器位姿控制、关节位置控制、关节轨迹执行及关节状态反馈。

## 接口列表

| 接口 | 原语类型 | 载荷 | 说明 |
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
