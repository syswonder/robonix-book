# 夹爪 (Gripper)

命名空间：`robonix/prm/gripper`

## 概述

夹爪原语抽象抓取执行器，支持开合命令、宽度设置及宽度状态反馈。常与机械臂配合使用。

## 接口列表

| 接口 | 原语类型 | 载荷 | 说明 |
|------|----------|------|------|
| `close` | command | req → status | 闭合夹爪 |
| `open` | command | req → status | 张开夹爪 |
| `set_width` | command | width → status | 设置目标宽度 |
| `state_width` | stream | `std_msgs/msg/Float64` | 当前宽度 |

## 典型组合

| 硬件 | 建议实现 |
|------|----------|
| 二指夹爪（开/关） | close, open |
| 可调宽度夹爪 | set_width, state_width |
| 全功能 | close, open, set_width, state_width |
