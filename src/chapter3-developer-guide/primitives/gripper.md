# 夹爪 (Gripper)

命名空间：`robonix/prm/gripper`

## 概述

夹爪原语描述末端**抓取执行器**常见的离散动作（张开/闭合）、连续量控制（目标宽度）以及宽度状态反馈。它通常与机械臂原语配合使用，但也可以在 manifest 中作为独立 node 实现，取决于你的驱动架构。

## 接口列表

| 接口 | 通信语义 | 载荷 | 说明 |
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
