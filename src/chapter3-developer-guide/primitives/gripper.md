# 夹爪 (Gripper)

命名空间：`robonix/prm/gripper`

末端抓取执行器的控制与状态接口。

## 接口列表

| 接口 | 方向 | 载荷 | 说明 |
|------|------|------|------|
| `close` | 请求 | req → status | 闭合夹爪 |
| `open` | 请求 | req → status | 张开夹爪 |
| `set_width` | 请求 | width → status | 设置目标宽度 |
| `state_width` | 输出 | `std_msgs/msg/Float64` | 当前宽度 |

## 典型组合

| 硬件 | 建议实现 |
|------|----------|
| 二指夹爪（开/关） | close, open |
| 可调宽度夹爪 | set_width, state_width |
| 全功能 | close, open, set_width, state_width |
