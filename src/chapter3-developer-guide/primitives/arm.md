# 机械臂 (Arm)

命名空间：`robonix/prm/arm`

笛卡尔空间与关节空间的控制接口。

## 接口列表

| 接口 | 方向 | 载荷 | 说明 |
|------|------|------|------|
| `move_ee` | 请求 | pose → status | 末端执行器位姿控制 |
| `move_joint` | 请求 | joint_positions → status | 关节位置控制 |
| `joint_trajectory` | 请求 | trajectory → status | 关节轨迹执行 |
| `state_joint` | 输出 | `sensor_msgs/msg/JointState` | 关节状态 |

## 典型组合

| 场景 | 建议实现 |
|------|----------|
| 仅笛卡尔控制 | move_ee, state_joint |
| 仅关节控制 | move_joint, state_joint |
| 轨迹控制 | joint_trajectory, state_joint |
| 全功能 | 以上全部 |
