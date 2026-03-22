# 底盘 (Base)

命名空间：`robonix/prm/base`

## 概述

底盘原语抽象**移动机器人底座**在不同产品形态下可能暴露的能力：从最简单的差速速度控制与里程计，到带全局规划的导航、急停、重定位，再到路径跟踪控制器等。不同机型往往只实现其中一部分接口，文档中的“典型组合”用于给出常见集成方式，而不是强制要求。

## 接口列表

| 接口 | 通信语义 | 载荷 | 说明 |
|------|----------|------|------|
| `navigate` | command | goal: `geometry_msgs/msg/PoseStamped` | 导航到目标位姿 |
| `move` | command | cmd_vel, odom, status | 速度控制 + 里程计反馈 |
| `stop` | command | req, success | 急停 |
| `relocalize` | command | req, success | 重定位（如 AMCL 重设） |
| `controller` | command | path, success | 路径跟踪控制器 |
| `pose_cov` | stream | `geometry_msgs/msg/PoseWithCovarianceStamped` | 带协方差的位姿估计 |
| `odom` | stream | `nav_msgs/msg/Odometry` | 里程计 |
| `goal_status` | query | goal_id → NavigationStatus | 导航目标状态查询 |
| `battery_ok` | query | req → ok | 电池是否正常 |
| `status` | query | req → res | 通用状态字符串 |
| `joint_state` | stream | `sensor_msgs/msg/JointState` | 底盘关节状态（如万向轮） |

## 典型组合

| 场景 | 建议实现 |
|------|----------|
| 差速底盘 | move, odom, stop |
| 导航底盘（Nav2） | navigate, pose_cov, goal_status, stop |
| 路径跟踪 | controller, pose_cov, odom |
| 带电池监控 | battery_ok |
