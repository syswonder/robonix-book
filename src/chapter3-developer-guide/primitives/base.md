# 底盘 (Base)

命名空间：`robonix/prm/base`

## 概述

底盘原语抽象移动机器人底座，涵盖导航、运动控制、位姿估计、里程计、急停、重定位等。不同机器人可能支持不同子集（如仅差速、仅导航、或带路径跟踪控制器）。

## 接口列表

| 接口 | 原语类型 | 载荷 | 说明 |
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
