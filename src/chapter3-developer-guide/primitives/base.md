# 底盘 (Base)

命名空间：`robonix/prm/base`

移动机器人底座的能力抽象。不同机型只需实现其中一部分接口。

## 接口列表

| 接口 | 方向 | 载荷 | 说明 |
|------|------|------|------|
| `navigate` | 请求 | goal → status | 导航到目标位姿 |
| `move` | 请求 | cmd_vel → status | 速度控制 |
| `stop` | 请求 | req → success | 急停 |
| `relocalize` | 请求 | req → success | 重定位 |
| `controller` | 请求 | path → success | 路径跟踪控制器 |
| `pose_cov` | 输出 | `geometry_msgs/msg/PoseWithCovarianceStamped` | 带协方差的位姿估计 |
| `odom` | 输出 | `nav_msgs/msg/Odometry` | 里程计 |
| `goal_status` | 请求-响应 | goal_id → NavigationStatus | 导航状态查询 |
| `battery_ok` | 请求-响应 | req → ok | 电池是否正常 |
| `status` | 请求-响应 | req → res | 通用状态 |
| `joint_state` | 输出 | `sensor_msgs/msg/JointState` | 底盘关节状态 |

## 典型组合

| 场景 | 建议实现 |
|------|----------|
| 差速底盘 | move, odom, stop |
| 导航底盘（Nav2） | navigate, pose_cov, goal_status, stop |
| 路径跟踪 | controller, pose_cov, odom |
