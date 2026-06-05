# 底盘 robonix/primitive/chassis

底盘原语覆盖移动机器人的**低层**运动控制和反馈。IDL 定义在 `capabilities/lib/chassis/`，契约 TOML 在 `capabilities/primitive/chassis/`。

> **注意**：目标式导航（`navigate` / `status` / `cancel`）不属于底盘原语，由 `robonix/service/navigation/*` 服务承担（通常 Nav2）——详见 [导航服务](../service/navigation.md)。位姿在 map 帧的查询也不在这里——那是定位服务（`service/map/pose`）的职责。底盘原语只负责下发瞬时速度（`move` / `twist_in`）和反馈底盘自身的运动事实（`odom`）。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/primitive/chassis/driver` | `rpc` | `lifecycle/Driver` | `primitive/chassis/driver.v1.toml` |
| `robonix/primitive/chassis/move` | `rpc` | `chassis/MoveCommand` → `std_msgs/String` | `primitive/chassis/move.v1.toml` |
| `robonix/primitive/chassis/twist_in` | `pub-sub (in)` | `geometry_msgs/Twist` | `primitive/chassis/twist_in.v1.toml` |
| `robonix/primitive/chassis/odom` | `pub-sub (out)` | `nav_msgs/Odometry` | `primitive/chassis/odom.v1.toml` |

`move` 是底盘的唯一运动入口——既支持瞬时速度（linear/angular 一次写入直接下发到 `cmd_vel`），也支持封顶执行时长（`duration_sec`）让一次"转 30°"或"前进 20 cm"这种短动作能完整跑完。**`move` 走 gRPC，刻意不暴露为 MCP**——它下发的是未经避障的瞬时速度，不挂到大模型的工具列表上（直接让 LLM 控速会撞东西）。导航服务（`simple_nav` / Nav2 封装 / teleop）通过 gRPC 调它；要带路径规划的运动，LLM 走 `service/navigation/navigate`，由它内部组合安全目标后再调 `move`。

> **历史变更**：早期版本有 `robonix/primitive/chassis/state` 契约，返回一个 `RobotState` 巨型消息（`base_pose + joint_state + tcp_pose + gripper`）。`base_pose` 字段实际上是 AMCL 的输出，让底盘原语去依赖一个上层定位服务——这是分层倒置（原语应该是叶子节点）。该契约已删除，消费者改为：
>
> - 想拿 odom-frame 位姿 → 订阅 `primitive/chassis/odom`
> - 想拿 map-frame 位姿 → 订阅 `service/map/pose`

## 典型组合

基础移动底盘实现 `driver` + `move` + `odom` + `twist_in`：

- `driver`：gRPC，rbnx boot 通过 `Driver(CMD_INIT)` 把底盘启起来
- `move`：gRPC（刻意不暴露为 MCP），导航服务下发瞬时速度命令
- `odom`：ROS 2，给 `service/map/*` 做融合定位用
- `twist_in`：ROS 2，Nav2 控制器把 `cmd_vel` 发到这里
