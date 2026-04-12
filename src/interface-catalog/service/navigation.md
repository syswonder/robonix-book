# 导航服务（`robonix/srv/navigation/*`）

导航服务接收**目标式**运动指令（"去这个位姿"），后台规划路径并控制底盘完成。Robonix
用 3 条 RPC 约定了导航生命周期的最小接口。

参考实现：`rust/examples/packages/tiago_sim_stack` + 容器内 Nav2（planner + controller + behavior tree）。导航能力属于"规划/决策"而不是"硬件原语"，所以放在 `srv/` 命名空间。

## 接口

| Contract | 请求 → 响应 | 行为 |
|----------|---|---|
| `srv/navigation/navigate` | `geometry_msgs/PoseStamped` → `{goal_id, accepted, message}` | 把 `PoseStamped` 作为目标发送给规划器，返回一个 `goal_id` |
| `srv/navigation/status` | `{goal_id}` → `{known, status, terminal, message}` | 查询指定 goal 的状态（pending / active / succeeded / aborted / canceled） |
| `srv/navigation/cancel` | `{goal_id}` → `{accepted, message}` | 取消指定 goal；返回 ack |

## 为什么这 3 条够支撑 Nav2？

Nav2 的对外编程模型本质是一个 [**`nav2_msgs/action/NavigateToPose`**](https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action) action server：

```
# Nav2 action
Goal:       geometry_msgs/PoseStamped pose    ←  robonix 的 navigate 输入
Result:     builtin_interfaces/Empty          ←  robonix 的 status(terminal) 响应
Feedback:   PoseStamped current_pose          ←  由 prm/base/odom 提供（不需要再走契约）
```

ROS 2 action 实质是"RPC + 状态流 + cancel"三件套，在 Robonix 我们把它拆成三个正交 RPC：

| ROS 2 action 语义 | Robonix 契约 | 备注 |
|---|---|---|
| `send_goal()` | `srv/navigation/navigate` | 一次性把目标扔进去，拿 goal_id |
| `get_result()` / feedback → `SUCCEEDED/ABORTED/CANCELED` | `srv/navigation/status`（poll） | 上层轮询；也可以订阅 `goal_status` topic |
| `cancel_goal()` | `srv/navigation/cancel` | 幂等 |

这三条 RPC 能完整映射 Nav2 action server 的所有外部状态转移，Agent 侧只用最简单的
"发目标 / 查状态 / 取消"三个 tool call 就可以驱动 Nav2。

### 为什么不把 Nav2 action 原样暴露？

- **tool-use 友好**：LLM 生成 tool call 时，最容易的是"一次请求一次回应"的 unary RPC。Agent 不需要、也不应该维护 ROS 2 action client 的长生命期句柄。
- **跨 transport**：Robonix 契约要能在 gRPC / MCP / ROS 2 三种 transport 上实现同一语义。Action 强绑 ROS 2，不适合做统一抽象。
- **轮询/订阅解耦**：`status` 是 pull 模式，简单；需要 push 的可以订阅 `prm/base/goal_status` topic。

## 实现层 — Nav2 怎么接进来

`tiago_sim_stack/tiago_bridge` 在 provider 侧做两件事：

1. **对上**：`DeclareInterface` 声明 `srv/navigation/{navigate,status,cancel}` 三个 MCP tool，Agent 通过 MCP 调用。
2. **对下**：bridge 内部持有一个 `rclpy` action client，把 RPC 请求翻译成 Nav2 action goal/cancel，把 result/feedback 翻译回 Robonix 的 `status` 结构。

```
Agent ──MCP──► navigation/navigate(pose)
               │
               ▼
    tiago_bridge ──rclpy action client──► Nav2 BT navigator
                                              │
                                              ▼
                                         规划 (planner_server)
                                         控制 (controller_server)
                                         恢复 (behavior_server)
                                              │
                                              ▼
                                         cmd_vel ──► prm/base/twist_in
```

下游栅格/扫描由 SLAM 服务（`srv/common/map/{occupancy_grid, scan_2d}`）产生，直接进
Nav2 costmap。整条链路不需要 Robonix 层额外插手。

## TODO

- **Feedback stream**：目前没有契约提供"导航过程实时进度"（当前位姿、剩余距离、偏离路径等）。Agent 如果要 monitor，必须轮询 `status`。可以新增 `srv/navigation/feedback` 作为 `topic_out` 流。
- **多目标 / 路径点**：`navigate` 只接受单个 `PoseStamped`。Nav2 支持 `NavigateThroughPoses`，可扩展为 `srv/navigation/navigate_through`。
- **语义目标**：`navigate` 只接几何位姿。与 `srv/common/map/semantic`（规划中）配合后，可扩 `navigate_to(place_name)`。
- **AMR 行为**：取消后的恢复策略（back to home、hold、continue last）还没标准化。
