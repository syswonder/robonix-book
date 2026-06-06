# 导航 robonix/service/navigation

导航服务承担**目标式**运动：消费方给一个 map 帧目标，服务内部做路径规划 + 避障，组合调用底盘原语（`primitive/chassis/move` / `twist_in`）把机器人送到位。和底盘原语的瞬时速度严格分层——LLM 永远走导航服务，不直接控速。

契约 TOML 在 `capabilities/service/navigation/`，IDL 在 `capabilities/lib/navigation/`。

## 接口

| 契约 ID | 模式 | 载荷（IDL） | 契约 TOML |
|---|---|---|---|
| `robonix/service/navigation/driver` | `rpc` | `lifecycle/Driver` | `service/navigation/driver.v1.toml` |
| `robonix/service/navigation/navigate` | `rpc` | `navigation/Navigate` | `service/navigation/navigate.v1.toml` |
| `robonix/service/navigation/status` | `rpc` | `navigation/GetNavigationStatus` | `service/navigation/status.v1.toml` |
| `robonix/service/navigation/cancel` | `rpc` | `navigation/CancelNavigation` | `service/navigation/cancel.v1.toml` |

`navigate(goal: geometry_msgs/PoseStamped)` 返回 provider 分配的 `goal_id`，消费方拿它去 `status` / `cancel` 寻址同一个目标。`status_message` 只是一句人类可读的说明（接受/拒绝的原因），不是结构化数据，消费方别去解析它的内容。

参考实现：`examples/webots/services/simple_nav`——自研的 A\* 栅格规划器（8 邻接 + 障碍膨胀）加路径跟随，往 `chassis/twist_in` 发 `cmd_vel`。**不依赖 Nav2**。
