# 力/力矩传感器 (Force-Torque)

命名空间：`robonix/prm/force_torque`

六维力传感器接口。

## 接口列表

| 接口 | 方向 | 载荷 | 说明 |
|------|------|------|------|
| `wrench` | 输出 | `geometry_msgs/msg/WrenchStamped` | 力/力矩（含参考系与时间戳） |

单接口，实现即提供完整能力。
