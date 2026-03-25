# 力/力矩 robonix/prm/force_torque

力/力矩传感器原语目前只定义一个输出接口。

## 接口

| abstract_interface_id | 模式 | 方向 | 说明 |
|-----------------------|------|------|------|
| `robonix/prm/force_torque/wrench` | pub-sub | output | 六维力/力矩数据流 |

payload 类型通常为 `geometry_msgs/msg/WrenchStamped.msg`。
