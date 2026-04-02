# 力/力矩 robonix/prm/force_torque

力/力矩传感器原语目前只定义一个输出接口。**仓库内尚无**对应 `rust/contracts/prm/*.toml`。

## 接口

| 契约 ID（`contract_id`） | 模式 | 方向 | 说明 |
|-----------------------|------|------|------|
| `robonix/prm/force_torque/wrench` | pub-sub | output | 六维力/力矩数据流 |

payload 类型通常为 `geometry_msgs/msg/WrenchStamped.msg`。
