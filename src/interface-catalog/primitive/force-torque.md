# 力/力矩 robonix/prm/force_torque

力/力矩传感器原语。对应 TOML 尚未入库，新增时按 `rust/contracts/README.md` 规范创建版本化文件。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/prm/force_torque/wrench` | `pub-sub (out)` | `geometry_msgs/WrenchStamped` | — |
