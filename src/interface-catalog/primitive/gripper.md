# 夹爪 robonix/prm/gripper

夹爪原语定义开合控制和宽度反馈。对应 TOML 尚未入库，新增时按 `rust/contracts/README.md` 规范创建版本化文件。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/prm/gripper/open` | `rpc` | `std_msgs/Empty` → `std_msgs/String` | — |
| `robonix/prm/gripper/close` | `rpc` | `std_msgs/Empty` → `std_msgs/String` | — |
| `robonix/prm/gripper/set_width` | `rpc` | `std_msgs/Float32` → `std_msgs/String` | — |
| `robonix/prm/gripper/state_width` | `pub-sub (out)` | `std_msgs/Float32` | — |

## 典型组合

简单二指夹爪实现 `open` + `close` 即可。可变宽度夹爪额外实现 `set_width` + `state_width`。
