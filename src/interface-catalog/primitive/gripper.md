# 夹爪 robonix/prm/gripper

夹爪原语定义开合控制和宽度状态反馈。**仓库内尚无**对应 `rust/contracts/prm/*.toml`，新增契约时同上规范落盘。

## 预定义接口

| 契约 ID（`contract_id`） | 模式 | 说明 |
|-----------------------|------|------|
| `robonix/prm/gripper/open` | RPC | 打开夹爪 |
| `robonix/prm/gripper/close` | RPC | 关闭夹爪 |
| `robonix/prm/gripper/set_width` | RPC | 设置夹爪宽度到指定值 |
| `robonix/prm/gripper/state_width` | pub-sub (output) | 实时夹爪宽度反馈 |

## 典型组合

简单的二指夹爪实现 `open` + `close` 即可。可变宽度夹爪额外实现 `set_width` + `state_width`。
