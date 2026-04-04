# Liaison 服务

统一入口，接收文本/语音等多模态输入，转发至 Pilot。载荷契约与 Pilot 相同：`Intent` → 流式 `PilotEvent`。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/sys/runtime/liaison` | `rpc_server_stream` | `liaison/HandleIntent_Request` → stream `pilot/PilotEvent` | `sys/liaison.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。`unified_entry = true`，`interruptible = true`。`Interrupt` 等扩展 RPC 在同服务上（`lib/liaison/srv/Interrupt.srv`），不另立契约 ID。

## 实现

- **二进制**：`robonix-liaison`（`rust/crates/robonix-liaison`）
- **注册**：`declare_interface_full(..., "robonix/sys/runtime/liaison")`，接口叶子名通常为 `liaison`
