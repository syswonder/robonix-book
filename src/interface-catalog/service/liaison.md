# Liaison 服务

统一入口（语音 / 多模态等），**载荷契约与 Pilot 相同**：`Intent` → 流式 `PilotEvent`。控制面 **`contract_id = robonix/sys/runtime/liaison`**。

## 契约（与 `rust/contracts/sys/liaison.v1.toml` 同步）

| 项 | 值 |
|----|-----|
| **契约 ID（`contract_id`）** | `robonix/sys/runtime/liaison` |
| **版本** | `1` |
| **`kind`** | `service` |
| **`[io.srv]`** | `srv = "liaison/srv/HandleIntent"` |
| **`[mode].type`** | `rpc_server_stream` |
| **`[semantics]`** | `unified_entry = true`，`interruptible = true` |

## 具体 gRPC（robonix-codegen / `lib/liaison`）

**`LiaisonService.HandleIntent(HandleIntent_Request)`** → stream **`PilotEvent`**。**`Interrupt`** 等为同服务上的扩展 RPC（见 `lib/liaison/srv/Interrupt.srv`），本 TOML 不拆额外契约 ID。

## 实现与注册

- 二进制示例：**`robonix-liaison`**。
- 注册时 **`declare_interface_full(..., "robonix/sys/runtime/liaison")`**，接口叶子名通常为 `liaison`。
