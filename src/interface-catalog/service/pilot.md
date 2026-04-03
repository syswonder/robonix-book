# Pilot 服务

推理、规划与会话运行时。控制面 `DeclareInterface` 建议使用 **`contract_id = robonix/sys/runtime/pilot`**（与下表一致）；数据面由 **`PilotService`** 暴露。

## 契约（与 `rust/contracts/sys/pilot.v1.toml` 同步）

| 项 | 值 |
|----|-----|
| **契约 ID（`contract_id`）** | `robonix/sys/runtime/pilot` |
| **版本** | `1`（TOML `version`，不拼进 `contract_id`） |
| **`kind`** | `service` |
| **`[io].input`** | `pilot/msg/Intent` |
| **`[io].output`** | `pilot/msg/PilotEvent` |
| **`[mode].type`** | `stream_out`（`Intent` 入 → `PilotEvent` 流） |
| **`[semantics]`** | `stateful = true`，`interruptible = true` |

## 具体 gRPC（robonix-codegen / `lib/pilot`）

TOML 注释：**Payload 级** I/O 如上；**命名 RPC** 由 `lib/pilot/srv/HandleIntent.srv` 生成，即 **`PilotService.HandleIntent(HandleIntent_Request)`**。`robonix_contracts.proto` 中可能表现为泛型 `Stream(Intent)` 门面，见 **`rust/contracts/README.md`**。

## 实现与注册

- 二进制示例：**`robonix-pilot`**（`rust/crates/robonix-pilot`）。
- 注册时 **`declare_interface_full(..., "robonix/sys/runtime/pilot")`**，接口叶子名通常为 `pilot`。

## 与 Executor / VLM 的衔接（工具结果与图像）

Pilot 通过 **Executor** 调用 MCP 工具；若工具返回的 JSON 解析为 **`sensor_msgs/Image` 线对象**（`width` / `height` / `encoding` / base64 **`data`**）或顶层 **`image_base64`**，会将图像以多模态形式送入 **VLM**（详见 [接入指南 · Provider 注册](../../integration-guide/provider-registration.md)）。终端侧 **`rbnx chat`** 按 **Esc** 可向 Pilot 发送 **`AbortSession`** 中断当前轮次。
