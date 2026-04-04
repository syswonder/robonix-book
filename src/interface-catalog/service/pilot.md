# Pilot 服务

推理、规划与会话运行时。VLM 驱动的 ReAct 推理循环，将 `Intent` 分解为 `TaskGraph` 逐轮执行。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/sys/runtime/pilot` | `rpc_server_stream` | `pilot/HandleIntent_Request` → stream `pilot/PilotEvent` | `sys/pilot.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。`HandleIntent` 为有状态流式 RPC，`interruptible = true`（`rbnx chat` 按 Esc 发 `AbortSession` 中断当前轮次）。

## 实现

- **二进制**：`robonix-pilot`（`rust/crates/robonix-pilot`）
- **注册**：`declare_interface_full(..., "robonix/sys/runtime/pilot")`，接口叶子名通常为 `pilot`

## 与 Executor / VLM 的衔接

Pilot 通过 Executor 下发 MCP 工具调用；若工具返回 JSON 中含 **`sensor_msgs/Image` 线格式**字段（`width` / `height` / `encoding` / base64 `data`）或顶层 `image_base64`，会将图像以多模态形式送入 VLM（详见 [Provider 注册](../../integration-guide/provider-registration.md)）。
