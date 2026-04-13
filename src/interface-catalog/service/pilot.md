# Pilot 服务

推理、规划与会话运行时。VLM 驱动的 ReAct 推理循环：接收 Liaison 提交的**任务**（`Task`），逐轮规划出 RTDL **方案**（`Plan`，结构化技能图），交给 Executor 执行。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/srv/pilot` | `rpc_server_stream` | `pilot/SubmitTask_Request` → stream `pilot/PilotEvent` | `srv/pilot.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。`SubmitTask` 为有状态流式 RPC，`interruptible = true`（`rbnx chat` 按 Esc 发 `AbortSession` 中断当前轮次）。

## 实现

- **二进制**：`robonix-pilot`（`rust/crates/robonix-pilot`）
- **注册**：`declare_interface_full(..., "robonix/srv/pilot")`，接口叶子名通常为 `pilot`

## 与 Executor / VLM 的衔接

每一轮，Pilot 把 VLM 的输出整理成 RTDL 方案下发给 Executor。Executor 调 MCP / gRPC / Built-in 工具，结果回流给 Pilot，进入下一轮 ReAct。

若工具返回 JSON 中含 **`sensor_msgs/Image` 线格式**字段（`width` / `height` / `encoding` / base64 `data`）或顶层 `image_base64`，Pilot 会将图像以多模态形式送入 VLM（详见 [Provider 注册](../../integration-guide/provider-registration.md)）。
