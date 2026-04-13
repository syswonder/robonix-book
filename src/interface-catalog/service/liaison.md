# Liaison 服务

Liaison 是用户交互的统一入口，所有客户端（CLI、GUI、语音、机器人间交互）长期均应经由 Liaison 接入系统。当前 `rbnx chat` 直连 Pilot 属过渡调试形态。Liaison 把客户端输入构造成**任务**（`Task`）转发给 Pilot，并把 Pilot 流式返回的 `PilotEvent` 适配回客户端。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/srv/liaison` | `rpc_server_stream` | `liaison/SubmitTask_Request` → stream `pilot/PilotEvent` | `srv/liaison.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。`unified_entry = true`，`interruptible = true`。`Interrupt` 等扩展 RPC 在同服务上（`lib/liaison/srv/Interrupt.srv`），不另立契约 ID。

## 内部职责

Liaison 作为网关与适配器层，不持有会话状态，职责限于以下三项（均处于规划阶段，尚未实现）：

- 界面适配：将 CLI、GUI、语音、机器人等各类客户端输入构造为统一的**任务**载荷（附带 `user_id` 与 `session_id`），并将 `PilotEvent` 回复转为对应客户端可渲染的格式（文本、TTS 音频、结构化卡片等）。
- 语音服务调用：输入侧调用 ASR 系统服务将音频转为文本，输出侧调用 TTS 将回复转为语音。
- 用户区分：识别请求来源的用户或客户端，在任务载荷上附加身份信息，供 Pilot 进行 session 路由。

会话（Session）管理归属 Pilot 而非 Liaison：对话历史、工具调用上下文等 session 状态与 Pilot 的推理循环强绑定。Liaison 仅在任务里携带 `session_id`，不管理 session 生命周期。

接入的 4 类客户端通道：

| 通道 | 传输方式 | 说明 |
|------|---------|------|
| CLI | gRPC | `rbnx chat` 等命令行工具 |
| GUI | gRPC / WebSocket | 图形界面客户端 |
| 语音 (Mic+Speaker) | gRPC + ASR/TTS 系统服务 | 麦克风输入经 ASR 转文本，回复经 TTS 转语音 |
| 机器人交互 | gRPC / ROS 2 | 机器人间或上位机发起的自动化交互 |

## 实现

- 二进制：`robonix-liaison`（`rust/crates/robonix-liaison`）
- 注册：`declare_interface_full(..., "robonix/srv/liaison")`，接口叶子名通常为 `liaison`
