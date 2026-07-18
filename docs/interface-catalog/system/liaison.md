---
title: 交互服务
---
<span id="liaison-robonixsystemliaison"></span>
# 交互服务（Liaison）


## 角色

交互服务是带身份归一化、访问控制和语音编排的用户入口。`rbnx chat` 以及需要这些策略的移动应用、网页或接口客户端应先连接交互服务：文本路径转发给规划器，语音路径由交互服务编排（麦克风 → 流式语音识别 → 声纹 → 规划器 → 可选语音合成 → 扬声器）。非交互命令 `rbnx ask` 当前会绕过交互服务，直接连接规划器。

能力约定 TOML 在 `capabilities/system/liaison/`，接口定义语言（Interface Definition Language，IDL）文件在 `capabilities/lib/liaison/`。

## 接口

| 能力约定 ID | 模式 | 当前实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/system/liaison/submit` | `rpc_server_stream` | gRPC | [`liaison/SubmitTask`](../../reference/idl.md#liaison-srv-submittask-srv)（`pilot/Task` → 流 `pilot/PilotEvent`） | `system/liaison/submit.v1.toml` |
| `robonix/system/liaison/voice` | `rpc_server_stream` | gRPC | [`liaison/StartVoiceSession`](../../reference/idl.md#liaison-srv-startvoicesession-srv)（`StartVoiceSessionRequest` → 流 `VoiceEvent`） | `system/liaison/voice.v1.toml` |
| `robonix/system/liaison/handsfree/events` | `rpc_server_stream` | gRPC | [`liaison/WatchHandsfreeEvents`](../../reference/idl.md#liaison-srv-watchhandsfreeevents-srv) | `system/liaison/handsfree/events.v1.toml` |
| `robonix/system/liaison/handsfree/set_enabled` | `rpc` | gRPC | [`liaison/SetHandsfree`](../../reference/idl.md#liaison-srv-sethandsfree-srv) | `system/liaison/handsfree/set_enabled.v1.toml` |
| `robonix/system/liaison/handsfree/status` | `rpc` | gRPC | [`liaison/GetHandsfreeStatus`](../../reference/idl.md#liaison-srv-gethandsfreestatus-srv) | `system/liaison/handsfree/status.v1.toml` |

内置交互服务将上表 5 条约定全部以 gRPC 注册。

`submit` 的事件流就是规划器事件流原样转发；`voice` 的流是交互服务自己合成的 `VoiceEvent`，其中 `event_kind=PILOT` 的事件把内嵌 `PilotEvent` 装回 `pilot` 字段。`handsfree/*` 提供免手持模式的开关、状态查询和只读事件流。用户身份走 `Task.context_json.user_id`（默认 `local:<os_user>`）。

## `rbnx chat` 的连接路径

```text
rbnx chat
    │  atlas.find(contract="robonix/system/liaison/submit", grpc) → endpoint
    ▼
robonix-liaison (gRPC)
    ├── Enter  → liaison.SubmitTask(Task) → Pilot.SubmitTask 转发
    └── F2     → liaison.StartVoiceSession(req)
                 → mic → ASR → voiceprint → Pilot → (可选 TTS+speaker)
                 → 每阶段产生一个 VoiceEvent
```

## 语音处理链路的能力依赖

| 阶段 | 能力约定 | 参考提供方 |
|---|---|---|
| 录音 | `robonix/primitive/audio/mic` | `audio_driver` / `audio_client_bridge` |
| 自动语音识别（Automatic Speech Recognition，ASR） | `robonix/service/speech/asr_stream` | `services/speech` |
| 声纹 | `robonix/service/voiceprint/identify` | `services/voiceprint` |
| Pilot | `robonix/system/pilot` | `robonix-pilot` |
| 语音合成（Text-to-Speech，TTS） | `robonix/service/speech/tts` | `services/speech` |
| 播放 | `robonix/primitive/audio/speaker` | `audio_driver` / `audio_client_bridge` |

依赖缺失时的行为如下：

- 没有麦克风或语音识别 → 模拟模式使用预设文本；非模拟模式发送错误并终止会话。
- 没有声纹提供方 → 访问控制关闭时可回退到客户端 `client_user_id` 提示；访问控制开启时，该提示不能授权语音会话，必须由已登记声纹通过置信度阈值和允许列表，否则会话被拒绝。
- 没有语音合成或扬声器 → 跳过播放，文本回复仍走 `PILOT` 事件。

## 启动

`liaison` 是内置系统二进制（与 `atlas`、`executor`、`pilot` 同级）；部署清单配置 `system.liaison` 时由 `rbnx boot` 拉起：

```yaml
system:
  liaison:
    listen: 127.0.0.1:50081
    log: info
    pilot_endpoint: 127.0.0.1:50071  # 可选，默认从 atlas 发现
```

也可手动起 `robonix-liaison --listen … --atlas … --pilot-endpoint …`，或用环境变量 `ROBONIX_LIAISON_PORT / ROBONIX_ATLAS_ENDPOINT / ROBONIX_PILOT_ENDPOINT`。
