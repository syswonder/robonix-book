# Liaison robonix/system/liaison

[toc]

## 角色

Liaison 是统一的用户交互入口。所有外部交互（rbnx chat TUI、移动 App、Web、API client）都不直连 Pilot，而是先打到 Liaison：文本路径直接转发给 Pilot，语音路径由 Liaison 自己编排（mic → ASR → voiceprint → Pilot → 可选 TTS → speaker）。

能力约定 TOML 在 `capabilities/system/liaison/`，IDL 在 `capabilities/lib/liaison/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/system/liaison/submit` | `rpc_server_stream` | [`liaison/SubmitTask`](../../reference/idl.md#liaison-srv-submittask-srv)（`pilot/Task` → 流 `pilot/PilotEvent`） | `system/liaison/submit.v1.toml` |
| `robonix/system/liaison/voice` | `rpc_server_stream` | [`liaison/StartVoiceSession`](../../reference/idl.md#liaison-srv-startvoicesession-srv)（`StartVoiceSessionRequest` → 流 `VoiceEvent`） | `system/liaison/voice.v1.toml` |
| `robonix/system/liaison/handsfree/events` | `rpc_server_stream` | [`liaison/WatchHandsfreeEvents`](../../reference/idl.md#liaison-srv-watchhandsfreeevents-srv) | `system/liaison/handsfree/events.v1.toml` |
| `robonix/system/liaison/handsfree/set_enabled` | `rpc` | [`liaison/SetHandsfree`](../../reference/idl.md#liaison-srv-sethandsfree-srv) | `system/liaison/handsfree/set_enabled.v1.toml` |
| `robonix/system/liaison/handsfree/status` | `rpc` | [`liaison/GetHandsfreeStatus`](../../reference/idl.md#liaison-srv-gethandsfreestatus-srv) | `system/liaison/handsfree/status.v1.toml` |

`submit` 的事件流就是 Pilot 事件流原样转发；`voice` 的流是 Liaison 自己合成的 `VoiceEvent`，其中 `event_kind=PILOT` 的事件把内嵌 `PilotEvent` 装回 `pilot` 字段。`handsfree/*` 提供免手持模式的开关、状态查询和只读事件流。用户身份走 `Task.context_json.user_id`（默认 `local:<os_user>`）。

## rbnx chat 的连接路径

```
rbnx chat
    │  atlas.find(contract="robonix/system/liaison/submit", grpc) → endpoint
    ▼
robonix-liaison (gRPC)
    ├── Enter  → liaison.SubmitTask(Task) → Pilot.SubmitTask 转发
    └── Ctrl+V → liaison.StartVoiceSession(req)
                 → mic → ASR → voiceprint → Pilot → (可选 TTS+speaker)
                 → 每阶段产生一个 VoiceEvent
```

## 语音 pipeline 的能力依赖

| 阶段 | 能力约定 | 参考 provider |
|---|---|---|
| 录音 | `robonix/primitive/audio/mic` | `audio_driver` / `audio_macos_bridge` |
| ASR | `robonix/service/speech/asr` | `services/speech` |
| 声纹 | `robonix/service/voiceprint/identify` | `services/voiceprint` |
| Pilot | `robonix/system/pilot` | `robonix-pilot` |
| TTS | `robonix/service/speech/tts` | `services/speech` |
| 播放 | `robonix/primitive/audio/speaker` | `audio_driver` / `audio_macos_bridge` |

任何阶段缺 provider 都不会中断退出：

- 没 mic / ASR → mock 模式用预设 transcript；非 mock 模式发 ERROR 终止会话。
- 没 voiceprint provider → fallback 到客户端 `client_user_id` hint，发一条 `KIND_USER_IDENTIFIED` 事件附 `using client hint` 说明。
- 没 TTS / speaker → 跳过播放，文本回复仍走 PILOT 事件。

## 启动

`liaison` 是内置 system 二进制（与 atlas / executor / pilot 同级）；部署清单配置 `system.liaison` 时由 `rbnx boot` 拉起：

```yaml
system:
  liaison:
    listen: 127.0.0.1:50081
    log: info
    pilot_endpoint: 127.0.0.1:50071  # 可选，默认从 atlas 发现
```

也可手动起 `robonix-liaison --listen … --atlas … --pilot-endpoint …`，或用环境变量 `ROBONIX_LIAISON_PORT / ROBONIX_ATLAS_ENDPOINT / ROBONIX_PILOT_ENDPOINT`。
