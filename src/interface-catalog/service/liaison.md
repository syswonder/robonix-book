# Liaison 服务

## 角色

Liaison 是统一的用户交互入口。所有外部交互（rbnx chat TUI、移动 App、Web、API client）都不会直连 Pilot，而是先打到 Liaison。

文本路径直接转发，语音路径 Liaison 自己编排（mic → ASR → voiceprint → Pilot → 可选 TTS → speaker）。

合约：`robonix/system/liaison`，gRPC 服务 `robonix.contracts.SystemLiaison`，包含两个 RPC：

| RPC | 入参 | 出参（流） | 用途 |
| --- | --- | --- | --- |
| `SubmitTask` | `pilot/Task` | `pilot/PilotEvent` | 文本/API/预构造任务 |
| `StartVoiceSession` | `liaison/StartVoiceSessionRequest` | `liaison/VoiceEvent` | 语音 push-to-talk 会话 |

`SubmitTask` 流即 Pilot 的事件流原样转发；`StartVoiceSession` 流是 Liaison 自己合成的 `VoiceEvent`，其中 `event_kind=PILOT` 的事件把内嵌的 `PilotEvent` 装回 `pilot` 字段。

## rbnx chat 的连接路径

```
rbnx chat
    │
    │ 1. atlas.QueryCapabilities("robonix/system/liaison", grpc)
    │ 2. atlas.ConnectCapability(...) → endpoint
    ▼
robonix-liaison (gRPC)
    │
    ├── Enter  → SystemLiaison.SubmitTask(Task)
    │           → Pilot.SubmitTask 转发
    │
    └── Ctrl+V → SystemLiaison.StartVoiceSession(req)
                → mic → ASR → voiceprint → Pilot → (可选 TTS+speaker)
                → 每个阶段产生一个 VoiceEvent
```

User identity 走 `Task.context_json.user_id`（默认填 `local:<os_user>`）。`Task` 本身没有 `user_id` 字段，统一通过 context_json 传递，避免给 Pilot 增加新的必填字段。

## 启动

通过 rbnx boot：

```yaml
system:
  liaison:
    listen: 127.0.0.1:50081
    log: info
    pilot_endpoint: 127.0.0.1:50071  # optional, 默认从 atlas 发现
```

rbnx-cli 现在把 `liaison` 当成内置 system 二进制（和 atlas/executor/pilot 同级），spawn `robonix-liaison` 时会传 `--listen / --atlas / --pilot-endpoint / --log`。

也可以手动启动：

```bash
robonix-liaison \
  --listen 127.0.0.1:50081 \
  --atlas  127.0.0.1:50051 \
  --pilot-endpoint 127.0.0.1:50071
```

或全靠环境变量：`ROBONIX_LIAISON_PORT / ROBONIX_ATLAS_ENDPOINT / ROBONIX_PILOT_ENDPOINT`。

## 语音 pipeline 的能力依赖

| 阶段 | 合约 | provider |
| --- | --- | --- |
| 录音 | `robonix/primitive/audio/mic` | `system/audio_driver`（生产） / `mock_audio` 例子（测试） |
| ASR | `robonix/system/speech/asr` | `system/speech` (`speech_service`) |
| 声纹识别 | `robonix/system/speech/voiceprint` | **未实现**，详见下文 |
| Pilot | `robonix/system/pilot` | `robonix-pilot` |
| TTS | `robonix/system/speech/tts` | `system/speech` (`speech_service`) |
| 播放 | `robonix/primitive/audio/speaker` | `system/audio_driver` / `mock_audio` |

任何阶段缺 provider 都不会硬崩：
* 没 mic → mock 模式下用预设 transcript；非 mock 模式下报 ERROR 终止会话
* 没 ASR → 同上
* **没 voiceprint** → fallback 到客户端传来的 `client_user_id` hint，发一条 `KIND_USER_IDENTIFIED` 事件附带说明
* 没 TTS / speaker → 跳过播放，文本回复仍走 PILOT 事件

## 声纹注册识别（现状）

**当前不真正支持声纹识别**，只有合约骨架。

* 合约 `robonix/system/speech/voiceprint` 已声明（`capabilities/system/speech/voiceprint.v1.toml`）
* IDL `Identify.srv` 已定义（`lib/system/voiceprint/srv/Identify.srv`）
* gRPC client 已生成，Liaison 会调用
* **server 实现：无**。整个仓库没有任何 package 注册了这个合约的 provider

实际跑下来 `KIND_USER_IDENTIFIED` 事件的 `user_id` 永远等于客户端在 `StartVoiceSessionRequest.client_user_id` 里填的 hint（自动 prefix `voice:` 如果没带），`status_message` 会写 `no SystemSpeechVoiceprint provider — using client hint`。

要做"找几个人声看是否能区分"的真测试，需要先实现 provider：

1. 起一个 Python 包 `system/voiceprint/` 注册合约 `robonix/system/speech/voiceprint`
2. 实现 `Identify(audio_data, encoding, sample_rate_hz) → (user_id, user_name, confidence, is_known)`
3. 后端可以选 SpeechBrain ECAPA-TDNN / Resemblyzer / pyannote 等，需要先做一遍每个用户的 enrollment（采几秒说话样本算 embedding 存起来），inference 阶段拿新音频算 embedding 做 cosine 相似度

合约本身已经够，不用动 IDL，加 enrollment 接口的话再加一个 `Enroll.srv`。

## 端到端 mock 测试

仓库带了 mock 栈，不依赖任何模型权重 / 麦克风，仅验证 Liaison ↔ Pilot 的 wiring：

```bash
# build
cargo build --release -p robonix-atlas -p robonix-liaison --examples

# 在 3 个终端分别起 atlas / mock_pilot / liaison（mock 模式）
./rust/target/release/robonix-atlas --listen 127.0.0.1:50061
ROBONIX_ATLAS=127.0.0.1:50061 MOCK_PILOT_PORT=50072 \
    ./rust/target/release/examples/mock_pilot
ROBONIX_ATLAS_ENDPOINT=127.0.0.1:50061 ROBONIX_LIAISON_PORT=50082 \
    ROBONIX_LIAISON_VOICE_MOCK=1 ROBONIX_LIAISON_VOICE_MOCK_TEXT="hello world" \
    ROBONIX_PILOT_ENDPOINT=127.0.0.1:50072 \
    ./rust/target/release/robonix-liaison

# 跑 smoke test
ROBONIX_ATLAS=127.0.0.1:50061 ./rust/target/release/examples/voice_demo_client
```

期望：Stage 1 (text) 和 Stage 2 (voice mock) 都打 PASS。

带真 ASR/TTS 的端到端测（用 WAV 做麦克风源）：`examples/webots/run_tui_test.sh`。
