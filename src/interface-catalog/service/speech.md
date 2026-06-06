# 语音 robonix/service/speech

语音服务提供 ASR（识别）、TTS（合成）、以及端到端对话。一元与流式都有：`asr` / `tts` 是一元 RPC，`asr_stream` / `tts_stream` / `dialog` 是流式，给低延迟语音交互用。它是 [Liaison](../system/liaison.md) 语音 pipeline 的 ASR / TTS 两段。

契约 TOML 在 `capabilities/service/speech/`，IDL 在 `capabilities/lib/{speech,asr,tts}/`。

## 接口

| 契约 ID | 模式 | 载荷（IDL） | 契约 TOML |
|---|---|---|---|
| `robonix/service/speech/driver` | `rpc` | `lifecycle/Driver` | `service/speech/driver.v1.toml` |
| `robonix/service/speech/asr` | `rpc` | `asr/Recognize` | `service/speech/asr.v1.toml` |
| `robonix/service/speech/asr_stream` | `rpc_bidirectional_stream` | `asr/RecognizeStream` | `service/speech/asr_stream.v1.toml` |
| `robonix/service/speech/tts` | `rpc` | `tts/Synthesize` | `service/speech/tts.v1.toml` |
| `robonix/service/speech/tts_stream` | `rpc_server_stream` | `tts/SynthesizeStream` | `service/speech/tts_stream.v1.toml` |
| `robonix/service/speech/dialog` | `rpc_server_stream` | `speech/StartDialog` | `service/speech/dialog.v1.toml` |
| `robonix/service/speech/speak` | `rpc` | `speech/Speak` | `service/speech/speak.v1.toml` |
| `robonix/service/speech/list_speakers` | `rpc` | `speech/ListSpeakers` | `service/speech/list_speakers.v1.toml` |

`speak` 是"合成并直接播放"的便捷入口（服务内部接扬声器）；`tts` / `tts_stream` 只回音频流，由调用方播放。`list_speakers` 列可用音色。流式契约只能走 gRPC（ROS 2 不原生支持流式 RPC）。

参考实现：`services/speech`（FunASR ASR + TTS）。
