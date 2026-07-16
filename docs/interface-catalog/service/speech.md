<span id="语音-robonixservicespeech"></span>
# 语音

语音服务提供 ASR（识别）、TTS（合成）和唤醒词检测。一元与流式都有：`asr` / `tts` 是一元 RPC，`asr_stream` / `tts_stream` / `dialog` / `wake_word` 是流式。Robonix `8c2551ce` 中的 `dialog` 虽已注册，但实现既不处理 VAD、ASR、轮次或音频，也构造了不符合 `DialogEvent` IDL 的响应，当前不可调用；实际语音 pipeline 由 [Liaison](../system/liaison.md) 编排。

能力约定 TOML 在 `capabilities/service/speech/`；直接和嵌套 IDL 位于 `capabilities/lib/{speech,asr,tts,audio,lifecycle}/`。

## 接口

| 能力约定 ID | 模式 | 默认实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/speech/driver` | `rpc` | gRPC（生命周期） | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `service/speech/driver.v1.toml` |
| `robonix/service/speech/asr` | `rpc` | gRPC | [`asr/Recognize`](../../reference/idl.md#asr-srv-recognize-srv) | `service/speech/asr.v1.toml` |
| `robonix/service/speech/asr_stream` | `rpc_bidirectional_stream` | gRPC | [`asr/RecognizeStream`](../../reference/idl.md#asr-srv-recognizestream-srv) | `service/speech/asr_stream.v1.toml` |
| `robonix/service/speech/tts` | `rpc` | gRPC | [`tts/Synthesize`](../../reference/idl.md#tts-srv-synthesize-srv) | `service/speech/tts.v1.toml` |
| `robonix/service/speech/tts_stream` | `rpc_server_stream` | gRPC | [`tts/SynthesizeStream`](../../reference/idl.md#tts-srv-synthesizestream-srv) | `service/speech/tts_stream.v1.toml` |
| `robonix/service/speech/dialog` | `rpc_server_stream` | gRPC（已注册，实现不可用） | [`speech/StartDialog`](../../reference/idl.md#speech-srv-startdialog-srv) | `service/speech/dialog.v1.toml` |
| `robonix/service/speech/speak` | `rpc` | MCP | [`speech/Speak`](../../reference/idl.md#speech-srv-speak-srv) | `service/speech/speak.v1.toml` |
| `robonix/service/speech/list_speakers` | `rpc` | MCP | [`speech/ListSpeakers`](../../reference/idl.md#speech-srv-listspeakers-srv) | `service/speech/list_speakers.v1.toml` |
| `robonix/service/speech/wake_word` | `rpc_client_stream` | gRPC | [`speech/DetectWakeWord`](../../reference/idl.md#speech-srv-detectwakeword-srv) | `service/speech/wake_word.v1.toml` |

`speak` 是“合成并直接播放”的 MCP 入口；其 `target` 填 `list_speakers` 返回的扬声器 `provider_id`，留空时使用部署配置的默认扬声器，否则取第一个可用提供方。`tts` 返回完整音频缓冲区，`tts_stream` 返回音频分块，两者都由调用方播放。`list_speakers` 列出的是播放目标，不是 TTS 音色。流式能力约定只能走 gRPC（ROS 2 不原生支持流式 RPC）。

参考实现：Robonix [`8c2551ce`](https://github.com/syswonder/robonix/tree/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/speech) 中的 `services/speech`。`local` profile 使用 Whisper 单次识别、FunASR 流式识别和 Edge TTS；其中 ASR 模型在本机运行，Edge TTS 仍需要访问 Microsoft 服务。部署也可选择 Tencent 或自定义后端。某个后端初始化失败时，对应的 ASR、TTS 或唤醒词调用会返回 `UNAVAILABLE`。

> 实现依据：[package manifest](https://github.com/syswonder/robonix/blob/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/speech/package_manifest.yaml) · [gRPC / MCP 注册](https://github.com/syswonder/robonix/blob/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/speech/speech_service/service.py) · [`DialogEvent` IDL](https://github.com/syswonder/robonix/blob/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/capabilities/lib/speech/msg/DialogEvent.msg)
