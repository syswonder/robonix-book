---
title: 语音
---
<span id="语音-robonixservicespeech"></span>
# 语音

语音服务提供自动语音识别（Automatic Speech Recognition，ASR）、语音合成（Text-to-Speech，TTS）和唤醒词检测。一元与流式都有：`asr` 和 `tts` 是一元 RPC，`asr_stream`、`tts_stream`、`dialog` 和 `wake_word` 是流式。当前版本不支持 `dialog`；连续语音交互与轮次编排使用 [Liaison](../system/liaison.md)。

能力约定 TOML 在 `capabilities/service/speech/`；直接和嵌套使用的接口定义语言（Interface Definition Language，IDL）文件位于 `capabilities/lib/{speech,asr,tts,audio,lifecycle}/`。

新软件包省略 Driver 条目，由框架自动注册共享的 `robonix/lifecycle/driver`；显式选择共享 Driver 的行为相同。未实现生命周期回调时，框架记录警告并执行空操作。

:::warning[后向兼容：语音服务命名空间 Driver]
`robonix/service/speech/driver`、`lifecycle/Driver` 和 `service/speech/driver.v1.toml` 只用于仍由软件包自行维护 Driver TOML 的旧实现。目前仍可使用，但计划迁移到共享 Driver；两种 Driver 不能同时注册。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

## 接口

| 能力约定 ID | 模式 | 默认实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/speech/asr` | `rpc` | gRPC | [`asr/Recognize`](../../reference/idl.md#asr-srv-recognize-srv) | `service/speech/asr.v1.toml` |
| `robonix/service/speech/asr_stream` | `rpc_bidirectional_stream` | gRPC | [`asr/RecognizeStream`](../../reference/idl.md#asr-srv-recognizestream-srv) | `service/speech/asr_stream.v1.toml` |
| `robonix/service/speech/tts` | `rpc` | gRPC | [`tts/Synthesize`](../../reference/idl.md#tts-srv-synthesize-srv) | `service/speech/tts.v1.toml` |
| `robonix/service/speech/tts_stream` | `rpc_server_stream` | gRPC | [`tts/SynthesizeStream`](../../reference/idl.md#tts-srv-synthesizestream-srv) | `service/speech/tts_stream.v1.toml` |
| `robonix/service/speech/dialog` | `rpc_server_stream` | gRPC（已注册，实现不可用） | [`speech/StartDialog`](../../reference/idl.md#speech-srv-startdialog-srv) | `service/speech/dialog.v1.toml` |
| `robonix/service/speech/speak` | `rpc` | 模型上下文协议（Model Context Protocol，MCP） | [`speech/Speak`](../../reference/idl.md#speech-srv-speak-srv) | `service/speech/speak.v1.toml` |
| `robonix/service/speech/list_speakers` | `rpc` | MCP | [`speech/ListSpeakers`](../../reference/idl.md#speech-srv-listspeakers-srv) | `service/speech/list_speakers.v1.toml` |
| `robonix/service/speech/wake_word` | `rpc_client_stream` | gRPC | [`speech/DetectWakeWord`](../../reference/idl.md#speech-srv-detectwakeword-srv) | `service/speech/wake_word.v1.toml` |

`speak` 是“合成并直接播放”的 MCP 入口；其 `target` 填 `list_speakers` 返回的扬声器 `provider_id`，留空时使用部署配置的默认扬声器，否则取第一个可用实例。`list_speakers` 只返回通过 gRPC 暴露扬声器能力的运行实例，因此返回的 ID 可以直接用于 `speak.target`；仅通过 ROS 2 暴露的扬声器不会出现在该列表中。`tts` 返回完整音频缓冲区，`tts_stream` 返回音频分块，两者都由调用方播放。`list_speakers` 列出的是播放目标，不是 TTS 音色。流式能力约定只能走 gRPC（ROS 2 不原生支持流式 RPC）。

参考实现：Robonix 源码中的 [`services/speech`](https://github.com/syswonder/robonix/tree/181d3eb974fd495a795ed120a0a4c6e6f342d179/services/speech)。`local` 配置组合使用 Whisper 单次识别、FunASR 流式识别和 Edge TTS；其中 ASR 模型在本机运行，Edge TTS 仍需要访问 Microsoft 服务。部署也可选择 Tencent 或自定义后端。某个后端初始化失败时，对应的 ASR、TTS 或唤醒词调用会返回 `UNAVAILABLE`。

> 实现依据：[软件包清单](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/services/speech/package_manifest.yaml) · [gRPC / MCP 注册](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/services/speech/speech_service/service.py) · [`DialogEvent` IDL](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/capabilities/lib/speech/msg/DialogEvent.msg)
