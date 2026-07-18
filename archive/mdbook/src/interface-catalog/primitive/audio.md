# 音频 robonix/primitive/audio

音频原语覆盖麦克风采集与扬声器播放，是 [Liaison](../system/liaison.md) 语音 pipeline 的两端。`mic`（`topic_out`）持续吐音频块，`speaker`（`topic_in`）接收音频块播放；`list_devices` / `select_device` 让上层在多声卡设备里选具体输入/输出，`bridge_info` 用于发现客户端音频桥的反向 WebSocket endpoint。

能力约定 TOML 在 `capabilities/primitive/audio/`，IDL 在 `capabilities/lib/audio/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/primitive/audio/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/audio/driver.v1.toml` |
| `robonix/primitive/audio/bridge_info` | `rpc` | [`audio/GetAudioBridgeInfo`](../../reference/idl.md#audio-srv-getaudiobridgeinfo-srv) | `primitive/audio/bridge_info.v1.toml` |
| `robonix/primitive/audio/mic` | `topic_out` | [`audio/AudioChunk`](../../reference/idl.md#audio-msg-audiochunk-msg) | `primitive/audio/mic.v1.toml` |
| `robonix/primitive/audio/speaker` | `topic_in` | [`audio/AudioChunk`](../../reference/idl.md#audio-msg-audiochunk-msg) | `primitive/audio/speaker.v1.toml` |
| `robonix/primitive/audio/list_devices` | `rpc` | [`audio/ListAudioDevices`](../../reference/idl.md#audio-srv-listaudiodevices-srv) | `primitive/audio/list_devices.v1.toml` |
| `robonix/primitive/audio/select_device` | `rpc` | [`audio/SelectAudioDevice`](../../reference/idl.md#audio-srv-selectaudiodevice-srv) | `primitive/audio/select_device.v1.toml` |

`audio/AudioChunk` 是 mic / speaker / ASR / TTS 共用的流元素（`timestamp_ns + data + sequence + duration_s`，自带时间戳而非 `std_msgs/Header`，便于跨进程移植）。

参考实现：`examples/webots/primitives/audio_driver`（Linux ALSA）与 `audio_macos_bridge`（macOS 端采集/播放桥）。
