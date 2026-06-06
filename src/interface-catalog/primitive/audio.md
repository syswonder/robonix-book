# 音频 robonix/primitive/audio

音频原语覆盖麦克风采集与扬声器播放，是 [Liaison](../system/liaison.md) 语音 pipeline 的两端。`mic`（`topic_out`）持续吐音频块，`speaker`（`topic_in`）接收音频块播放；`list_devices` / `select_device` 让上层在多声卡设备里选具体输入/输出。

契约 TOML 在 `capabilities/primitive/audio/`，IDL 在 `capabilities/lib/audio/`。

## 接口

| 契约 ID | 模式 | 载荷（IDL） | 契约 TOML |
|---|---|---|---|
| `robonix/primitive/audio/driver` | `rpc` | `lifecycle/Driver` | `primitive/audio/driver.v1.toml` |
| `robonix/primitive/audio/mic` | `topic_out` | `audio/AudioChunk` | `primitive/audio/mic.v1.toml` |
| `robonix/primitive/audio/speaker` | `topic_in` | `audio/AudioChunk` | `primitive/audio/speaker.v1.toml` |
| `robonix/primitive/audio/list_devices` | `rpc` | `audio/ListAudioDevices` | `primitive/audio/list_devices.v1.toml` |
| `robonix/primitive/audio/select_device` | `rpc` | `audio/SelectAudioDevice` | `primitive/audio/select_device.v1.toml` |

`audio/AudioChunk` 是 mic / speaker / ASR / TTS 共用的流元素（`timestamp_ns + data + sequence + duration_s`，自带时间戳而非 `std_msgs/Header`，便于跨进程移植）。

参考实现：`examples/webots/primitives/audio_driver`（Linux ALSA）与 `audio_macos_bridge`（macOS 端采集/播放桥）。
