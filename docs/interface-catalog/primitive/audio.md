---
title: 音频
---
<span id="音频-robonixprimitiveaudio"></span>
# 音频

音频原语覆盖麦克风采集与扬声器播放，是[交互服务](../system/liaison.md)语音处理链路的两端。`mic`（`topic_out`）持续输出音频块，`speaker`（`topic_in`）接收音频块播放。多设备提供方可以实现 `list_devices` / `select_device`，客户端音频桥可以实现 `bridge_info` 来发布反向 WebSocket 访问地址；这三个接口都不是所有音频提供方的必选项。

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

`audio/AudioChunk` 是麦克风、扬声器、语音识别和语音合成共用的流元素（`timestamp_ns + data + sequence + duration_s`）。它不携带编码、采样率、声道数或采样位数，不是自描述音频容器；提供方与消费方必须通过会话或部署配置约定格式。当前交互服务的语音路径默认使用 16 kHz、单声道、`pcm_s16le`，改变格式时必须同步修改两端。

固定设备提供方可以不注册 `list_devices` / `select_device`，也可以对查询返回 `UNIMPLEMENTED`；调用方此时应继续使用当前原语的默认设备。`bridge_info` 仅适用于客户端桥，不能作为本地 ALSA 提供方的必备探针。

参考实现：

- [ALSA 音频原语](https://github.com/syswonder/primitive-audio-driver-rbnx)：使用机器人 Linux 主机上的麦克风和扬声器。
- [客户端音频桥](https://github.com/syswonder/primitive-audio-client-bridge-rbnx)：由 Robonix Client 连接机器人，并提供客户端电脑上的麦克风和扬声器。

Robonix 源码树只在 Webots 部署清单中通过远程 `main` 分支引用这两个软件包，不包含它们的 `package_manifest.yaml` 或实现源码。部署后应以 `rbnx caps -v` 的实际注册结果确认可用接口，不能只根据上面的仓库链接推断运行时能力。
