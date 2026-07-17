---
title: 声纹
---
<span id="声纹-robonixservicevoiceprint"></span>
# 声纹

声纹服务做说话人识别：先给每个用户做一遍 enrollment（采集说话样本并计算 embedding），识别阶段再对新音频做相似度匹配。[Liaison](../system/liaison.md) 用识别结果标注用户；启用访问控制后，还会检查置信度和用户 allow-list。

能力约定 TOML 在 `capabilities/service/voiceprint/`，接口定义语言（Interface Definition Language，IDL）文件在 `capabilities/lib/voiceprint/`。

> 表中的命名空间 Driver 是已有软件包的兼容接口。新软件包省略 Driver 条目时由框架自动使用共享的 `robonix/lifecycle/driver`；显式共享仍受支持，两种 Driver 只能选择一条。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。

## 接口

| 能力约定 ID | 模式 | 默认实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/voiceprint/driver` | `rpc` | gRPC（生命周期） | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `service/voiceprint/driver.v1.toml` |
| `robonix/service/voiceprint/enroll` | `rpc` | gRPC | [`voiceprint/Enroll`](../../reference/idl.md#voiceprint-srv-enroll-srv) | `service/voiceprint/enroll.v1.toml` |
| `robonix/service/voiceprint/identify` | `rpc` | gRPC | [`voiceprint/Identify`](../../reference/idl.md#voiceprint-srv-identify-srv) | `service/voiceprint/identify.v1.toml` |
| `robonix/service/voiceprint/list` | `rpc` | gRPC + 模型上下文协议（Model Context Protocol，MCP） | [`voiceprint/ListEnrolled`](../../reference/idl.md#voiceprint-srv-listenrolled-srv) | `service/voiceprint/list.v1.toml` |
| `robonix/service/voiceprint/delete` | `rpc` | gRPC | [`voiceprint/DeleteEnrolled`](../../reference/idl.md#voiceprint-srv-deleteenrolled-srv) | `service/voiceprint/delete.v1.toml` |

`identify(audio_data, encoding, sample_rate_hz)` 返回最接近的 `user_id`、`user_name`、余弦相似度 `confidence`、阈值判定 `is_known` 和 `error`。默认实现支持 `pcm_s16le` 与 WAV；空 `encoding` 按 `pcm_s16le` 处理，`sample_rate_hz=0` 按 16 kHz 处理，其他采样率会重采样到 16 kHz。

`enroll` 还需要调用方提供稳定的 `user_id` 与显示用 `user_name`。默认实现会拒绝重复的 ID、显示名，以及相似度达到同一阈值的已注册声音。`list` 返回 JSON 用户数组和数量；`delete` 按 `user_id` 删除，ID 不存在时也返回成功。

若声纹提供方不可用，交互服务可把客户端提示作为身份回退值，但它不能绕过已启用的访问控制。访问控制启用时，只有已注册、置信度达到阈值且位于允许列表中的声纹可以通过；其他语音请求都会被拒绝。

参考实现：Robonix 源码中的 [`services/voiceprint`](https://github.com/syswonder/robonix/tree/181d3eb974fd495a795ed120a0a4c6e6f342d179/services/voiceprint)，使用 SpeechBrain ECAPA-TDNN embedding 与余弦相似度，模型来自 ModelScope 的 `speechbrain/spkrec-ecapa-voxceleb`。注册数据持久化到 `<data_dir>/enrolled.json`，写入通过临时文件和原子替换完成。

## 生命周期与配置

提供方注册后等待 `Driver(CMD_INIT)`。参考实现在 `on_init` 中校验实例配置并打开注册数据库，但不加载 ECAPA-TDNN 模型；初始化失败会返回错误，提供方进入 `ERROR`。相同配置的重复初始化是幂等操作；已初始化后再传入不同配置会返回错误。

初始化成功后，启动器发送 `CMD_ACTIVATE`。参考实现在 `on_activate` 中加载 ECAPA-TDNN 模型，加载失败时不会进入 `ACTIVE`。`identify` 和 `enroll` 需要处于激活状态；`list` 和 `delete` 只需要注册数据库已经初始化。`CMD_DEACTIVATE` 会释放模型但保留配置和注册数据库，后续可以再次激活；`CMD_SHUTDOWN` 也会释放模型。业务 RPC 不会自行加载模型或补做初始化。

| `config:` 键 | 环境变量回退 | 默认值 | 用途 |
|---|---|---|---|
| `data_dir` | `VOICEPRINT_DATA_DIR` | `services/voiceprint/rbnx-build/data` | `enrolled.json` 所在目录 |
| `threshold` | `VOICEPRINT_THRESHOLD` | `0.25` | `is_known` 与重复声音检查的最低余弦相似度 |
| `device` | `VOICEPRINT_DEVICE` | CUDA 可用时为 `cuda:0`，否则为 `cpu` | ECAPA-TDNN 推理设备 |

实例 `config:` 或 `rbnx start --set` 中的同名值优先于环境变量。默认 `data_dir` 位于 `rbnx-build`，执行清理构建会删除其中的注册数据；需要长期保留时，应把 `data_dir` 设为 `rbnx-build` 之外的持久路径。
