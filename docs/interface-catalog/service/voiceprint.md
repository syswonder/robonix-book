<span id="声纹-robonixservicevoiceprint"></span>
# 声纹

声纹服务做说话人识别：先给每个用户做一遍 enrollment（采集说话样本并计算 embedding），识别阶段再对新音频做相似度匹配。[Liaison](../system/liaison.md) 用识别结果标注用户；启用访问控制后，还会检查置信度和用户 allow-list。

能力约定 TOML 在 `capabilities/service/voiceprint/`，IDL 在 `capabilities/lib/voiceprint/`。该服务没有 `driver` 能力约定。

## 接口

| 能力约定 ID | 模式 | 默认实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/voiceprint/enroll` | `rpc` | gRPC | [`voiceprint/Enroll`](../../reference/idl.md#voiceprint-srv-enroll-srv) | `service/voiceprint/enroll.v1.toml` |
| `robonix/service/voiceprint/identify` | `rpc` | gRPC | [`voiceprint/Identify`](../../reference/idl.md#voiceprint-srv-identify-srv) | `service/voiceprint/identify.v1.toml` |
| `robonix/service/voiceprint/list` | `rpc` | gRPC + MCP | [`voiceprint/ListEnrolled`](../../reference/idl.md#voiceprint-srv-listenrolled-srv) | `service/voiceprint/list.v1.toml` |
| `robonix/service/voiceprint/delete` | `rpc` | gRPC（实现已注册，manifest 漏列） | [`voiceprint/DeleteEnrolled`](../../reference/idl.md#voiceprint-srv-deleteenrolled-srv) | `service/voiceprint/delete.v1.toml` |

`identify(audio_data, encoding, sample_rate_hz)` 返回 `user_id`、`user_name`、`confidence`、`is_known` 和 `error`。`enroll` 还需要调用方提供稳定的 `user_id` 与显示用 `user_name`；`list` 返回 JSON 用户数组和数量；`delete` 按 `user_id` 删除。

若声纹提供方不可用，交互服务可把客户端提示作为身份回退值，但它不能绕过已启用的访问控制。访问控制启用时，只有已注册、置信度达到阈值且位于允许列表中的声纹可以通过；其他语音请求都会被拒绝。

参考实现：Robonix [`8c2551ce`](https://github.com/syswonder/robonix/tree/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/voiceprint) 中的 `services/voiceprint`，使用 SpeechBrain ECAPA-TDNN embedding 与余弦相似度。`VOICEPRINT_DATA_DIR` 指定 `enrolled.json` 目录，`VOICEPRINT_THRESHOLD` 指定相似度阈值（默认 `0.25`），`VOICEPRINT_DEVICE` 选择 `cuda:0` 或 `cpu`。

> 上游已知缺陷：实现代码已注册 `delete` gRPC 服务，但 `services/voiceprint/package_manifest.yaml` 只列出 `identify`、`enroll`、`list`。运行中的提供方可以注册 `delete`，但依赖清单枚举能力的目录和校验会漏掉它；调用前应从 Atlas 确认实际注册状态。

> 实现依据：[package manifest](https://github.com/syswonder/robonix/blob/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/voiceprint/package_manifest.yaml) · [gRPC / MCP 注册](https://github.com/syswonder/robonix/blob/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/voiceprint/voiceprint_service/service.py)
