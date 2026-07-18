# 声纹 robonix/service/voiceprint

声纹服务做说话人识别：先给每个用户做一遍 enrollment（采几秒说话样本算 embedding），识别阶段拿新音频算 embedding 做相似度匹配。[Liaison](../system/liaison.md) 语音会话用它把音频归到具体用户，驱动访问控制。

能力约定 TOML 在 `capabilities/service/voiceprint/`，IDL 在 `capabilities/lib/voiceprint/`。该服务没有 `driver` 能力约定。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/service/voiceprint/enroll` | `rpc` | [`voiceprint/Enroll`](../../reference/idl.md#voiceprint-srv-enroll-srv) | `service/voiceprint/enroll.v1.toml` |
| `robonix/service/voiceprint/identify` | `rpc` | [`voiceprint/Identify`](../../reference/idl.md#voiceprint-srv-identify-srv) | `service/voiceprint/identify.v1.toml` |
| `robonix/service/voiceprint/list` | `rpc` | [`voiceprint/ListEnrolled`](../../reference/idl.md#voiceprint-srv-listenrolled-srv) | `service/voiceprint/list.v1.toml` |
| `robonix/service/voiceprint/delete` | `rpc` | [`voiceprint/DeleteEnrolled`](../../reference/idl.md#voiceprint-srv-deleteenrolled-srv) | `service/voiceprint/delete.v1.toml` |

`identify(audio, encoding, sample_rate)` 返回 `(user_id, user_name, confidence, is_known)`。`enroll` 采样建档，`list` / `delete` 管理已注册用户。若部署里没起声纹 provider，Liaison 会 fallback 到客户端传来的用户 hint（见 [Liaison](../system/liaison.md)）。

参考实现：`services/voiceprint`（如 SpeechBrain ECAPA-TDNN / Resemblyzer embedding + cosine 相似度）。
