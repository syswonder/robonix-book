# Cognition · Reason（认知层推理角色）

认知层推理角色的参考实现。Pilot 通过 `CognitionService.Reason`（即 `VlmService.ChatStream`）驱动 ReAct 推理循环，支持多模态输入（文本、图像）与工具调用。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/srv/cognition/reason` | `rpc_server_stream` | `vlm/ChatStream_Request` → stream `vlm/ChatStreamEvent` | `sys/vlm_chat.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。模型名、token 上限、区域等为部署元数据，不在契约内。同包 `VlmService` 上还保留 `Chat`（一元）、`Describe` 等 RPC，均不另立契约 ID。

## 多角色认知层

Robonix 对认知层的长期设想为多模型分工：

- 推理模型（reason）：接收感知数据与用户任务，进行 CoT 推理与逻辑分析，即本契约。
- 世界模型（world，`robonix/srv/cognition/world`，规划中）：预测环境状态变化，辅助规划。
- 代码模型（code，`robonix/srv/cognition/code`，规划中）：生成结构化的 RTDL / TaskGraph 执行计划。

当前 `vlm_service`（OpenAI 兼容后端）承担 reason 角色的全部推理工作，为最简单的单模型配置。后续按需拆出独立的 world 与 code 契约。

## 实现

- 参考实现：`rust/examples/packages/vlm_service/vlm_service/service.py`（OpenAI 兼容后端）
- 注册：`contract_id="robonix/srv/cognition/reason"`，接口叶子名常为 `chat`

本契约目前为认知层唯一实现，未来将与 `cognition/world`、`cognition/code` 等契约并列。
