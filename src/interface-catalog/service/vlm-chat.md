# VLM Chat 服务

多模态对话（含工具调用）能力。Pilot 通过 `VlmService.ChatStream` 驱动 ReAct 推理，支持图像输入。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/sys/model/vlm/chat` | `rpc_server_stream` | `vlm/ChatStream_Request` → stream `vlm/ChatStreamEvent` | `sys/vlm_chat.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。模型名、token 上限、区域等为部署元数据，不在契约内。同包 `VlmService` 上还保留 `Chat`（一元）、`Describe` 等 RPC，均不另立契约 ID。

## 实现

- **参考实现**：`rust/examples/packages/vlm_service/vlm_service/service.py`（OpenAI 兼容后端）
- **注册**：`contract_id="robonix/sys/model/vlm/chat"`，接口叶子名常为 `chat`
