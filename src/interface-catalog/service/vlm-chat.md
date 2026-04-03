# VLM Chat 服务

多模态对话（含工具调用）能力，契约 ID **`robonix/sys/model/vlm/chat`**。

## 契约（与 `rust/contracts/sys/vlm_chat.v1.toml` 同步）

| 项 | 值 |
|----|-----|
| **契约 ID（`contract_id`）** | `robonix/sys/model/vlm/chat` |
| **版本** | `1` |
| **`kind`** | `service` |
| **`[io].input`** | `vlm/srv/ChatStream`（请求侧为 `ChatStream_Request`） |
| **`[io].output`** | `vlm/msg/ChatStreamEvent`（流元素） |
| **`[mode].type`** | `stream_out` |
| **`[semantics]`** | `multimodal = true`，`tool_use = true` |

TOML 注释：模型名、token 上限、区域等由部署元数据承载，**不在**本契约内。

## 具体 gRPC（robonix-codegen / `lib/vlm`）

与契约对齐的主路径：**`VlmService.ChatStream`** → `stream ChatStreamEvent`（见 **`robonix_proto/vlm.proto`**）。

同一 **`VlmService`** 上通常还有（**未单独拆契约 TOML**）：

| RPC | 说明 |
|-----|------|
| **`Chat`** | 一元对话（`lib/vlm/srv/Chat.srv`） |
| **`Describe`** | 图像描述（`lib/vlm/srv/Describe.srv`） |

参考实现：**`rust/examples/packages/vlm_service/vlm_service/service.py`**（OpenAI 兼容后端）。

## 注册

`DeclareInterface` 使用 **`contract_id="robonix/sys/model/vlm/chat"`**，接口叶子名常为 `chat`。
