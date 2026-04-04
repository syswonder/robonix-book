# VLM Chat 服务

多模态对话（含工具调用）能力，契约 ID **`robonix/sys/model/vlm/chat`**。

## 契约（与 `rust/contracts/sys/vlm_chat.v1.toml` 同步）

| 项 | 值 |
|----|-----|
| **契约 ID（`contract_id`）** | `robonix/sys/model/vlm/chat` |
| **版本** | `1` |
| **`kind`** | `service` |
| **`[io.srv]`** | `srv = "vlm/srv/ChatStream"`（request：`ChatStream_Request`；response 单字段 → 流元素 **`ChatStreamEvent`**） |
| **`[mode].type`** | `rpc_server_stream` |
| **`[semantics]`** | `multimodal = true`，`tool_use = true` |

TOML 注释：模型名、token 上限、区域等由部署元数据承载，**不在**本契约内。

## 具体 gRPC（robonix-codegen / `lib/vlm`）

- 契约门面：`robonix_contracts.proto` 中由 **`vlm_chat.v1.toml`** 的 **`[mode].type = "rpc_server_stream"`** 与 **`lib/vlm/srv/ChatStream.srv`**（**response** 段单一字段 **`vlm/ChatStreamEvent`**）定义 **`Stream(…Request) returns (stream ChatStreamEvent)`**。
- 同包 **`VlmService`** 上还可保留 **`ChatStream`**、**`Chat`**、**`Describe`** 等由 IDL 生成的 RPC；与 **`robonix/sys/model/vlm/chat`** 契约 id 对齐的是 codegen 生成的 **`VlmChat`** 服务名（见 `robonix_contracts.proto`）。

参考实现：**`rust/examples/packages/vlm_service/vlm_service/service.py`**（OpenAI 兼容后端）。

## 注册

`DeclareInterface` 使用 **`contract_id="robonix/sys/model/vlm/chat"`**，接口叶子名常为 `chat`。
