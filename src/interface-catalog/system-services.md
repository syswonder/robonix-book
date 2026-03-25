# 系统服务 robonix/sys

系统服务注册在 `robonix/sys/` 命名空间下，与硬件原语（`prm`）分开，方便 Agent 通过 namespace 前缀区分硬件和软件能力。

## VLM 服务 robonix/sys/model/vlm

| abstract_interface_id | 模式 | IDL | gRPC 映射 |
|-----------------------|------|-----|-----------|
| `robonix/sys/model/vlm/chat` | RPC | `lib/vlm/srv/Chat.srv` | `VlmService/Chat` |
| `robonix/sys/model/vlm/describe` | RPC | `lib/vlm/srv/Describe.srv` | `VlmService/Describe` |

`Chat` 是主要接口，对应 OpenAI chat completions 的语义。请求中包含消息序列（含可选 image_base64）和工具定义，返回文本和/或 tool_calls。`robonix-agent` 的 ReAct 循环通过这个接口与 VLM 交互。

`Describe` 是一个简化的图像描述接口，接受一张图片和一个 prompt，返回文本描述。

参考实现是 `rust/examples/packages/vlm_service/vlm_service/service.py`，它通过 OpenAI Python SDK 将请求代理到任意 OpenAI 兼容后端。

## 扩展系统服务

在 `robonix/sys/` 下增加新服务时，先在 `rust/robonix-interfaces/lib/` 中添加对应的 `.srv` 文件，然后运行 `ridlc` 生成 proto，最后将新的 `abstract_interface_id` 添加到 `robonix-server` 的 `ROBO_SYSTEM_INTERFACE_CATALOG` 中。

当前预留但尚未实现的命名空间路径：

| 路径前缀 | 用途 |
|---------|------|
| `robonix/sys/map/` | 语义地图查询、地图管理 |
| `robonix/sys/planning/` | 任务或运动规划 |
| `robonix/sys/manager/` | 技能库、任务管理器、模型管理器 |
| `robonix/sys/debug/` | 调试和检查工具 |
