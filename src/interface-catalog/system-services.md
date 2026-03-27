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

## 记忆系统服务 robonix/prm/memsearch

记忆服务（Memsearch Service）是 Robonix Agent 具备长期记忆和自我进化能力的核心组件。它基于 `memsearch[onnx]` 构建，使用本地 `milvus-lite` 向量数据库进行存储和检索。

它通过 Model Context Protocol (MCP) 将能力暴露给 Agent，注册在 `com.robonix.services.memsearch` 节点下。

### MCP 工具接口

该服务提供了三个核心的 MCP 工具供 Agent（或大模型）使用：

1. **`search_memory(query: str)`**
   - **功能**: 在长期记忆库中搜索与用户意图、偏好或历史上下文相关的记忆。
   - **使用场景**: Agent 在回答问题前，会自动进行静默召回（Silent RAG），将相关记忆注入到 System Prompt 中。大模型也可以在多轮对话中主动调用它来查阅过往决策。

2. **`save_memory(content: str)`**
   - **功能**: 将重要的事实、用户偏好或决策持久化保存到本地 Markdown 文件中，并建立向量索引。
   - **使用场景**: 当大模型识别到用户偏好（如“我最喜欢的咖啡是冰美式”），会自动触发此工具进行记忆写入。

3. **`compact_memory()`**
   - **功能**: 对近期的零散记忆进行归纳、总结和压缩。
   - **使用场景**: Agent 会在生命周期结束（如用户输入 `quit` 退出终端）时自动触发，实现自我进化和知识提炼。

### 部署与配置

记忆服务默认开启，它被集成在端到端启动脚本中：
```bash
START_MEMSEARCH=1 ./examples/run.sh
```
若在边缘设备或不需要记忆模块的场景中，可以通过设置环境变量 `START_MEMSEARCH=0` 来关闭此服务，Agent 会自动降级为无长期记忆状态。

底层数据存储在 `rust/examples/packages/memsearch_service/agent_milvus.db`（向量库）和 `agent_memory/` 目录下。

## 扩展系统服务

在 `robonix/sys/` 下增加新服务时，先在 `rust/robonix-interfaces/lib/` 中添加对应的 `.srv` 文件，然后运行 `ridlc` 生成 proto，最后将新的 `abstract_interface_id` 添加到 `robonix-server` 的 `ROBO_SYSTEM_INTERFACE_CATALOG` 中。

当前预留但尚未实现的命名空间路径：

| 路径前缀 | 用途 |
|---------|------|
| `robonix/sys/map/` | 语义地图查询、地图管理 |
| `robonix/sys/planning/` | 任务或运动规划 |
| `robonix/sys/manager/` | 技能库、任务管理器、模型管理器 |
| `robonix/sys/debug/` | 调试和检查工具 |
