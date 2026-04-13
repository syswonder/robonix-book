# Executor 服务

负责工具调用的下发与结果回传，供 Pilot 在 ReAct 循环中调度 MCP / gRPC / builtin 工具。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/srv/executor` | `rpc_server_stream` | `executor/Execute_Request` → stream `executor/TaskCallEvent` | `sys/executor.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。`ListTools` 等同服务上的其他 RPC 不占用本契约 ID。`stateless = true`。

## Pilot → Executor 的语义边界

Pilot 与 Executor 之间传递的并非自然语言，而是确定性的结构化 RTDL / TaskGraph。即使 VLM 生成的是 `tool_calls`，经 Pilot 打包后进入 Executor 的已是无歧义的执行图。

Executor 不做推理，仅负责工具调用与 Skill RTDL 调用（Skill Engine）的分发。异常通过上报通道回流至 Pilot，由 Pilot 决定是否重新引入 VLM 介入。该边界体现的是推理与执行的分层：推理归 Pilot / VLM，确定性执行归 Executor。

Executor 执行的工具分为四类：

| 类型 | 来源 | 发现方式 |
|------|------|---------|
| Built-in Tools | Executor 进程内（read_file、list_dir、exec 等） | 内置，不经 Atlas 注册 |
| MCP Tools | Skill Node / Service Node 暴露的 MCP 工具 | Atlas `QueryNodes` + `NegotiateChannel` |
| gRPC Tools | Primitive / Service 的 gRPC 接口 | Atlas `QueryNodes` + `NegotiateChannel` |
| Skill RTDL | 结构化技能图调用（规划中） | Atlas Skill Library |

## 实现

- 二进制：`robonix-executor`（`rust/crates/robonix-executor`）
- 注册：`declare_interface_full(..., "robonix/srv/executor")`，接口叶子名通常为 `executor`
