---
title: 规划器
---
<span id="pilot-robonixsystempilot"></span>
# 规划器（Pilot）

规划器是规划和决策核心：接收任务，结合记忆与大语言模型产出方案，再交给执行器执行。任务入口 `robonix/system/pilot`（`SubmitTask`）通常由[交互服务](liaison.md)转发用户任务；`get_health` 返回规划器模块自身的健康报告。

能力约定 TOML 在 `capabilities/system/pilot.v1.toml` 与 `capabilities/system/pilot/`，IDL 在 `capabilities/lib/pilot/` 与 `capabilities/lib/module_health/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/system/pilot` | `rpc_server_stream` | [`pilot/SubmitTask`](../../reference/idl.md#pilot-srv-submittask-srv)（`pilot/Task` → 流 `pilot/PilotEvent`） | `system/pilot.v1.toml` |
| `robonix/system/pilot/get_health` | `rpc` | [`module_health/GetModuleHealth`](../../reference/idl.md#module-health-srv-getmodulehealth-srv) | `system/pilot/get_health.toml` |

`SubmitTask(task: pilot/Task)` 返回一条 `PilotEvent` 流。事件类型包括自然语言增量、方案、批次结果、状态、最终回复、节点状态和任务状态；该接口不暴露模型的内部思维链。用户身份放在 `Task.context_json.user_id` 中。

内置规划器将上表 2 条约定以 gRPC 注册。运行时必须配置 OpenAI 兼容视觉语言模型（VLM）的 `upstream`、`api_key` 和 `model`；`api_format` 当前只支持 `openai`。启动时可选加载 Soma YAML/URDF；每轮规划前刷新 Soma `get_health`，并尝试经 Executor 调用 Scene `get_robot_context`。当前不会自动查询 Vitals。这些上下文和长期记忆均可降级；Executor 则是执行生成方案所需的运行依赖。
