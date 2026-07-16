# Pilot robonix/system/pilot

Pilot 是规划/决策核心：接任务，结合记忆 + LLM 产出方案，交给 Executor 执行。任务入口 `robonix/system/pilot`（`SubmitTask`）通常由 [Liaison](liaison.md) 转发用户任务进来；`get_health` 返回 Pilot 模块自身的健康报告。

能力约定 TOML 在 `capabilities/system/pilot.v1.toml` 与 `capabilities/system/pilot/`，IDL 在 `capabilities/lib/pilot/` 与 `capabilities/lib/module_health/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/system/pilot` | `rpc_server_stream` | [`pilot/SubmitTask`](../../reference/idl.md#pilot-srv-submittask-srv)（`pilot/Task` → 流 `pilot/PilotEvent`） | `system/pilot.v1.toml` |
| `robonix/system/pilot/get_health` | `rpc` | [`module_health/GetModuleHealth`](../../reference/idl.md#module-health-srv-getmodulehealth-srv) | `system/pilot/get_health.toml` |

`SubmitTask(task: pilot/Task)` 返回一条 `PilotEvent` 流——规划过程的增量事件（思考、能力调用、结果、最终回答）。用户身份走 `Task.context_json.user_id`，不额外加必填字段。
