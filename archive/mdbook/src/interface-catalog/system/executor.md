# Executor robonix/system/executor

Executor 接 Pilot 产出的 `Plan`，按计划逐步调用各 provider 的能力（gRPC / MCP / ROS 2），把每步结果作为事件流回吐。技能（skill）的激活也由它驱动——首次调用前对技能发 `Driver(CMD_ACTIVATE)`。v0.1 里安全监督（sentinel）作为 Executor 的子模块跑（见 [系统组件](../../architecture/components.md)）。

能力约定 TOML 在 `capabilities/system/executor/`，IDL 在 `capabilities/lib/executor/` 与 `capabilities/lib/module_health/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/system/executor/execute` | `rpc_server_stream` | [`executor/Execute`](../../reference/idl.md#executor-srv-execute-srv)（`pilot/Plan` → 流 `executor/CapabilityCallEvent`） | `system/executor/execute.v1.toml` |
| `robonix/system/executor/control_plan` | `rpc` | [`executor/ControlPlan`](../../reference/idl.md#executor-srv-controlplan-srv) | `system/executor/control_plan.v1.toml` |
| `robonix/system/executor/cancel_all_plans` | `rpc` | [`executor/CancelAll`](../../reference/idl.md#executor-srv-cancelall-srv) | `system/executor/cancel_all_plans.v1.toml` |
| `robonix/system/executor/list_active_plans` | `rpc` | [`executor/ListActivePlans`](../../reference/idl.md#executor-srv-listactiveplans-srv) | `system/executor/list_active_plans.v1.toml` |
| `robonix/system/executor/get_health` | `rpc` | [`module_health/GetModuleHealth`](../../reference/idl.md#module-health-srv-getmodulehealth-srv) | `system/executor/get_health.toml` |

`Execute(plan: pilot/Plan)` 流回 `CapabilityCallEvent`——每步能力调用的开始与结果。
