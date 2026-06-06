# Executor robonix/system/executor

Executor 接 Pilot 产出的 `Plan`，按计划逐步调用各 provider 的能力（gRPC / MCP / ROS 2），把每步结果作为事件流回吐。技能（skill）的激活也由它驱动——首次调用前对技能发 `Driver(CMD_ACTIVATE)`。v0.1 里安全监督（sentinel）作为 Executor 的子模块跑（见 [系统组件](../../architecture/components.md)）。

契约 TOML：`capabilities/system/executor.v1.toml`，IDL 在 `capabilities/lib/executor/`。

## 接口

| 契约 ID | 模式 | 载荷（IDL） | 契约 TOML |
|---|---|---|---|
| `robonix/system/executor` | `rpc_server_stream` | `executor/Execute`（`pilot/Plan` → 流 `executor/CapabilityCallEvent`） | `system/executor.v1.toml` |

`Execute(plan: pilot/Plan)` 流回 `CapabilityCallEvent`——每步能力调用的开始与结果。
