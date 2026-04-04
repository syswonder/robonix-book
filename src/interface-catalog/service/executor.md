# Executor 服务

工具调用下发与结果回传，供 Pilot 在 ReAct 循环中调度 MCP / gRPC / builtin 工具。

## 接口

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/sys/runtime/executor` | `rpc_server_stream` | `executor/Execute_Request` → stream `executor/TaskCallEvent` | `sys/executor.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。`ListTools` 等同服务上的其他 RPC 不占用本契约 ID。`stateless = true`。

## 实现

- **二进制**：`robonix-executor`（`rust/crates/robonix-executor`）
- **注册**：`declare_interface_full(..., "robonix/sys/runtime/executor")`，接口叶子名通常为 `executor`
