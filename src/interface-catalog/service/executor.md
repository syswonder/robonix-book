# Executor 服务

工具调用下发与结果回传（面向 Pilot）。控制面 **`contract_id = robonix/sys/runtime/executor`**。

## 契约（与 `rust/contracts/sys/executor.v1.toml` 同步）

| 项 | 值 |
|----|-----|
| **契约 ID（`contract_id`）** | `robonix/sys/runtime/executor` |
| **版本** | `1` |
| **`kind`** | `service` |
| **`[io].input`** | `pilot/msg/TaskGraph`（语义为行为图；v1 为线性 `TaskCall[]`，BT/RTDL 见 IDL 注释 TODO） |
| **`[io].output`** | `executor/msg/TaskCallEvent` |
| **`[mode].type`** | `stream_out` |
| **`[semantics]`** | `stateless = true` |

## 具体 gRPC（robonix-codegen / `lib/executor`）

**`ExecutorService.Execute(Execute_Request { graph })`** → 流式 **`TaskCallEvent`**。TOML 注明 **`ListTools`** 等同服务上的其它 RPC，**不**占用本契约 ID。

## 实现与注册

- 二进制示例：**`robonix-executor`**。
- 注册时 **`declare_interface_full(..., "robonix/sys/runtime/executor")`**，接口叶子名通常为 `executor`。
