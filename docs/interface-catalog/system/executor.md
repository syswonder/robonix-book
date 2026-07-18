---
title: 执行器
---
<span id="executor-robonixsystemexecutor"></span>
# 执行器（Executor）

执行器接收规划器产出的执行方案，按照机器人任务描述语言（Robot Task Description Language，RTDL）的 `sequence`、`parallel` 和 `do` 结构调度，并以事件流返回执行结果。当前外部 `do` 调用只走模型上下文协议（Model Context Protocol，MCP）；目标为执行器自身时执行进程内置工具。gRPC 用于下表的方案控制面和技能的 `Driver(CMD_ACTIVATE)` 激活，执行器不直接读写 ROS 2 话题。安全监督器（Sentinel）尚未实现，能力分发链路当前不会经过其策略检查，详见[系统组件](../../architecture/components.md)。

能力约定 TOML 在 `capabilities/system/executor/`，接口定义语言（Interface Definition Language，IDL）文件在 `capabilities/lib/executor/` 与 `capabilities/lib/module_health/`。

## 接口

| 能力约定 ID | 模式 | 当前实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/system/executor/execute` | `rpc_server_stream` | gRPC | [`executor/Execute`](../../reference/idl.md#executor-srv-execute-srv)（`pilot/Plan` → 流 `executor/RtdlEvent`） | `system/executor/execute.v1.toml` |
| `robonix/system/executor/control_plan` | `rpc` | gRPC | [`executor/ControlPlan`](../../reference/idl.md#executor-srv-controlplan-srv) | `system/executor/control_plan.v1.toml` |
| `robonix/system/executor/cancel_all_plans` | `rpc` | gRPC | [`executor/CancelAll`](../../reference/idl.md#executor-srv-cancelall-srv) | `system/executor/cancel_all_plans.v1.toml` |
| `robonix/system/executor/list_active_plans` | `rpc` | gRPC | [`executor/ListActivePlans`](../../reference/idl.md#executor-srv-listactiveplans-srv) | `system/executor/list_active_plans.v1.toml` |
| `robonix/system/executor/get_health` | `rpc` | gRPC | [`module_health/GetModuleHealth`](../../reference/idl.md#module-health-srv-getmodulehealth-srv) | `system/executor/get_health.toml` |

上表 5 条标准约定由内置执行器以 gRPC 注册。它们与下面的运行时 MCP 内置工具不是同一接口面。

`Execute(plan: pilot/Plan)` 流回 `RtdlEvent`：`plan_started` 表示方案开始，`node_state` 携带节点状态与叶子调用结果，`plan_complete` 给出方案终态。

## 运行时内置工具

执行器还会在 Atlas 中注册 10 条 `robonix/system/executor/builtin/*` MCP 能力：`read_file`、`write_file`、`patch_file`、`list_dir`、`run_command`、`cancel_plan`、`get_all_plans`、`get_plan_status`、`stop_plan_at` 和 `read_capability_doc`。它们在 Atlas 中声明为 MCP 传输，但目标提供方正是 Executor 自身时，分发器会改走进程内实现，不连接其 `internal://...` 记账端点。这些能力没有标准 TOML；它们是当前内置实现的运行时表面，并会进入规划器的可调用目录。

文件与命令工具以 `ROBONIX_WORKSPACE` 为边界；未设置时使用执行器的当前工作目录。部署方应把该目录和执行器进程权限视为模型可操作的安全边界。

## 方案控制

`control_plan` 是执行器控制面，不会创建新的 RTDL 方案。请求字段如下：

| 字段 | 含义 |
|---|---|
| `action` | `cancel`、`cancel_all` 或 `stop_at` |
| `plan_id` | `cancel` 和 `stop_at` 的目标方案 ID |
| `op_id` | `stop_at` 的目标操作 ID |
| `when` | `on_enter` 表示执行该操作前停止；`on_complete` 表示该操作完成后停止。空值默认为 `on_complete` |
| `wait_ms` | `cancel` 或 `cancel_all` 等待方案退出的最长时间；`0` 使用默认值 5000 ms |

响应中的 `success` 表示请求是否合法并已被接受。对 `cancel` / `cancel_all`，`completed` 表示目标方案是否在等待期限内离开活动表；对 `stop_at`，实现会在成功设置停止点后直接返回 `completed=true`，并不表示目标方案已经结束。取消是尽力而为：尚未执行的顺序节点会被跳过，正在运行且定义了取消接口的异步调用会收到取消请求，同步调用可能自然返回后才结束。

`cancel_all_plans` 是保留的无参数兼容接口，只返回是否成功；新控制路径应使用 `control_plan(action="cancel_all")`，以获得等待结果和说明文本。

## 查询活动方案

`list_active_plans` 返回 `plans_json`。其顶层结构为：

```json
{
  "count": 1,
  "plans": [
    {
      "plan_id": "...",
      "description": "...",
      "op_count": 2,
      "cancelled": false,
      "stop_points": 0,
      "ops": [
        {
          "op_id": "1",
          "kind": "do",
          "description": "...",
          "provider_id": "mapping",
          "contract_id": "robonix/service/map/save_map",
          "state": "running"
        }
      ]
    }
  ]
}
```

该快照来自执行器的活动方案表。方案进入终态后会从表中移除，不应把历史记录与活动方案混在一起解释。
