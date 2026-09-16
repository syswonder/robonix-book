---
title: 结果验证
---

# 结果验证

结果验证能力约定定义 verifier 与 Executor 之间的公共接口。任何自定义 verifier 都可以作为能力提供方实现该能力约定；Scene Verifier 和 VLM Verifier 是当前已有的两个实现。能力约定设置了 `llm_callable=false`，因此 Pilot 不会把它作为普通动作提供给规划模型。使用、开发和执行原理见[使用和开发结果验证](../../integration-guide/result-verification.md)。

能力约定 TOML 位于 `capabilities/service/verifier/verify.v1.toml`，接口定义语言（Interface Definition Language，IDL）位于 `capabilities/lib/verifier/srv/Verify.srv`。

## 接口

| 能力约定 ID | 模式 | 当前实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/verifier/verify` | `rpc` | 模型上下文协议（Model Context Protocol，MCP） | [`verifier/Verify`](../../reference/idl.md#verifier-srv-verify-srv) | `service/verifier/verify.v1.toml` |

请求和响应字段如下：

```text
string call_id
string args_json
---
bool passed
string detail
```

`call_id` 是被验证调用的原始 ID。`args_json` 是序列化后的 JSON 对象，携带 Executor 已知的调用上下文和规则参数。`passed` 表示独立证据是否确认预期结果，`detail` 给出简短原因；verifier 应始终返回非空说明。

## Executor 传递的上下文

普通部署不需要手工构造 `args_json`。实现新的 verifier 时，应按下面的公共结构读取它：

```json
{
  "target_provider_id": "pick",
  "target_contract_id": "robonix/skill/pick/pick",
  "target_description": "拿起红色水瓶",
  "target_args": {"object_name": "红色水瓶"},
  "target_output": {"success": true},
  "verifier_args": {"camera_provider_id": "wrist_camera"}
}
```

| 字段 | 含义 |
|---|---|
| `target_provider_id` | 实际执行目标能力的提供方 ID |
| `target_contract_id` | 已完成调用的能力约定 ID |
| `target_description` | RTDL `do` 节点对预期结果的描述 |
| `target_args` | 原始调用参数；有效 JSON 保留为 JSON 值，否则保留为字符串 |
| `target_output` | 原始成功输出；有效 JSON 保留为 JSON 值，否则保留为字符串 |
| `verifier_args` | 部署规则中为当前 verifier 配置的对象 |

verifier 应把目标输出当作上下文，而不是结果成立的独立证据。它可以根据自己的证据来源定义 `verifier_args` 字段，但应在软件包文档中说明字段、默认值和约束。

## 返回与错误语义

verifier 返回 `passed=true` 时，Executor 原样保留目标能力的成功输出；返回 `passed=false` 时，Executor 将节点改为失败，并使用 `detail` 解释原因。连接失败、超时、能力调用失败或响应不是所需 JSON 结构时，Executor 会按失败关闭（fail-closed）语义把验证标记为不可用并使节点失败。请求无效或证据来源不可用时，verifier 应返回调用错误，不应返回 `passed=true`。

verifier 自身不应再次请求验证同一个结果。Executor 对 `robonix/service/verifier/verify` 的内部调用不会递归套用部署中的验证规则。
