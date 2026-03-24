# RFC 004: Robonix MCP 集成设计

| 版本 | 日期 |
|------|------|
| 0.1 | 2026-03-24 |

## 1. 目标与范围

`robonix-server` 在提供控制平面 gRPC（或等价）API 的同时，并行暴露 MCP（Model Context Protocol）服务端点，使 LLM 客户端与 `robonix-agent` 能以标准协议发现与调用技能。

本 RFC 约定：MCP 与 gRPC 的职责划分、SKILL 到 MCP tool 的映射原则、与 RFC003/RFC005 的衔接。

不在本 RFC 内：MCP 传输层（stdio/SSE）的具体部署拓扑、鉴权实现细节、各 MCP 客户端 SDK 用法。


## 2. MCP 与 gRPC 的关系

| 接口 | 面向对象 | 职责 |
|------|----------|------|
| MCP | LLM / Agent / MCP 宿主 | 工具目录、调用约定、与对话上下文的集成；面向智能体编排。 |
| gRPC（控制面 API） | 系统组件、SDK、运维工具 | 节点注册、接口声明、channel 协商、技能元数据查询等；面向系统集成。 |

二者共享同一注册与元数据源（节点、SKILL.md），避免重复维护。Agent 优先经 MCP 消费可调用的工具视图；批量查询、非 MCP 场景仍可用 gRPC（如 RFC003 中的 `QueryAllSkills`）。


## 3. SKILL.md → MCP Tool

- 凡在 `RegisterNode`（或等价流程）中登记、且附带有效 SKILL.md 的节点，由 `robonix-server` 自动生成对应的 MCP tool 条目。
- `inputSchema`：从 SKILL.md 的 Parameters 段提取；格式为 JSON Schema，与 MCP 对 tool 参数 schema 的要求对齐（细则见 RFC005）。
- Tool 的名称、描述等展示字段从 SKILL.md 标准字段派生，保证 LLM 可理解、可稳定引用。


## 4. Agent（robonix-agent）行为

- 发现：通过 MCP 的 `tools/list`（或协议等价能力）获取当前可用 tool 集合。
- 调用：通过 MCP 的 `tools/call` 发起调用；服务端将调用路由到已注册节点所暴露的执行路径（具体路由与传输由实现绑定，本 RFC 不规定单一数据面形态）。

Agent 不替代控制面：生命周期、channel 协商仍以 gRPC 控制 API 为准；MCP 侧重可被模型调用的工具面。


## 5. 设计原则

1. 单一事实来源：SKILL 内容以节点注册时提交为准；MCP tool 为派生视图，不单独手写维护第二套 schema。
2. 协议边界清晰：MCP 不负责分配 ROS topic / shm key 等；此类仍走 `NegotiateChannel` 等控制面 API（RFC003、RFC006）。
3. 版本与兼容：MCP 能力集与 tool 列表随注册表变化；客户端应容忍 tool 增减并做好错误处理。


## 6. 与相关 RFC 的关系

| RFC | 关系 |
|-----|------|
| RFC003 | 控制平面总览；`RegisterNode`、`QueryAllSkills` 与本文 MCP 端点并列。 |
| RFC005 | SKILL.md 字段与 JSON Schema 规则，直接约束 `inputSchema` 生成。 |
| RFC006 | 多传输协商；MCP 调用后的实际数据路径可能依赖协商结果。 |


## 7. 小结

MCP = 面向 LLM/Agent 的工具协议面；gRPC = 面向系统/SDK 的控制面。`robonix-server` 统一注册 SKILL，自动映射 MCP tool，使 agent 发现与调用与系统集成路径一致。
