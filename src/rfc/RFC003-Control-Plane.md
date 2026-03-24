# RFC 003: Robonix 控制平面设计（Control Plane Design）

| 版本 | 日期 |
|------|------|
| 0.1 | 2026-03-24 |

## 1. 目标与范围

`robonix-server` 定位为纯控制平面（可视为服务网格的控制面）：集中管理谁在线、暴露什么接口、如何连上，不承担业务数据面传输，也不是通用通信框架。

本 RFC 约定控制平面提供的运行时 API 能力边界：

- 服务注册与发现（按命名空间、名称、传输类型等过滤）。
- Channel 协商：在多方同意的前提下，由系统分配具体端点标识（如 topic 名、共享内存 key 等）。

不在本 RFC 内：各传输栈的实现细节、安全认证与 ACL 的完整方案、高可用与持久化策略（仅概述职责）。


## 2. 核心概念

| 概念 | 含义 |
|------|------|
| Node | 独立进程；启动后通过 API 注册自身，获得在控制平面中的身份与元数据。 |
| Interface | 节点声明的对外通信接口；包含接口身份及支持的传输类型列表（可多选）。 |
| Channel | 控制平面在协商后分配的一条逻辑连接描述：人类/配置可读的端点信息（如 ROS 2 topic 名、gRPC 目标地址、共享内存 key、Zenoh key expression 等），供数据面使用。 |
| Transport | 传输类型枚举，例如：`ros2`、`grpc`、`shared_memory`、`zenoh`（可扩展）。Interface 声明"能用什么"；协商时由消费者侧选定其一。 |


## 3. 运行时 API（概览）

以下为能力命名与职责摘要，非 protobuf/REST 级规范。

| API | 职责 |
|-----|------|
| `RegisterNode` | 注册节点：`node_id`、`namespace`、`kind`（节点类别/角色）、`skill_md`（关联的 SKILL.md 内容或引用，供发现与 agent 使用）。 |
| `DeclareInterface` | 在已注册节点上声明接口：接口身份及支持的 Transport 列表。 |
| `QueryNodes` | 发现服务：按命名空间、名称、传输类型等条件过滤，返回匹配的节点与接口元数据。 |
| `NegotiateChannel` | 通道协商：消费者指定目标接口与所选 Transport；控制平面校验兼容性并分配 Channel（端点元数据）。 |
| `QueryAllSkills` | 聚合返回系统中已注册的 SKILL.md（或等价摘要），供 agent 构建系统 prompt 或工具目录。 |


## 4. 设计原则

1. 不抽象通信语义
   控制平面不在数据面强加任何通信模式；具体的通信模式属于应用层自行约定，而非 `robonix-server` 的职责。

2. 分配后即用原生栈
   `NegotiateChannel` 返回的是元数据与端点信息。之后用户直接使用对应生态 API：ROS 2 用 `rclpy` / `rclcpp`，gRPC 用原生 channel，共享内存用平台 API，Dora 用其运行时接口等。

3. 不触碰业务数据
   Robonix 控制平面只管理元数据（注册、发现、协商结果），不路由、不缓存、不解析业务消息载荷。


## 5. MCP 集成

`robonix-server` 在提供上述控制平面 API 的同时，可暴露 MCP（Model Context Protocol）服务端点：

- 将已注册节点关联的 SKILL.md 自动映射为 MCP tool 定义（名称、描述、参数 schema 等由约定规则从 SKILL.md 派生或配置）。
- Agent 可通过 MCP 发现与调用这些工具，与 `QueryAllSkills` 形成互补：前者面向 MCP 客户端协议，后者面向批量拉取与 prompt 拼装。

具体映射规则与 MCP 能力列表版本化属实现细节，本 RFC 仅固定职责：SKILL 注册源与控制平面一致，避免重复维护。


## 6. 与相关文档的关系

| 文档 | 关系 |
|------|------|
| RFC001（RIDL） | 接口契约与类型；控制平面可引用接口身份，但不实现 RIDL 语义。 |
| RFC002（Package） | Package / Node / entry 与构建启停；运行时注册将进程与 manifest 中的逻辑节点对齐。 |


## 7. 小结

robonix-server = 元数据 + 协商；数据面 = 用户选定的 Transport + 原生 API。在此边界下，系统获得统一发现与 channel 分配，同时保持各传输栈的惯用编程模型与性能特征。
