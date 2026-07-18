# 运行时通信

Robonix 用**能力约定（Contract）**描述稳定的接口语义，再由每个能力提供方（Provider）选择实际的**传输方式（Transport）**。这两层不能混为一谈：能力约定中的 `rpc`、`topic_in` 和 `topic_out` 说明调用形态，ROS 2、gRPC 和模型上下文协议（Model Context Protocol，MCP）才是运行时通信实现。

本页说明组件之间的连接关系；各组件职责与落地状态见[系统组件](components.md)，Atlas 控制面字段见[能力目录](atlas.md)。

例如，底盘速度输入的能力约定 ID 是 `robonix/primitive/chassis/twist_in`。这个 ID 不是 ROS 2 话题名；某个底盘提供方可以把它绑定到 `/cmd_vel`，另一个提供方也可以绑定到不同端点。调用方先按能力约定发现实现，再从 Atlas 取得该实例的实际端点。

## 接口模式与传输方式

能力约定 TOML 的 `[mode].type` 定义数据流方向和调用形态：

下表是当前常用模式的 **API 形态摘录**，不是全部能力约定清单。具体请求/响应字段以[生成的 IDL 参考](../reference/idl.md)和对应 TOML 为准。

| 模式 | 接口语义 | 当前常见传输 |
|---|---|---|
| `rpc` | 一次请求、一次响应 | gRPC、MCP、ROS 2 service |
| `rpc_server_stream` | 一次请求、连续响应 | gRPC |
| `rpc_client_stream` | 连续请求、一次响应 | gRPC |
| `rpc_bidirectional_stream` | 双向连续消息 | gRPC |
| `topic_out` | 提供方持续发布 | ROS 2、gRPC |
| `topic_in` | 提供方持续接收 | ROS 2、gRPC |

`rpc` 不等于 gRPC。相同的请求—响应能力可以面向确定性客户端提供 gRPC，也可以面向模型规划提供 MCP；提供方还可以为同一能力约定分别声明多个传输。Atlas 用下面的组合区分运行时能力：

```text
(provider_id, contract_id, transport)
```

当前 Python API 尚未在注册阶段完整拒绝所有不合理的“模式—传输”组合。开发者仍需通过代码生成、提供方启动和真实消费方调用，验证两端使用了相同的数据结构与传输。

## 三种传输分别承担什么

### ROS 2

ROS 2 适合机器人内部的连续数据平面，例如图像、点云、激光扫描、里程计、关节状态和速度命令。原语与服务仍然编写普通 ROS 2 发布者、订阅者、服务或动作客户端；Robonix 额外把实际端点登记到 Atlas，使消费者不必硬编码设备话题。

ROS 2 action 可以作为导航等提供方的内部实现，但当前不是 Robonix 能力约定的一种独立模式。对外仍通过 `rpc`、`topic_in` 或 `topic_out` 等能力约定提供接口。

ROS 2 概念与通信接口见 [ROS 2 官方文档](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html)。跨主机或容器的中间件配置见[多平台部署](multiplatform-deployment.md#ros-2rmw-与-zenoh)。

### gRPC

gRPC 适合确定性的跨进程控制面、请求—响应和流式接口。Robonix 当前用它承载：

- Atlas 的注册、发现和通道记账接口；
- Liaison、Pilot、Executor、Soma 等系统组件之间的控制调用；
- 每个受管提供方的生命周期 Driver；
- 语音识别、语音合成等面向程序调用的服务接口。

`rbnx boot` 或 Soma 先通过 Atlas 找到提供方的共享 `robonix/lifecycle/driver`，再直接调用 `Driver(CMD_INIT)` 和 `Driver(CMD_ACTIVATE)`。Executor 首次调用仍处于 `INACTIVE` 的技能时，也通过该 Driver 激活技能。生命周期回调可以留空；Driver 仍会存在，并以 warning 和空操作完成状态转换。

:::warning[后向兼容：已有命名空间 Driver]
已有软件包可以暂时继续使用自己维护的 `<provider-namespace>/driver` 和 Driver TOML，但不能同时注册共享 Driver。该兼容方式计划迁移，详见[软件包与部署清单规范](../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

gRPC 的请求、响应和流式模型见 [gRPC 官方介绍](https://grpc.io/docs/what-is-grpc/introduction/)。

### MCP

MCP 用于把模型可以规划和调用的能力暴露为工具，例如相机快照、场景查询、导航目标、语音播报和任务级技能。Pilot 从 Atlas 查询当前注册的 MCP 能力、描述和输入结构，据此生成机器人任务描述语言（Robot Task Description Language，RTDL）方案。发现和生成方案本身不调用这些业务工具；但在规划前的上下文准备中，Pilot 会经 Executor 读取 Scene 快照，并可读取记忆服务。

方案提交给 Executor 后，Executor 才为每个 `do` 节点连接目标提供方的 MCP 端点并调用工具。Executor 自身的一部分内置操作也登记为 MCP 能力，但由进程内分发，不会建立外部 MCP 网络连接。

MCP 的工具与传输规范见 [Model Context Protocol 规范](https://modelcontextprotocol.io/specification/2025-11-25)。

## Atlas 如何建立连接

Atlas 是能力目录和控制面，不代理业务数据。一次完整连接按以下顺序发生：

1. 提供方调用 `RegisterPrimitive`、`RegisterService` 或 `RegisterSkill` 注册身份。
2. 提供方对每个 `(contract_id, transport)` 调用 `DeclareCapability`，登记传输参数和候选端点。
3. 消费方用 `Query` 查找符合条件的提供方和能力。查询结果用于发现，不返回可直接访问的端点。
4. 消费方调用 `ConnectCapability`，Atlas 返回最终端点、传输参数和 `channel_id`，并记录消费关系。
5. 消费方直接通过 ROS 2、gRPC 或 MCP 连接提供方；图像、请求和工具结果不经过 Atlas。
6. 使用结束后调用 `DisconnectCapability`。提供方退出或被同 ID 新实例接管时，Atlas 也会清理旧通道。

因此，`ConnectCapability` 建立的是可观测的连接记录并发放端点，不是数据代理、消息总线或网络隧道。详细字段与诊断命令见 [Atlas 能力目录](atlas.md)。

## 用户任务的调用链

通过 Client 或 `rbnx chat` 提交任务时，默认链路如下：

下表是典型文本任务的组件/API 调用摘录；语音前处理、各提供方内部 SDK 调用和失败恢复分支未在表中展开。

| 阶段 | 调用 | 传输 |
|---|---|---|
| 用户交互 | Client / `rbnx chat` → Liaison | gRPC |
| 任务规划 | Liaison → Pilot `SubmitTask` | gRPC |
| 方案执行 | Pilot → Executor | gRPC |
| 模型工具 | Executor → 目标原语、服务或技能提供方 | MCP（直连提供方） |
| 技能首次激活 | Executor → 目标技能 Driver | gRPC |
| 机器人连续数据 | 原语 ↔ 服务 | 通常为 ROS 2 |

Liaison 每次通过 Atlas 解析 Pilot 的实际端点；Pilot 同样通过 Atlas 解析 Executor。`rbnx ask` 是面向调试的另一条入口，会绕过 Liaison 并直接连接 Pilot。RTDL `do` 节点一直保存完整 `contract_id`；Pilot 给模型展示的简短名、以及 Executor 在单个 MCP 服务内使用的 leaf 工具名，都不是 ROS 2 话题名，也不是 Atlas 中的能力身份。

模型可见工具并不替代机器人数据平面。以“抓取积木”为例，Pilot 和 Executor 通过 MCP 启动抓取技能；技能通过 Atlas 发现机械臂能力，再用 ROS 2 关节命令和关节状态与机械臂原语通信；该原语最后调用厂商 SDK 控制机械臂和夹爪。完整改造步骤见[抓积木接入示例](../tutorials/existing-python-feature.md)。

## 一个提供方可以同时使用多种传输

提供方按消费方和数据特性选择传输，不需要整包只选一种：

- 相机可以用 ROS 2 持续发布 RGB 和深度图，同时用 MCP 提供按需快照。
- Scene 可以通过 ROS 2 消费相机与地图数据，同时用 MCP 提供房间、对象和机器人上下文查询。
- Speech 可以用 gRPC 提供语音识别与合成缓冲区，同时用 MCP 提供模型可调用的 `speak`。
- 所有受管提供方都通过 gRPC Driver 接收生命周期命令，不影响其业务能力使用其它传输。

接口目录中的传输列描述当前参考实现，不代表能力约定永久绑定该传输。具体部署仍应以 Atlas 中实际注册的 `(provider_id, contract_id, transport)` 为准。架构图里的 `/rgb`、`/odom`、`/map` 一类短标签只能视为某次部署的端点示例，不能代替完整 `contract_id`；消费方仍须通过 `ConnectCapability` 取得该实例的最终端点。图中把某个接口标成 gRPC、ROS 2 或 MCP，也只表示画出的那条路径，不表示独占绑定：同一能力约定可以同时声明多种传输，图里没有列出的接口也不能据此判定不存在。

## 开发与排查

开发一个能力时，按下面的顺序确定通信方式：

1. 先复用或定义能力约定，确认输入、输出和 `mode`。
2. 根据实际调用方、数据频率和部署边界选择 ROS 2、gRPC 或 MCP。
3. 在提供方中实现接口，并把实际端点声明给 Atlas。
4. 在消费方中通过 Atlas 查询并连接，不硬编码另一个软件包的端点。
5. 用真实消费方完成端到端调用；不要只以“提供方进程已启动”作为验收。

运行部署后可检查：

```bash
rbnx caps -v
rbnx channels
rbnx inspect
```

`rbnx caps -v` 用于核对提供方、能力约定和传输；`rbnx channels` 用于检查已经建立的消费关系；`rbnx inspect` 提供 Atlas 的整体诊断快照。标准接口和当前参考实现见[接口目录](../interface-catalog/index.md)，Python 调用方法见[开发者指南](../developer-guide.md#14-python-接口)。
