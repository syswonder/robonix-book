# 能力目录（Atlas）


Atlas 是 Robonix 的能力提供方（Provider）注册、能力发现和通道记账服务。它保存“谁提供什么能力、当前处于什么状态、应通过哪种传输连接”等控制面信息，但不代理 gRPC、ROS 2 或 MCP 的数据流量。

本页侧重 Atlas 控制面。能力约定模式、三种传输的数据路径和跨组件调用顺序见[运行时通信](runtime-communication.md)。

## 提供方与能力

Atlas 只定义三种提供方类型：

下表是提供方分类的概念摘录，不是可调用 API 或部署清单。

| 类型 | 典型职责 |
|---|---|
| 原语（Primitive） | 对硬件或设备数据提供标准原语接口 |
| 服务（Service） | 提供可复用算法、系统服务或常驻能力 |
| 技能（Skill） | 提供可被智能体调用的任务级能力 |

`robonix/system/*` 是能力约定的命名空间，不是第四种提供方类型。Pilot、Scene 等系统组件在 Atlas 的数据模型中仍使用上述类型之一。

一个提供方注册一次，然后可以声明多条能力。核心字段包括：

- `id`：Atlas 内全局唯一的提供方 ID。对 `rbnx` 管理的原语、服务和技能软件包，必须与部署项的 `name` 一致；非内置系统软件包使用 `system:` 下的键名。
- `kind`：原语、服务或技能。
- `namespace`：主命名空间，用于检索和诊断，不是授权边界。
- `state`：提供方生命周期状态。
- `capability_md`：提供方注册时上传的 `CAPABILITY.md` 内容。
- `capabilities`：该提供方声明的能力约定与传输绑定。

同一个能力约定可以由多个提供方实现。例如，四个相机都可以声明 `robonix/primitive/camera/rgb`。Atlas 的运行时唯一键是：

```text
(provider_id, contract_id, transport)
```

因此，能力约定 ID 表示标准接口，不表示某个具体设备。

## 注册与声明

提供方的注册分为两步：

1. 调用 `RegisterPrimitive`、`RegisterService` 或 `RegisterSkill`，登记提供方身份和主命名空间。
2. 每种 `(contract, transport)` 调用一次 `DeclareCapability`，提交传输参数和候选访问地址。

注册成功不等于可以调用。带生命周期驱动能力约定的提供方还要完成：

```text
REGISTERED -> Driver(CMD_INIT) -> INACTIVE
INACTIVE   -> Driver(CMD_ACTIVATE) -> ACTIVE
```

消费者应检查提供方是否为 `ACTIVE`。Atlas 会记录状态，但 `ConnectCapability` 本身不会因提供方非 `ACTIVE` 而拒绝连接。技能通常在初始化后保持 `INACTIVE`，由 Executor 在首次调用时激活。

## 三种传输

下表是 Atlas 注册信息中的传输绑定摘录；完整接口形态与选择原则见[运行时通信](runtime-communication.md#接口模式与传输方式)。

| 传输方式 | 绑定参数 | 访问地址示例 | 典型用途 |
|---|---|---|---|
| gRPC | `proto_file`、`service_name`、`method` | `127.0.0.1:50105` | 生命周期、RPC、流式二进制接口 |
| ROS 2 | `qos_profile` | `/camera/color/image_raw` | 话题与 ROS 2 数据平面 |
| MCP | `input_schema_json` | `http://127.0.0.1:50106/mcp` | Pilot 可发现的工具调用 |

同一提供方可以用不同传输暴露不同能力约定，也可以用多个传输暴露同一能力约定。

Atlas 会检查 `(transport, endpoint)` 冲突。ROS 2 访问地址可由 Atlas 生成或在冲突时改名；gRPC 和 MCP 访问地址发生冲突时，提供方必须选择新地址。提供方必须使用 `DeclareCapabilityResponse.endpoint` 返回的最终值。

## 查询与建立连接

`Query` 返回提供方、生命周期、能力约定、传输方式和传输参数，但故意不返回访问地址。消费者准备调用时必须执行：

```text
ConnectCapability(consumer_id, provider_id, contract_id, transport)
  -> channel_id + endpoint + transport params
```

Atlas 记录这条消费者到提供方的边，并把可连接的访问地址返回给消费者。随后消费者直接建立 gRPC、ROS 2 或 MCP 连接；数据不会经过 Atlas。

使用结束后调用 `DisconnectCapability(channel_id)`。该操作是幂等的。提供方注销、心跳超时或同 ID 接管时，Atlas 会自动清除指向旧提供方的通道。

## 命名空间诊断

普通能力约定应位于提供方声明的主命名空间下。若不一致，Atlas 仍接受该能力，但会设置 `namespace_mismatch=true` 并记录警告。能力约定描述文件显式设置 `cross_namespace = true` 时，不产生该诊断。

可使用以下命令查看：

```bash
rbnx caps -v
rbnx caps --json
rbnx channels
rbnx inspect
```

命名空间只帮助分类、检索和发现错误配置，不阻止系统启动或能力调用。

## 规划器（Pilot）如何读取能力文档

Python 提供方默认查找软件包根目录的 `CAPABILITY.md`。提供方在自己的文件系统中读取文档，并在注册时同时发送：

- `capability_md_path`：仅用于诊断；容器内路径不保证能被宿主机访问。
- `capability_md`：完整 Markdown 内容；这是跨主机、跨容器的可移植数据。

Pilot 会把每条 MCP 能力的提供方 ID、由能力约定派生的展示名、能力实例 `description` 和 `input_schema_json` 放入当轮能力目录。对于注册了 `CAPABILITY.md` 的提供方，Pilot 还会列出提供方 ID、类型和文档前置元数据中的简短 `description`。当需要完整说明时，模型调用 Executor 的内置操作 `read_capability_doc`，并传入 `provider_id`。该操作从 Atlas 读取已注册的文档内容。

因此，能力文档流程中不要让模型猜文件路径，也不要用 `read_file` 读取提供方的 `CAPABILITY.md`。

## 心跳与生命周期

Python 提供方框架默认每 30 秒发送一次心跳；Executor、Pilot、Soma、Vitals 和 Liaison 等内置组件当前每 20 秒发送一次。当前 Atlas 默认参数为：

| 参数 | 默认值 | 环境变量 |
|---|---:|---|
| 标记为 `TERMINATED` 的超时 | 90 秒 | `ROBONIX_ATLAS_HEARTBEAT_TIMEOUT_MS` |
| `TERMINATED` 后删除记录的延迟 | 10 分钟 | `ROBONIX_ATLAS_GC_AFTER_TERMINATED_MS` |
| 驱逐扫描间隔 | 10 秒 | `ROBONIX_ATLAS_EVICTION_INTERVAL_MS` |

同类型、同 ID 的重新注册采用接管语义：旧能力和相关通道会被移除，新提供方必须重新声明能力。不同类型使用同一个 ID 会返回 `AlreadyExists`。

## 主要 RPC

下表仅列出最常用的 Atlas RPC **接口摘录**。请求/响应字段、枚举和完整服务定义以 [`system/atlas/proto/atlas.proto`](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/system/atlas/proto/atlas.proto) 为准。

| RPC | 作用 |
|---|---|
| `RegisterPrimitive` / `RegisterService` / `RegisterSkill` | 注册提供方身份 |
| `DeclareCapability` | 声明一条能力约定与传输绑定 |
| `Heartbeat` / `Unregister` | 维持或结束注册 |
| `SetLifecycleState` | 上报提供方生命周期状态 |
| `Query` | 按提供方 ID、命名空间、类型、能力约定和传输方式发现能力 |
| `ConnectCapability` / `DisconnectCapability` | 获取访问地址并记录或释放通道 |
| `QueryContract` / `ListContracts` | 查询标准能力约定描述文件 |
| `InspectAtlas` | 获取提供方、能力和通道的调试快照 |

未显式声明 Driver 时，运行时为提供方选择共享的 `robonix/lifecycle/driver`；清单也可以显式写出该约定。已有软件包可显式保留一条 `<provider-namespace>/driver`。每个提供方始终有且只有一条 Driver；Atlas 把两种约定都识别为 Driver，并会拒绝同一提供方同时声明两条生命周期入口。

Atlas 的线协议定义位于 `system/atlas/proto/atlas.proto`。标准能力约定描述文件位于 Robonix 源码树的 `capabilities/`；它们如何映射到 ROS 2、gRPC 与 MCP 见[运行时通信](runtime-communication.md)。
