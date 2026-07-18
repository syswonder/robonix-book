# Atlas 能力目录

[toc]

Atlas 是 Robonix 的 provider 注册、能力发现和通道记账服务。它保存“谁提供什么能力、当前处于什么状态、应通过哪种传输连接”等控制面信息，但不代理 gRPC、ROS 2 或 MCP 的数据流量。

## Provider 与 Capability

Atlas 只定义三种 provider kind：

| Kind | 典型职责 |
|---|---|
| Primitive | 对硬件或设备数据提供标准原语接口 |
| Service | 提供可复用算法、系统服务或长驻能力 |
| Skill | 提供可被 Agent 调用的任务级能力 |

`robonix/system/*` 是 contract 命名空间，不是第四种 provider kind。Pilot、Scene 等系统组件在 Atlas 的数据模型中仍使用上述 kind 之一。

一个 provider 注册一次，然后可以声明多条 Capability。核心字段包括：

- `id`：Atlas 内全局唯一的 provider ID。部署启动时必须与 `robonix_manifest.yaml` 中的实例 `name` 一致。
- `kind`：Primitive、Service 或 Skill。
- `namespace`：主命名空间，用于检索和诊断，不是授权边界。
- `state`：provider 生命周期状态。
- `capability_md`：provider 注册时上传的 `CAPABILITY.md` 内容。
- `capabilities`：该 provider 声明的 contract 与传输绑定。

同一个 contract 可以由多个 provider 实现。例如，四个相机都可以声明 `robonix/primitive/camera/rgb`。Atlas 的运行时唯一键是：

```text
(provider_id, contract_id, transport)
```

因此，contract ID 表示标准接口，不表示某个具体设备。

## 注册与声明

provider 的注册分为两步：

1. 调用 `RegisterPrimitive`、`RegisterService` 或 `RegisterSkill`，登记 provider 身份和主命名空间。
2. 每种 `(contract, transport)` 调用一次 `DeclareCapability`，提交传输参数和候选 endpoint。

注册成功不等于可以调用。带 driver contract 的 provider 还要完成：

```text
REGISTERED -> Driver(CMD_INIT) -> INACTIVE
INACTIVE   -> Driver(CMD_ACTIVATE) -> ACTIVE
```

消费者必须检查 provider 是否为 `ACTIVE`。Skill 通常在初始化后保持 `INACTIVE`，由 Executor 在首次调用时激活。

## 三种传输

| Transport | 绑定参数 | endpoint 示例 | 典型用途 |
|---|---|---|---|
| gRPC | `proto_file`、`service_name`、`method` | `127.0.0.1:50105` | 生命周期、RPC、流式二进制接口 |
| ROS 2 | `qos_profile` | `/camera/color/image_raw` | Topic 与 ROS 2 数据平面 |
| MCP | `input_schema_json` | `http://127.0.0.1:50106/mcp` | Pilot 可发现的工具调用 |

同一 provider 可以用不同传输暴露不同 contract，也可以用多个传输暴露同一 contract。

Atlas 会检查 `(transport, endpoint)` 冲突。ROS 2 endpoint 可由 Atlas 生成或在冲突时改名；gRPC 和 MCP endpoint 发生冲突时，provider 必须选择新地址。provider 必须使用 `DeclareCapabilityResponse.endpoint` 返回的最终值。

## Query 与 ConnectCapability

`Query` 返回 provider、生命周期、contract、transport 和传输参数，但故意不返回 endpoint。消费者准备调用时必须执行：

```text
ConnectCapability(consumer_id, provider_id, contract_id, transport)
  -> channel_id + endpoint + transport params
```

Atlas 记录这条 consumer 到 provider 的边，并把可拨号 endpoint 返回给 consumer。随后 consumer 直接建立 gRPC、ROS 2 或 MCP 连接；数据不会经过 Atlas。

使用结束后调用 `DisconnectCapability(channel_id)`。该操作是幂等的。provider 注销、心跳超时或同 ID takeover 时，Atlas 会自动清除指向旧 provider 的 channel。

## 命名空间诊断

普通 contract 应位于 provider 声明的主 namespace 下。若不一致，Atlas 仍接受 Capability，但会设置 `namespace_mismatch=true` 并记录 warning。contract descriptor 显式设置 `cross_namespace = true` 时，不产生该诊断。

可使用以下命令查看：

```bash
rbnx caps -v
rbnx caps --json
rbnx channels
rbnx inspect
```

namespace 只帮助分类、检索和发现错误配置，不阻止系统启动或能力调用。

## CAPABILITY.md 如何到达 Pilot

Python provider 默认查找 package 根目录的 `CAPABILITY.md`。provider 在自己的文件系统中读取文档，并在注册时同时发送：

- `capability_md_path`：仅用于诊断；容器内路径不保证能被宿主机访问。
- `capability_md`：完整 Markdown 内容；这是跨主机、跨容器的可移植数据。

Pilot 只把 provider ID、kind 和文档 frontmatter 中的简短 `description` 放入提示词。当需要完整说明时，模型调用 Executor 的 `read_capability_doc` builtin，并传入 `provider_id`。该 builtin 从 Atlas 读取已注册的内容。

因此，能力文档流程中不要让模型猜文件路径，也不要用 `read_file` 读取 provider 的 `CAPABILITY.md`。

## 心跳与生命周期

provider 客户端默认每 30 秒发送一次 Heartbeat。当前 Atlas 默认参数为：

| 参数 | 默认值 | 环境变量 |
|---|---:|---|
| 标记为 `TERMINATED` 的超时 | 90 秒 | `ROBONIX_ATLAS_HEARTBEAT_TIMEOUT_MS` |
| `TERMINATED` 后删除记录的延迟 | 10 分钟 | `ROBONIX_ATLAS_GC_AFTER_TERMINATED_MS` |
| 驱逐扫描间隔 | 10 秒 | `ROBONIX_ATLAS_EVICTION_INTERVAL_MS` |

同 kind、同 ID 的重新注册采用 takeover 语义：旧 Capability 和相关 channel 会被移除，新 provider 必须重新声明 Capability。不同 kind 使用同一个 ID 会返回 `AlreadyExists`。

## 主要 RPC

| RPC | 作用 |
|---|---|
| `RegisterPrimitive` / `RegisterService` / `RegisterSkill` | 注册 provider 身份 |
| `DeclareCapability` | 声明一条 contract 与 transport 绑定 |
| `Heartbeat` / `Unregister` | 维持或结束注册 |
| `SetLifecycleState` | 上报 provider 生命周期状态 |
| `Query` | 按 provider ID、namespace、kind、contract、transport 发现能力 |
| `ConnectCapability` / `DisconnectCapability` | 获取 endpoint 并记录或释放 channel |
| `QueryContract` / `ListContracts` | 查询标准 contract descriptor |
| `InspectAtlas` | 获取 provider、Capability 和 channel 的调试快照 |

Atlas 的 wire 定义位于 `system/atlas/proto/atlas.proto`。标准 contract descriptor 位于 Robonix 源码树的 `capabilities/`。
