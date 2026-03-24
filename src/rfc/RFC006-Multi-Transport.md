# RFC 006: Robonix 多传输支持

| 版本 | 日期 |
|------|------|
| 0.1 | 2026-03-24 |

## 1. 目标与范围

Robonix 支持多种传输方式承载数据面，由控制平面完成声明、协商与端点分配；应用层协商后使用各生态原生 API，不再造统一消息总线抽象。本文列出传输类型、`DeclareInterface` / `NegotiateChannel` 职责及端点分配要点。

不在本 RFC 内：各传输的具体序列化格式、QoS 细节、安全传输与加密。


## 2. 支持的传输类型

| Transport | 典型用途 |
|-----------|----------|
| `ros2` | 与现有机器人软件栈互通；topic / service / action 等。 |
| `grpc` | 跨语言服务调用、与云或边缘组件对接。 |
| `zenoh` | 基于 Eclipse Zenoh 的 pub/sub；数据以 Apache Arrow IPC 编码，适用于跨进程/跨机器的高性能场景。 |
| `shared_memory` | 低延迟大数据块；共享内存段、同步原语由平台约定。 |

枚举可扩展；新传输需在控制平面注册语义与协商字段，并保持 `NegotiateChannel` 返回结构可扩展。


## 3. 声明与协商

1. `DeclareInterface`（提供方）
   节点为某接口声明其支持的 Transport 列表（可多选）。可与 RIDL 接口身份绑定，由实现将 `interface_id` 与传输能力关联。

2. `NegotiateChannel`（消费方）
   消费者指定目标 `(node_id, interface_id)` 与所选 Transport。控制平面校验提供方是否支持该传输，若通过则分配具体通道描述并返回。

3. 端点分配
   系统生成或登记具体端点标识，例如：
   - `ros2`：topic / service / action 名称（及可选命名空间前缀策略）；
   - `grpc`：服务名、方法路径或完整 authority + path；
   - `zenoh`：Zenoh key expression（如 `/rbnx/zenoh/<uuid>`）；
   - `shared_memory`：共享内存 key、段大小、版本或能力协商字段。


## 4. 不抽象通信原则

- Robonix 不在数据面统一抽象通信模式；协商结果仅为元数据 + 端点，调用方随后直接使用：
  - ROS2：`rclcpp` / `rclpy` 等；
  - gRPC：原生 stub；
  - Zenoh：`eclipse-zenoh` API；
  - 共享内存：操作系统或项目约定的映射与同步 API。

控制平面不路由业务载荷（与 RFC003 一致）。


## 5. 各传输端点分配策略（概要）

| Transport | 策略要点 |
|-----------|----------|
| `ros2` | 全局唯一 topic/service 名；可按 `node_id` + `interface_id` 哈希或注册表递增分配；避免与用户硬编码冲突的策略由实现定义（如预留前缀 `robonix/`）。 |
| `grpc` | 分配逻辑服务名与监听地址，或仅分配 path 由已有 server 承载；多租户时纳入 namespace。 |
| `zenoh` | 分配全局唯一的 key expression（如 `/rbnx/zenoh/<uuid>`）；Zenoh 会话按此 key 进行 pub/sub。 |
| `shared_memory` | 分配唯一 shm key（或文件路径句柄）；可附带 fd 传递协商标记（高级场景见 RFC007）。 |


## 6. 与相关 RFC 的关系

| RFC | 关系 |
|-----|------|
| RFC001 | RIDL 定义接口契约；传输选择与 RIDL 语义正交，由实现绑定。 |
| RFC003 | 控制平面 API 总览；本文细化 Transport 与协商。 |
| RFC004 | MCP 工具调用后，实际数据路径仍可能依赖本文协商结果。 |

小结：`DeclareInterface` 声明能力、`NegotiateChannel` 选定并分配端点；Robonix 只做控制面元数据，数据面坚持原生 API，策略保持简单、可观测、可扩展。
