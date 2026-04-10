# 多 Transport 并发模型

## 概述

Robonix 支持多种 data-plane transport：gRPC、ROS 2、MCP、shared memory。一个 interface 在 `DeclareInterface` 时可以声明支持多种 transport（`supported_transports: ["grpc", "ros2"]`），consumer 通过 `NegotiateChannel` 选择其中一种建立通道。

**核心设计决策**：provider 在声明时**同时启动所有 supported transports**，而非按需延迟启动。

## 为什么同时启动

同一个 provider 的同一个 interface 可能被多个 consumer 以不同 transport 消费：

```
                            ┌── Nav2 ←─── ROS 2 topic (/robonix/slam/odom)
  SLAM provider             │
  (odom interface)  ────────┤── Pilot ←── gRPC stream (PrmBaseOdom.Stream)
  supported: [grpc, ros2]   │
                            └── 本地 VLA ←─ shm (零拷贝, 未来)
```

按需启动（lazy）虽然省资源，但会引入复杂的生命周期管理：第一个 consumer negotiate 时启动 transport，最后一个 release 时关闭，期间需要引用计数。目前阶段，eager 模式更简单可靠。

## 按 contract mode 区分的安全模型

不同 mode 的 contract 在多 transport 并发下有不同的安全特性：

### `topic_out` / `topic_in`（流式数据）

**安全**：本质上是一份数据的多路 fan-out。

- 各 transport 独立消费同一份数据源
- gRPC stream 有背压可能丢帧，ROS 2 best-effort QoS 也可能丢帧，但这是 transport 层面的正常行为
- consumer 之间互不影响
- 无状态，无副作用

```toml
# 示例：robonix/prm/base/odom
[mode]
type = "topic_out"       # ← 多 transport 并行安全

[semantics]
realtime = true
```

**provider 实现要求**：无特殊要求。gRPC servicer 和 ROS 2 publisher 可以从同一个数据源独立读取。

### `rpc`（请求-响应，可能有副作用）

**需要注意**：多个 transport 暴露同一个 RPC，意味着同一操作可能从不同入口并发调用。

```toml
# 示例：robonix/srv/slam/save_map
[mode]
type = "rpc"             # ← 需要 provider 保证并发安全

[semantics]
stateless = false        # 有副作用
```

**provider 实现要求**：

| 情况 | 要求 |
|------|------|
| 无副作用的查询（如 `get_status`） | 无需特殊处理 |
| 有副作用的写操作（如 `save_map`） | provider 必须内部加锁或保证幂等 |
| 有状态的模式切换（如 `switch_mode`） | 使用互斥锁保护状态变更 |

**示例**（Python，mapping_rbnx 中的做法）：

```python
class SlamState:
    def __init__(self):
        self._lock = threading.Lock()
        self.mode = "mapping"

    def switch_mode(self, new_mode: str) -> bool:
        with self._lock:          # 无论从 gRPC 还是 ROS 2 service 调用
            self.mode = new_mode  # 都经过同一把锁
            return True
```

### `rpc_server_stream` / `rpc_client_stream`（流式 RPC）

与 `topic_out` 类似，多 consumer 各自拿到独立的 stream，互不影响。provider 按 per-stream 管理资源即可。

## 已知问题与限制

### 1. 健康状态不一致

某个 transport 的 server 崩溃（如 ROS 2 node 挂了）但另一个还活着（gRPC 正常）。从 Atlas 角度看，node heartbeat 正常，但部分 consumer 的 channel 已经不可用。

**当前状态**：Atlas 只做 node 级别 heartbeat，不做 per-transport 健康检查。

**未来方向**：per-channel health probe，或 consumer 侧超时重协商。

### 2. 序列化开销

同一份数据被序列化为多种格式（protobuf for gRPC, CDR for ROS 2, raw bytes for shm）。对于高带宽数据（点云、图像），这个开销不可忽略。

**当前状态**：接受开销。

**未来方向**：`robonix-buffer` 提供 transport-agnostic 的零拷贝缓冲区，各 transport 层做 view 而非 copy。shm transport 已在 `zero_copy_demo` 中验证。

### 3. 语义等价性

不同 transport 对同一 contract 的语义保证可能不同：

| 特性 | gRPC | ROS 2 | shm |
|------|------|-------|-----|
| 可靠交付 | 是（TCP） | 看 QoS profile | 否（ring buffer） |
| 背压 | 有（HTTP/2 flow control） | 无（会丢） | 有（满则覆盖或阻塞） |
| 跨网络 | 是 | 是（DDS） | 否（同主机） |
| 延迟 | 中 | 中 | 极低 |

**当前状态**：contract 语义定义中不区分 transport 级别差异，由 consumer 根据场景自行选择。

**未来方向**：在 contract TOML 中可选声明 `[transport_hints]`，提示推荐 transport 和 QoS 要求。

## 总结

| 维度 | 设计选择 |
|------|---------|
| 启动策略 | **Eager**：声明时所有 supported transports 同时启动 |
| 并发安全 | **Provider 责任**：topic 类无需关心；rpc 类 provider 需保证并发安全 |
| 协商粒度 | **Per-interface**：同一 node 的不同 interface 可独立协商不同 transport |
| 多 consumer | **允许**：同一 interface 可被多个 consumer 以不同 transport 同时消费 |
| Atlas 角色 | **不做 transport 层面约束**：只记录 channel，不限制并发模式 |
