# robonix-server

## 作用

控制平面核心服务，提供 gRPC API：

- `RegisterNode`：节点注册（含 SKILL.md）
- `DeclareInterface`：声明接口及支持的传输类型
- `QueryNodes`：按命名空间、名称、传输类型发现服务
- `NegotiateChannel`：协商通道，系统分配端点（topic 名、shm key 等）
- `QueryAllSkills`：获取所有已注册的 SKILL.md

## 构建与启动

```bash
cd rust
cargo build -p robonix-server
cargo run -p robonix-server
```

需要 ROS2 环境（rclrs）。如有 colcon workspace，先 source 再启动：

```bash
./start_server
```

## 环境变量

| 变量 | 说明 |
|------|------|
| `ROBONIX_META_GRPC_ADDR` | 监听地址，默认 `0.0.0.0:50051` |
| `ROBONIX_META_GRPC_ENDPOINT` | 客户端连接地址，默认同上 |
| `RUST_LOG` | 日志级别，如 `robonix_server=info` |
| `RMW_IMPLEMENTATION` | 仅当本进程或插件使用 ROS2 客户端时相关；常见为 `rmw_fastrtps_cpp` |

## 确认运行

日志中出现 `starting robonix runtime meta API (gRPC)` 即表示服务就绪。

可通过 `InspectRuntime` RPC 查看当前注册状态（返回 JSON 快照）。

## 参考

- [RFC003: 控制平面设计](../rfc/RFC003-Control-Plane.md)
- [RFC006: 多传输支持](../rfc/RFC006-Multi-Transport.md)
