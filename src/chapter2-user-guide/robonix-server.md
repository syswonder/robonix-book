# robonix-server

<!-- toc -->

## 作用

- 提供 gRPC meta API（默认 `0.0.0.0:50051`）：节点注册、Query channel 分配与解析
- 运行 ping query 服务（`robonix/system/debug/ping`）
- 使用 rclrs + rmw_zenoh 作为 ROS 2 传输层

## 启动

```bash
cd rust
./start_server
```

`start_server` 会执行 `setup_zenoh_rclrs_env`（source ROS、设置 RMW、colcon build、source overlay），然后启动 `robonix-server`。

## 环境变量

| 变量 | 说明 |
|------|------|
| `RMW_IMPLEMENTATION` | 默认 `rmw_zenoh_cpp` |
| `ROBONIX_META_GRPC_ADDR` | 监听地址，默认 `0.0.0.0:50051` |
| `ROBONIX_META_GRPC_ENDPOINT` | 客户端连接地址，默认同 `ROBONIX_META_GRPC_ADDR` |
| `RUST_LOG` | 日志级别，如 `info`、`debug` |

## 确认运行

- 日志中出现 `meta-runtime: registered node 'robonix-server'`、`ping query runtime ready`
- `./callquery robonix-server robonix/system/debug/ping '"test"'` 返回 `pong:"test"`

**说明**：callquery 仅支持 request/response 均为 `std_msgs/msg/String` 的 query。`semantic_query`、`map_manager`、`model_manager`、`skill_library` 等使用 `robonix_msg` 类型（如 Object[]），需用 Python 客户端（如 query_demo）调用。
