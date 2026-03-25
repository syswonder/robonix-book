# 接口目录

Robonix 的接口类型定义以 ROS IDL（`.msg` / `.srv`）为单一真相源，存储在 `rust/robonix-interfaces/lib/` 下。所有 `robonix-interfaces/robonix_proto/` 中的 `.proto` 文件均由 `ridlc` 从 IDL 生成，不要手工编辑。

完整的接口能力表和生成规则见 `rust/robonix-interfaces/README.md`。

## IDL 政策

ROS IDL 是权威定义。pub-sub 接口用 `.msg`，RPC 接口用 `.srv`。Proto 文件只是机器生成的投影——如果需要修改接口，改 IDL 然后重新生成。

不要在 `robonix_proto/` 目录中添加与硬件供应商或应用相关的自定义 `.proto`。扩展系统接口的唯一方式是在 `lib/` 下添加或修改 ROS IDL 包，然后重新运行 `ridlc`。

## 生成命令

从 ROS IDL 重新生成所有 `.proto` 文件：

```bash
cd rust
cargo run -p ridlc -- --lang proto \
  -I robonix-interfaces/lib \
  -o robonix-interfaces/robonix_proto
```

如果删除了某个 IDL 包，需要先手工删除对应的 `.proto` 文件（`ridlc` 只生成不删除）。完全重新生成：

```bash
rm -f robonix-interfaces/robonix_proto/*.proto
cargo run -p ridlc -- --lang proto \
  -I robonix-interfaces/lib \
  -o robonix-interfaces/robonix_proto
```

生成 Python 桩文件（`examples/proto_gen/`）：

```bash
cd rust && ./examples/scripts/gen_proto_python.sh
```

## Proto 命名规则

对 ROS 包 `pkgname`，`ridlc` 生成 `robonix_proto/pkgname.proto`，包名空间 `robonix.pkgname`：

- 每个 `.msg` 文件生成一个同名 `message`
- 每个 `.srv` 文件生成 `Name_Request` / `Name_Response` message 和一个 `rpc Name` 方法，挂在 `PkgnameService`（PascalCase 包名 + `Service`）上

如果 `.srv` 文件首行包含 `# @robonix.grpc stream_server` 指令，`ridlc` 会生成 server-streaming RPC 而非 unary RPC（用于相机图像流等场景）。

## 目录布局

| 路径 | 内容 |
|------|------|
| `lib/prm_base/msg/`, `lib/prm_base/srv/` | 底盘原语 `robonix/prm/base/*` 的 IDL |
| `lib/prm_camera/srv/` | 相机 streaming RPC 指令 |
| `lib/vlm/srv/` | VLM 系统服务 `robonix/sys/model/vlm` |
| `lib/robonix_msg/msg/` | 共享消息类型（ChatMessage, RGBD 等） |
| `lib/common_interfaces/` | 上游 ROS 标准消息（sensor_msgs, geometry_msgs 等） |
| `robonix_proto/` | 生成的 `.proto` 文件 |

## 各命名空间接口

以下各页按命名空间列出所有已定义的接口、通信模式、IDL 来源和 gRPC 映射。

| 命名空间 | 页面 |
|----------|------|
| `robonix/prm/base` | [底盘](base.md) |
| `robonix/prm/camera` | [相机](camera.md) |
| `robonix/prm/sensor` | [传感器](sensor.md) |
| `robonix/prm/arm` | [机械臂](arm.md) |
| `robonix/prm/gripper` | [夹爪](gripper.md) |
| `robonix/prm/force_torque` | [力/力矩](force-torque.md) |
| `robonix/sys/model/vlm` | [系统服务](system-services.md) |
