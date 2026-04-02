# 接口目录

接口载荷与运行时 gRPC/ROS 形状的**权威定义**在 **`rust/robonix-interfaces/lib/**`**（ROS IDL：`.msg` / `.srv`）。

**契约**（稳定 ID + 通信模型摘要）在 **`rust/contracts/**/*.toml`**：`[contract] id` 即控制平面上的 **`contract_id`**，与 `DeclareInterface` / `QueryNodes` 使用同一字符串。

### 契约源码路径（仓库内） {#contract-toml-sources}

相对仓库根目录，当前已提交的契约文件如下（与 `[contract] id` 一一对应；见 **`rust/contracts/README.md`**）：

```
rust/contracts/
├── README.md
├── prm/
│   ├── base_move.v1.toml      # robonix/prm/base/move
│   ├── base_odom.v1.toml      # robonix/prm/base/odom
│   ├── camera_rgb.v1.toml     # robonix/prm/camera/rgb
│   ├── camera_depth.v1.toml   # robonix/prm/camera/depth
│   ├── sensor_imu.v1.toml     # robonix/prm/sensor/imu
│   └── sensor_lidar.v1.toml   # robonix/prm/sensor/lidar
└── sys/
    ├── pilot.v1.toml          # robonix/sys/runtime/pilot
    ├── executor.v1.toml       # robonix/sys/runtime/executor
    ├── liaison.v1.toml        # robonix/sys/runtime/liaison
    ├── vlm_chat.v1.toml       # robonix/sys/model/vlm/chat
    └── memory_search.v1.toml  # robonix/sys/memory/search
```

下文各页表格中的 **契约源码** 列：已入库则写 `rust/contracts/...`；尚未单独建 TOML 的契约标为 **—**（仍以 IDL / 目录为准，后续可补 TOML）。

文档按 **`RegisterNode.kind`** 与命名空间习惯分为两级：

| 类别 | 路径前缀 / 说明 | 入口 |
|------|-----------------|------|
| **原语（primitive）** | `robonix/prm/...`，多为硬件/仿真能力 | [primitive/](primitive/index.md) |
| **服务（service）** | `robonix/sys/...`，模型、记忆、Runtime 等 | [service/](service/index.md) |

## 代码生成（ridlc）

- **`robonix-interfaces/robonix_proto/*.proto`**：全部由 **`ridlc`** 根据 `lib/`（及 `--contracts`）生成，**禁止手工编辑**。
- 典型命令（在 `rust/` 下）：

```bash
cargo run -p ridlc -- --lang proto \
  -I robonix-interfaces/lib \
  --contracts contracts \
  -o robonix-interfaces/robonix_proto
```

- 仅改 IDL、不跑 `--contracts` 时仍会生成各包 proto，但不会更新 `robonix_contracts.proto`。

若删除某个 IDL 包，需自行删掉对应 `.proto` 后重跑（ridlc 不自动删除）。完全重生成可先 `rm -f robonix-interfaces/robonix_proto/*.proto` 再执行上式。

生成 Python 桩（`examples/proto_gen/`）：

```bash
cd rust && ./examples/scripts/gen_proto_python.sh
```

## IDL 政策

- Pub-sub 用 `.msg`；带 RPC 的用 `.srv`；需要 **server-streaming** 时在 `.srv` 首行使用 `# @robonix.grpc stream_server pkg/msg/Name`（见 `lib/vlm/srv/ChatStream.srv`、相机与 runtime 等示例）。
- **不要在 `robonix_proto/` 添加手写 `.proto`**；控制面专用定义在 **`rust/proto/`**（如 `robonix_runtime.proto`）。

## Proto 命名规则

对 ROS 包 `pkgname`，ridlc 生成 `robonix_proto/pkgname.proto`，`package robonix.pkgname`：

- 每个 `.msg` → 同名 `message`。
- 每个 `.srv` → `Name_Request` / `Name_Response` + `rpc Name`，挂在 `PkgnameService` 上。

完整能力表与英文说明见 `rust/robonix-interfaces/README.md`、`rust/contracts/README.md`。

## 目录布局（摘录）

| 路径 | 内容 |
|------|------|
| `lib/pilot/`, `lib/executor/`, `lib/liaison/` | Runtime 载荷与 `*Service`（Pilot / Executor / Liaison） |
| `lib/prm_base/`, `lib/vlm/`, … | 原语与系统服务 IDL |
| `lib/common_interfaces/` | 上游标准消息 |
| `rust/contracts/` | 契约 TOML（`contract_id`、模式、`[io]`） |
| `robonix_proto/` | ridlc 生成物（含 `robonix_contracts.proto`） |

## 各命名空间接口（快速链接）

### 原语 `robonix/prm`

| 命名空间 | 页面 |
|----------|------|
| `robonix/prm/base` | [底盘](primitive/base.md) |
| `robonix/prm/camera` | [相机](primitive/camera.md) |
| `robonix/prm/sensor` | [传感器](primitive/sensor.md) |
| `robonix/prm/arm` | [机械臂](primitive/arm.md) |
| `robonix/prm/gripper` | [夹爪](primitive/gripper.md) |
| `robonix/prm/force_torque` | [力/力矩](primitive/force-torque.md) |

### 服务 `robonix/sys`

| 说明 | 页面 |
|------|------|
| 总览、契约表、扩展流程 | [服务（sys）](service/index.md) |
| Runtime Pilot / Executor / Liaison | [Pilot](service/pilot.md)、[Executor](service/executor.md)、[Liaison](service/liaison.md) |
| VLM Chat | [VLM Chat](service/vlm-chat.md) |
| Memory Search（gRPC + MCP 说明） | [Memory Search](service/memory-search.md) |
| 终端对话 | 由 **`rbnx chat`** 连接 **Pilot**（[`service/pilot.md`](service/pilot.md)），非独立 `agent_chat` 契约 |
