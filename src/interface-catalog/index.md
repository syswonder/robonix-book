# 接口目录

接口载荷与运行时 gRPC/ROS 形状的**权威定义**在 **`rust/crates/robonix-interfaces/lib/**`**（ROS IDL：`.msg` / `.srv`）。

**契约**（稳定 ID + 通信模型摘要）在 **`rust/contracts/**/*.toml`**：`[contract] id` 即控制平面上的 **`contract_id`**，与 `DeclareInterface` / `QueryNodes` 使用同一字符串。

### 契约源码路径（仓库内） {#contract-toml-sources}

相对仓库根目录，**已提交的契约 TOML** 以仓库内实际文件为准（与 `[contract] id` 一一对应；完整说明见 **`rust/contracts/README.md`**）。当前 `rust/contracts/` 大致包含：

```
rust/contracts/
├── README.md
├── prm/
│   ├── base_cmd.v1.toml              # robonix/prm/base/cmd
│   ├── base_move.v1.toml             # robonix/prm/base/move
│   ├── base_odom.v1.toml             # robonix/prm/base/odom
│   ├── base_twist_in.v1.toml         # robonix/prm/base/twist_in
│   ├── base_navigate.v1.toml         # robonix/prm/base/navigate
│   ├── base_nav_status.v1.toml       # robonix/prm/base/nav_status
│   ├── base_nav_cancel.v1.toml       # robonix/prm/base/nav_cancel
│   ├── camera_rgb.v1.toml            # robonix/prm/camera/rgb
│   ├── camera_depth.v1.toml          # robonix/prm/camera/depth
│   ├── camera_snapshot.v1.toml       # robonix/prm/camera/snapshot
│   ├── camera_depth_snapshot.v1.toml # robonix/prm/camera/depth_snapshot
│   ├── manipulation_exec.v1.toml     # robonix/prm/manipulation/exec
│   ├── perception_detect.v1.toml    # robonix/prm/perception/detect
│   ├── robot_state.v1.toml           # robonix/prm/robot/state
│   ├── sensor_imu.v1.toml            # robonix/prm/sensor/imu
│   ├── sensor_lidar.v1.toml          # robonix/prm/sensor/lidar
│   └── sensor_lidar_snapshot.v1.toml # robonix/prm/sensor/lidar_snapshot
└── sys/
    ├── pilot.v1.toml                 # robonix/sys/runtime/pilot
    ├── executor.v1.toml              # robonix/sys/runtime/executor
    ├── liaison.v1.toml               # robonix/sys/runtime/liaison
    ├── vlm_chat.v1.toml              # robonix/sys/model/vlm/chat
    ├── memory_search.v1.toml         # robonix/sys/memory/search
    ├── memory_save.v1.toml           # robonix/sys/memory/save
    └── memory_compact.v1.toml        # robonix/sys/memory/compact
```

下文各页表格中的 **契约源码** 列：已入库则写 `rust/contracts/...`；尚未单独建 TOML 的契约标为 **—**（仍以 IDL / 目录为准，后续可补 TOML）。

文档按 **`RegisterNode.kind`** 与命名空间习惯分为两级：

| 类别 | 路径前缀 / 说明 | 入口 |
|------|-----------------|------|
| **原语（primitive）** | `robonix/prm/...`，多为硬件/仿真能力 | [primitive/](primitive/index.md) |
| **服务（service）** | `robonix/sys/...`，模型、记忆、Runtime 等 | [service/](service/index.md) |

## 代码生成（robonix-codegen）

- **`crates/robonix-interfaces/robonix_proto/*.proto`**：全部由 **`robonix-codegen`** 根据 `lib/`（及 `--contracts`）生成，**禁止手工编辑**。
- 典型命令（在 `rust/` 下）：

```bash
cargo run -p robonix-codegen -- --lang proto \
  -I crates/robonix-interfaces/lib \
  --contracts contracts \
  -o crates/robonix-interfaces/robonix_proto
```

- 仅改 IDL、不跑 `--contracts` 时仍会生成各包 proto，但不会更新 `robonix_contracts.proto`。

若删除某个 IDL 包，需自行删掉对应 `.proto` 后重跑（robonix-codegen 不自动删除）。完全重生成可先 `rm -f crates/robonix-interfaces/robonix_proto/*.proto` 再执行上式。

生成 Python 桩（建议生成到目标 package 内的 `proto_gen/`）：

```bash
cd rust && ./examples/scripts/gen_proto_python.sh
```

## MCP 与 Python 工具线格式（`--lang mcp`）

若 provider 在 **`DeclareInterface`** 中声明 **`supported_transports` 含 `"mcp"`**，除上述 **proto / gRPC** 外，还需用 **`robonix-codegen --lang mcp`** 生成 **`robonix_mcp_types/`**（各 ROS 消息的 `*_mcp.py` dataclass），并与契约 **`[io]`** 一致。

- **共享装饰器**：仓库 **`rust/examples/packages/robonix_mcp_contract`**，**`from robonix_mcp_contract import mcp_contract`**，用 **`@mcp_contract(mcp, contract_id=…, input_cls=…, output_cls=…)`** 注册 MCP 工具；MCP **`arguments`** 的 JSON 与 **`input_cls.to_dict()` / `from_dict()`** 一致（顶层键 = ROS 字段；如 **`std_msgs/String`** 为 **`{"data": "..."}`**；**`sensor_msgs/Image.data`** 在 JSON 中为 **base64**）。
- **`metadata_json.tools[].input_schema`** 应使用 **`InputCls.json_schema()`** 整表，与线格式对齐。
- 详细步骤与 **`rbnx chat` / Esc / Pilot→VLM 图像** 见 [接入指南 · Provider 注册](../integration-guide/provider-registration.md) 与 [快速上手 · MCP 与 codegen](../getting-started/quickstart.md)。

## IDL 政策

- Pub-sub 用 `.msg`；带 RPC 的用 `.srv`。
- **`robonix_contracts.proto` 中服务的形状**完全由契约 TOML 的 **`[mode].type`** 决定：`rpc`（一元 `.srv`）、`rpc_server_stream`（`.srv` **response** 段仅一个字段 = 流元素）、`rpc_client_stream`（**request** 段仅一个字段 = 流元素）、`topic_out` / `topic_in`（单条 **`[io.msg].msg`**）。详见 **`rust/contracts/README.md`**。
- **per-package `*Service`**（如 `vlm.proto` 中的 `VlmService`）当前由 codegen 为每个 `.srv` 生成 **unary** `rpc`。**带 stream 的 gRPC 门面**只在 **`robonix_contracts.proto`**：由 **`[mode].type`** 与对应 `.srv` 的 **request/response 单字段约定** 共同决定（见 `rust/contracts/README.md`）。
- **不要在 `robonix_proto/` 添加手写 `.proto`**；控制面专用定义在 **`rust/proto/`**（如 `robonix_runtime.proto`）。

## Proto 命名规则

对 ROS 包 `pkgname`，robonix-codegen 生成 `robonix_proto/pkgname.proto`，`package robonix.pkgname`：

- 每个 `.msg` → 同名 `message`（写入 `{pkg}.proto`）。
- 契约里引用的 `.srv` → 仅生成 `Name_Request` / `Name_Response`（同一 `{pkg}.proto`）；**不**再生成各包的 `*Service`。**gRPC 门面**只在 **`robonix_contracts.proto`**。

完整能力表与英文说明见 `rust/crates/robonix-interfaces/README.md`、`rust/contracts/README.md`。

## 目录布局（摘录）

| 路径 | 内容 |
|------|------|
| `lib/pilot/`, `lib/executor/`, `lib/liaison/` | Runtime 载荷与 `*Service`（Pilot / Executor / Liaison） |
| `lib/prm_base/`, `lib/vlm/`, … | 原语与系统服务 IDL |
| `lib/common_interfaces/` | 上游标准消息 |
| `rust/contracts/` | 契约 TOML（`contract_id`、模式、`[io]`） |
| `robonix_proto/` | robonix-codegen 生成物（含 `robonix_contracts.proto`） |

## 各命名空间接口（快速链接）

### 原语 `robonix/prm`

| 命名空间 | 页面 |
|----------|------|
| `robonix/prm/base` | [底盘](primitive/base.md) |
| `robonix/prm/camera` | [相机](primitive/camera.md) |
| `robonix/prm/sensor` | [传感器](primitive/sensor.md) |
| `robonix/prm/arm` | [机械臂](primitive/arm.md)（含 **`manipulation/exec`** 契约说明） |
| `robonix/prm/gripper` | [夹爪](primitive/gripper.md) |
| `robonix/prm/force_torque` | [力/力矩](primitive/force-torque.md) |
| `robonix/prm/perception` | [相机](primitive/camera.md)（**`perception/detect`**） |
| `robonix/prm/robot` | [底盘](primitive/base.md)（**`robot/state`**） |

### 服务 `robonix/sys`

| 说明 | 页面 |
|------|------|
| 总览、契约表、扩展流程 | [服务（sys）](service/index.md) |
| Runtime Pilot / Executor / Liaison | [Pilot](service/pilot.md)、[Executor](service/executor.md)、[Liaison](service/liaison.md) |
| VLM Chat | [VLM Chat](service/vlm-chat.md) |
| Memory Search（gRPC + MCP 说明） | [Memory Search](service/memory-search.md) |
| 终端对话 | 由 **`rbnx chat`** 连接 **Pilot**（[`service/pilot.md`](service/pilot.md)）；仓库**已移除** `agent_chat` / **AgentChat** 控制面契约，见 [Crate 索引 · robonix-pilot](../architecture/crates.md#robonix-pilot) |
