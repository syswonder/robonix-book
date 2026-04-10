# 快速上手

本节从零开始，带你构建 Robonix 并运行完整的 Tiago 仿真 Demo，整个流程只需一个脚本 `examples/run.sh`。

## 前置条件

| 依赖 | 说明 |
|------|------|
| Linux x86_64 (glibc) | 主要开发和测试平台 |
| Rust 工具链 (stable >= 1.85) | `rustup` 安装即可 |
| Python >= 3.10 | VLM 服务和 MCP 桥接均为 Python |
| Docker + Compose v2 | Tiago 仿真栈运行在容器中 |
| X11 桌面 | Webots 和 rviz2 的 GUI 需要 X11 转发 |
| NVIDIA GPU + nvidia-container-toolkit | 推荐；无 GPU 时 Webots 3D 渲染会非常卡顿 |
| VLM API 密钥 | 任意 OpenAI 兼容的视觉语言模型接口 |

如果只需运行 VLM + Agent 而不启动仿真，可跳过 Docker、X11、GPU 三项。

## 克隆与构建

```bash
git clone https://github.com/syswonder/robonix
cd robonix
git submodule update --init --recursive
cd rust
cargo build --workspace
make install    # 安装 rbnx, robonix-codegen, robonix-pilot, robonix-atlas 到 ~/.cargo/bin
```

安装 Python 依赖：

```bash
pip install -r examples/requirements.txt
```

### MCP 与 codegen（必读）

凡是在控制平面声明 **`supported_transports` 含 `"mcp"`**、并填写 **`DeclareInterface.listen_port`**（MCP HTTP 数据面端口）的 provider，其 Python 侧必须使用 **`robonix-codegen --lang mcp`** 根据 `rust/crates/robonix-interfaces/lib` 下的 ROS IDL 生成 **`robonix_mcp_types/`**（各 `*_mcp.py` dataclass），与 `rust/contracts` 中的 **`[io]`** 对齐。否则无法 `import` 契约类型，FastMCP 工具签名也无法与契约保持一致，**不应在缺少生成物的情况下直接启动桥接进程**。

- **推荐**：使用包的 **`rbnx build`**（或示例中 `tiago_sim_stack` / `maniskill_vla_demo` / `memsearch_service` 的 `scripts/build.sh`），脚本会在镜像构建或本地环境准备阶段执行 codegen，并将 **`robonix_mcp_types/`** 与共享包 **`rust/examples/packages/robonix_mcp_contract`**（import 名 **`robonix_mcp_contract`**）加入 **`PYTHONPATH`**（或以 `pip install -e` 安装该包）。
- **工具注册**：MCP 工具请使用 **`from robonix_mcp_contract import mcp_contract`** 与 **`@mcp_contract(mcp, contract_id=…, input_cls=…, output_cls=…)`** 注册，**不要**用裸 **`@mcp.tool()`** 手写与契约线格式不一致的参数名（例如 `std_msgs/String` 的 JSON 顶层键为 **`data`**）。线格式与 **`input_cls.to_dict()` / `from_dict()`** 一致；图像类消息在 JSON 中 **`data`** 为 base64 字符串。
- **本地调试**：在仓库 `rust/` 下执行：

  ```bash
  cargo run -p robonix-codegen -- --lang mcp \
    -I crates/robonix-interfaces/lib \
    -o examples/packages/tiago_sim_stack/tiago_bridge/robonix_mcp_types
  ```

  生成目录已列入各包 **`.gitignore`**，请勿手动维护。

完成 codegen 后，再进行 MCP 端口注册与 `rbnx tools` 发现。`./examples/run.sh` 链路上的 **`rbnx build`** 已包含上述步骤（以各包 `build.script` 为准）。

## 配置 VLM

```bash
cp examples/.env.example examples/.env
```

在 `examples/.env` 中填入 VLM 服务信息。以下是几种常见后端的示例：

```bash
# Qwen (阿里 DashScope，默认)
VLM_API_KEY=sk-xxx
VLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
VLM_MODEL=qwen3-vl-plus

# OpenAI GPT-4o
VLM_API_KEY=sk-xxx
VLM_BASE_URL=https://api.openai.com/v1
VLM_MODEL=gpt-4o

# Google Gemini (OpenAI 兼容接口)
VLM_API_KEY=AIza...
VLM_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai
VLM_MODEL=gemini-2.0-flash
```

## 一键运行

从 `rust/` 目录执行：

```bash
./examples/run.sh
```

脚本按以下顺序启动所有组件：

1. 检查 Python 依赖（grpcio, openai, mcp, numpy, PIL, uvicorn）
2. 启动 `robonix-atlas`，监听 `0.0.0.0:50051`
3. 通过 `rbnx validate + build + start` 启动 `vlm_service`——Python VLM 服务注册到控制平面，声明 `chat` 接口（gRPC）
4. 通过 `rbnx validate + build + start` 启动 `tiago_sim_stack`——`docker compose up` 构建并启动容器，容器内依次启动 Webots（3D 仿真）、Nav2（导航栈）、rviz2（可视化）、`tiago_bridge`（MCP 桥接，向控制平面注册工具接口）
5. 启动 `robonix-pilot`，注册自身，发现 VLM 和 MCP 工具，进入交互模式

> 注意（MCP 必读）：如果 provider 要声明 MCP 接口（`supported_transports=["mcp"]` + `listen_port`），必须在启动前完成 codegen。当前示例由包的 `scripts/build.sh` 自动执行：先生成 `robonix_mcp_types/`（`robonix-codegen --lang mcp`），再在 package 内准备 `tiago_bridge/proto_gen/`，最后进行 MCP 端口注册与服务启动。

启动完成后 Agent 以后台 gRPC 服务运行。使用 `rbnx chat` 在独立 TUI 中与 Agent 交互。

## 只跑子集

通过环境变量控制启动哪些组件：

```bash
# 只启动 VLM + Agent（不启动仿真容器）
START_SIM_STACK=0 ./examples/run.sh

# 只启动控制平面和仿真栈（不启动 Agent，用于调试桥接）
START_AGENT=0 ./examples/run.sh

# 使用已运行的 robonix-atlas（适合开发时反复重启 Agent）
SMOKE_USE_EXISTING_SERVER=1 ./examples/run.sh
```

## 交互示例

Agent 启动后在另一个终端运行 `rbnx chat` 进入 TUI 交互界面：

```bash
rbnx chat
```

在 TUI 中输入指令即可驱动机器人。**按 Esc** 可向 Pilot 发送 **`AbortSession`**，中断当前轮次的推理或工具执行（与 [Provider 注册](../integration-guide/provider-registration.md) 中说明一致）。

一个典型的 ReAct 交互如下：

```
> find the door and navigate to it
```

Agent 的 ReAct 循环执行步骤：

1. 调用 `camera_snapshot` 获取当前视野
2. VLM 分析图像，判断门的方向
3. 调用 `base_cmd` 旋转扫描，或 `base_navigate` 发送导航目标
4. 重复感知–行动循环，直到任务完成

Agent 在运行中持续交替调用感知工具（如 `camera_snapshot`、`robot_state`）和控制工具（如 `base_cmd`、`base_navigate`），形成连续的 sense-act 闭环。载荷类型与 `rust/contracts` 一致，由 **`robonix-codegen --lang mcp`** 生成 Python dataclass（见 `tiago_bridge/robonix_mcp_types/`）。当工具返回 **`sensor_msgs/Image` 线 JSON**（含 `width`/`height`/`encoding`/`data`）或顶层 **`image_base64`** 时，Pilot 会将图像以多模态形式送入 VLM 进行视觉推理。

## 节点注册与发现

`run.sh` 启动完成后，系统中有三个节点：

| 节点 | node_id | namespace | 接口 | 传输 |
|------|---------|-----------|------|------|
| VLM 服务 | `com.robonix.services.vlm` | `robonix/srv/model/vlm` | `chat` | gRPC |
| Tiago 桥接 | `com.robonix.prm.tiago` | `robonix/prm` | 每契约一个 MCP 接口名（如 `camera_snapshot`、`base_navigate`…）+ `rgb` | MCP, gRPC, ROS 2 |
| Pilot（对话入口） | `com.robonix.runtime.pilot` | `robonix/srv/runtime/pilot` | `pilot` | gRPC |

其中 VLM 在控制平面上只声明 `chat` 接口，数据面 gRPC 在同一监听端口上同时实现一元 `Chat` 与 server-streaming `ChatStream`（见 `rust/crates/robonix-interfaces/lib/vlm/srv/`）。

使用 `rbnx` CLI 查看运行时状态：

```bash
# 查看所有已注册节点
rbnx nodes

# 查看可用的 MCP 工具
rbnx tools

# 导出完整运行时快照（JSON）
rbnx inspect

# 与 Agent 交互
rbnx chat

# 生成系统拓扑图
rbnx graph -o topology.png
```

## 故障排除

Webots GUI 不显示时，检查 `DISPLAY` 环境变量是否已设置，并确认 Docker 可以访问 X11。通常执行 `xhost +local:docker` 即可解决，`run.sh` 在检测到 `xhost` 命令时会自动执行此步骤。

Webots 渲染卡顿通常是缺少 GPU 透传所致。确认 `nvidia-smi` 可执行且 `nvidia-container-toolkit` 已安装。`robonix_manifest.yaml` 中的启动脚本会自动检测 `nvidia-smi` 并合并 `compose.gpu.yaml`。

MCP 工具调用失败时，检查 `tiago_bridge` 是否已完成注册。容器中 Webots + Nav2 的启动约需 40 秒，在此期间 MCP 工具尚不可用。可通过 `cargo run -p robonix-cli -- tools` 确认工具列表是否非空。

若日志出现 `COPY rust/examples/packages/tiago_sim_stack/tiago_bridge/proto_gen ... not found` 或 MCP schema/参数类型不匹配，先重新执行：

```bash
rbnx build -p examples/packages/tiago_sim_stack
```

确认构建阶段已完成 `robonix-codegen --lang mcp` 与 `proto_gen` 生成，再执行 `rbnx start`。
