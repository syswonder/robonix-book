# 快速上手

本节从零开始，带你构建 Robonix 并运行完整的 Tiago 仿真 Demo。整个流程只需要一个脚本 `examples/run.sh`。

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

如果只想跑 VLM + Agent（不启动仿真），可以跳过 Docker、X11、GPU 三项。

## 克隆与构建

```bash
git clone https://github.com/syswonder/robonix
cd robonix
git submodule update --init --recursive
cd rust
cargo build --workspace
make install    # 安装 rbnx, ridlc, robonix-agent, robonix-server 到 ~/.cargo/bin
```

安装 Python 依赖：

```bash
pip install -r examples/requirements.txt
```

## 配置 VLM

```bash
cp examples/.env.example examples/.env
```

在 `examples/.env` 中填入你的 VLM 服务信息。以下是几种常见后端的示例：

```bash
# Qwen (阿里 DashScope，默认)
VLM_API_KEY=sk-xxx
VLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
VLM_MODEL=qwen-vl-plus-latest

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
2. 启动 `robonix-server`，监听 `0.0.0.0:50051`
3. 通过 `rbnx validate + build + start` 启动 `vlm_service`——Python VLM 服务注册到控制平面，声明 `chat` 接口（gRPC）
4. 通过 `rbnx validate + build + start` 启动 `tiago_sim_stack`——`docker compose up` 构建并启动容器，容器内依次启动 Webots（3D 仿真）、Nav2（导航栈）、rviz2（可视化）、`tiago_bridge`（MCP 桥接，向控制平面注册工具接口）
5. 启动 `robonix-agent`，注册自身，发现 VLM 和 MCP 工具，进入交互模式

启动完成后 Agent 等待 stdin 输入自然语言指令。输入 `quit` 退出。

## 只跑子集

通过环境变量控制启动哪些组件：

```bash
# 只启动 VLM + Agent（不启动仿真容器）
START_SIM_STACK=0 ./examples/run.sh

# 只启动控制平面和仿真栈（不启动 Agent，用于调试桥接）
START_AGENT=0 ./examples/run.sh

# 使用已经运行的 robonix-server（适合开发时反复重启 Agent）
SMOKE_USE_EXISTING_SERVER=1 ./examples/run.sh
```

## 交互示例

Agent 启动后会打印 `robonix-agent ready. Type 'quit' to exit.`，之后在终端输入指令即可驱动机器人。一个典型的 ReAct 交互如下：

```
> find the door and navigate to it
```

Agent 的 ReAct 循环会这样执行：

1. 调用 `get_camera_image` 获取当前视野
2. VLM 分析图像，判断门在哪个方向
3. 调用 `move_base` 旋转扫描，或 `navigate_to` 发送导航目标
4. 重复感知–行动循环，直到任务完成

Agent 在运行中会持续交替调用感知工具（`get_camera_image`、`get_robot_pose`）和控制工具（`move_base`、`navigate_to`），形成连续的 sense-act 闭环。

## 节点注册与发现

`run.sh` 启动完成后，系统中有三个节点：

| 节点 | node_id | namespace | 接口 | 传输 |
|------|---------|-----------|------|------|
| VLM 服务 | `com.robonix.services.vlm` | `robonix/sys/model/vlm` | `chat` | gRPC |
| Tiago 桥接 | `com.robonix.prm.tiago` | `robonix/prm/camera` | `mcp_tools`, `rgb` | MCP, gRPC, ROS 2 |
| Agent | `com.robonix.runtime.agent` | `robonix/sys/runtime/agent` | — | — |

可以用 `rbnx` CLI 查看运行时状态：

```bash
# 查看所有已注册节点
cargo run -p robonix-cli -- nodes --server 127.0.0.1:50051

# 查看可用的 MCP 工具
cargo run -p robonix-cli -- tools --server 127.0.0.1:50051

# 导出完整运行时快照（JSON）
cargo run -p robonix-cli -- inspect --server 127.0.0.1:50051
```

## 故障排除

Webots GUI 不显示时，检查 `DISPLAY` 环境变量是否已设置，并确保 Docker 可以访问 X11。通常执行 `xhost +local:docker` 即可解决。`run.sh` 在检测到 `xhost` 命令时会自动执行这一步。

Webots 非常卡顿通常是缺少 GPU 透传。确认 `nvidia-smi` 可执行、`nvidia-container-toolkit` 已安装。`robonix_manifest.yaml` 中的启动脚本会自动检测 nvidia-smi 并合并 `compose.gpu.yaml`。

MCP 工具调用失败时，检查 `tiago_bridge` 是否已完成注册。容器中 Webots + Nav2 的启动约需 40 秒，在此期间 MCP 工具尚不可用。可通过 `cargo run -p robonix-cli -- tools` 确认工具列表是否非空。
