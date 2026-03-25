# 快速上手

本节介绍如何从零开始构建并运行 Robonix 最小系统。

## 前置条件

- Linux（推荐 x86_64，glibc）
- Rust 工具链（`rustup` 安装即可，稳定版 ≥ 1.85）
- Python ≥ 3.10
- 一个 OpenAI 兼容 API 密钥（VLM 推理用）

如果要运行 Tiago 仿真 Demo，还需要：

- Docker + Docker Compose v2
- X11 桌面（用于 Webots / rviz2 GUI 显示）
- 推荐 NVIDIA 显卡 + nvidia-container-toolkit

## 克隆与构建

```bash
git clone https://github.com/syswonder/robonix
cd robonix
git submodule update --init --recursive
cd rust
cargo build --workspace
```

安装 Python 依赖：

```bash
pip install -r examples/requirements.txt
```

## 配置 VLM

```bash
cp examples/.env.example examples/.env
```

编辑 `examples/.env`，填入 VLM 服务的 API 地址和密钥：

```
VLM_API_BASE=https://api.openai.com/v1
VLM_API_KEY=sk-...
VLM_MODEL=gpt-4o
```

## 运行（VLM-only，无仿真）

最简模式只启动控制平面 + VLM 服务 + Agent：

```bash
cd rust
START_SIM_STACK=0 ./examples/run.sh
```

启动后 Agent 进入交互模式，通过 stdin 接收自然语言指令。输入 `quit` 退出。

## 运行（完整仿真 Demo）

带 Tiago Webots 仿真的完整 E2E 流程：

```bash
cd rust
./examples/run.sh
```

这将依次启动：

1. `robonix-server` — 控制平面 gRPC 服务
2. `vlm_service` — VLM 推理服务
3. `tiago_sim_stack` — Docker 容器（Webots + Nav2 + rviz2 + MCP 桥接）
4. `robonix-agent` — 交互式智能体

详细说明见 [Tiago 端到端 Demo](tiago-e2e-demo.md)。

## 格式化代码

```bash
cd rust
make fmt       # 格式化所有 Rust 代码
make check     # 检查格式 + clippy 警告
```
