# 快速开始

本节引导你从零启动 Robonix E2E 原型：启动控制平面、注册仿真/推理节点、运行智能体。

## 前置条件

- Rust 工具链（rustup）
- Python 3 + `grpcio` + `openai` + `pyarrow` + `Pillow`
- 仿真依赖：`mujoco>=3.3.0`、`libero`、`lerobot[smolvla]`、`robosuite`
- 一个 OpenAI 兼容的 VLM API（默认 Qwen-VL-Plus，也可用 GPT-4o 等）

## 1. 构建

```bash
cd rust
cargo build --workspace
```

`robonix-server` 已从 ROS2 解耦，无需 ROS2 环境即可构建和运行。

## 2. 配置

```bash
cd rust/examples
cp .env.example .env
```

编辑 `.env`，填入 VLM API 信息：

```
ROBONIX_SERVER=localhost:50051
VLM_API_KEY=your-api-key
VLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
VLM_MODEL=qwen3-vl-plus
MUJOCO_GL=egl
```

若无 GPU，可设 `VLA_MOCK=1` 使用随机动作（仅用于测试集成流程）。

## 3. 一键启动

```bash
cd rust/examples
./run_e2e.sh
```

这会依次启动：

1. `robonix-server` -- 控制平面（gRPC 端口 50051）
2. `sim_env.py` -- LIBERO/MuJoCo 仿真环境（端口 50053）
3. `vla_service.py` -- SmolVLA 推理服务（端口 50054）
4. `vlm_service.py` -- VLM 对话服务（端口 50052）
5. `robonix-agent` -- 交互式 ReAct 智能体（前台运行）

agent 通过控制平面发现所有已注册的服务，使用 VLM 进行规划，调用 VLA 服务在 LIBERO 仿真中执行机械臂操作任务。

输入 `quit` 或 Ctrl+C 退出，所有后台进程会自动清理。

## 4. 分步启动（可选）

如果需要单独调试各组件：

```bash
# 终端 1：启动 server
cd rust && cargo run -p robonix-server

# 终端 2：启动仿真和服务节点
cd rust/examples
python3 nodes/sim_env.py &
python3 nodes/vla_service.py &
python3 nodes/vlm_service.py &

# 终端 3：启动 agent
cd rust && cargo run -p robonix-agent
```

## 5. 验证

启动后可通过 CLI 或 gRPC 查看注册状态：

```bash
# 使用 robonix-cli
rbnx nodes

# 或直接调用 gRPC
grpcurl -plaintext localhost:50051 robonix.runtime.RobonixRuntime/InspectRuntime
```

应能看到 `sim-env`、`vla-service`、`vlm-service` 等已注册节点及其接口。

## 下一步

- [robonix-server 详细说明](../chapter2-user-guide/robonix-server.md)
- [节点开发指南](../chapter3-developer-guide/package-development.md)
- [传输示例](../../rust/examples/transports/README.md)（ROS2、gRPC、Dora、共享内存）
