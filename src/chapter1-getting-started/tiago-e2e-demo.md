# Tiago 端到端 Demo

本节详细介绍 Robonix 的最小端到端演示：用自然语言控制 Tiago 机器人在 Webots 仿真环境中执行导航等任务。

## 前置条件

| 依赖 | 说明 |
|------|------|
| Docker + Compose v2 | Tiago 仿真栈运行在容器中 |
| X11 桌面 | Webots 和 rviz2 的 GUI 需要 X11 转发 |
| NVIDIA GPU（推荐） | Webots 3D 渲染；无 GPU 会非常卡顿 |
| nvidia-container-toolkit | 将 GPU 透传给 Docker |
| VLM API | OpenAI 兼容的视觉语言模型 |


## 启动流程

从 `rust/` 目录执行：

```bash
./examples/run.sh
```

`run.sh` 依次执行：

1. **检查** Python 依赖（grpcio, openai, mcp 等）
2. **启动 robonix-server** — 监听 `0.0.0.0:50051`
3. **rbnx validate + build + start `vlm_service`** — Python VLM 服务注册到控制平面，声明 `chat` 接口（gRPC）
4. **rbnx validate + build + start `tiago_sim_stack`** — `docker compose up` 构建并启动容器
   - 容器内启动 Webots（3D 仿真 GUI）
   - 等待 Webots 就绪后启动 Nav2（导航栈）
   - 等待 Nav2 就绪后启动 rviz2（地图可视化）
   - 最后启动 `tiago_bridge`（MCP 服务，向 robonix-server 注册 `mcp_tools` 接口）
5. **启动 robonix-agent** — 注册自身，发现 VLM 和 MCP 工具，进入交互模式

## 节点注册与发现

| 节点 | node_id | namespace | 接口 | 传输 |
|------|---------|-----------|------|------|
| VLM 服务 | `com.robonix.services.vlm` | `robonix/sys/model/vlm` | `chat` | gRPC |
| Tiago 桥接 | `com.robonix.prm.tiago` | `robonix/prm/tiago` | `mcp_tools` | MCP |
| Agent | `com.robonix.runtime.agent` | `robonix/sys/runtime/agent` | — | — |

Agent 的发现逻辑：

- VLM：`QueryNodes { abstract_interface_id: "robonix/sys/model/vlm/chat", transport: "grpc" }` → `NegotiateChannel` → gRPC 端点
- MCP 工具：`QueryNodes { transport: "mcp" }` → 从 `metadata_json.endpoint` 连接 MCP 服务，获取可用工具列表

## MCP 工具（Tiago 桥接）

Agent 通过 MCP 协议调用以下工具控制机器人：

| 工具 | 功能 |
|------|------|
| `get_robot_pose` | 获取机器人当前位姿 |
| `send_nav_goal` | 发送导航目标（x, y, yaw） |
| `cancel_nav` | 取消当前导航 |
| `get_camera_image` | 获取 RGB 相机图像（base64） |

## 交互示例

Agent 启动后等待 stdin 输入。输入自然语言指令：

```
> navigate to position x=2.0 y=1.0
```

Agent 的 ReAct 循环：

1. **Thought** — VLM 分析指令，决定调用 `send_nav_goal`
2. **Action** — 调用 MCP 工具 `send_nav_goal(x=2.0, y=1.0, yaw=0.0)`
3. **Observation** — 工具返回导航结果
4. **Thought** — VLM 判断任务完成，返回最终回答

## 故障排除

### Webots GUI 不显示

确认 `DISPLAY` 环境变量已设置，并允许 Docker 访问 X11：

```bash
xhost +local:docker
```

### Webots 非常卡顿

通常是缺少 GPU 透传。确认：

1. `nvidia-smi` 命令可执行
2. 已安装 `nvidia-container-toolkit`
3. `compose.gpu.yaml` 会被自动合并（`robonix_manifest.yaml` 的 `node.start` 脚本会检测 nvidia-smi）

### MCP 工具调用失败

检查 `tiago_bridge` 是否已注册。运行：

```bash
cargo run -p robonix-cli -- tools --server 127.0.0.1:50051
```

应看到 MCP 工具列表。如果为空，可能是容器尚未完全启动（Webots + Nav2 启动需要约 40 秒）。
