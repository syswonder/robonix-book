# ManiSkill3 仿真 Demo

本节介绍如何运行 ManiSkill3 仿真 Demo。该 Demo 使用 Fetch 机器人在 ReplicaCAD 室内场景中执行语言指令驱动的操作任务，集成 GroundingDINO 目标检测、Rerun 3D 可视化及可选的 Octo 策略模型。

与 Tiago 仿真 Demo 不同，ManiSkill Demo 不依赖 Docker 或 ROS 2，全部在本地 Python 环境中运行。

## 系统架构

![ManiSkill3 Demo 架构](maniskill-arch.svg)

所有节点通过 `robonix-atlas` 控制平面注册与发现。节点间数据传输使用 gRPC（环境观测）和 MCP（工具调用）。用户通过 `rbnx chat` TUI 客户端与 Agent 交互。

## 前置条件

| 依赖 | 说明 |
|------|------|
| Linux x86_64 | 主要开发和测试平台 |
| Rust 工具链 (stable >= 1.85) | `rustup` 安装 |
| Python 3.11 | ManiSkill3 和 Octo 的依赖要求 |
| [uv](https://docs.astral.sh/uv/) | Python 包管理器 |
| NVIDIA GPU (>= 6 GB VRAM) | ManiSkill3 渲染 + GroundingDINO + Octo |
| X11 或 Wayland 桌面 | Rerun 可视化窗口 |
| VLM API 密钥 | Agent 的视觉语言推理后端 |

可选依赖：

| 依赖 | 说明 |
|------|------|
| — | `rbnx graph` 内置 PNG/SVG 渲染，无需 graphviz |
| `rerun-cli` | 独立 Rerun 桌面查看器 |

## 安装与设置

```bash
cd rust/examples/packages/maniskill_vla_demo

# 创建 Python 3.11 虚拟环境并安装所有依赖
./run.sh setup
```

`setup` 阶段会：

1. 通过 `uv` 安装 Python 3.11 并创建 `.venv`
2. 安装完整依赖栈（ManiSkill3、Octo、JAX、GroundingDINO、Rerun 等）
3. 下载 ReplicaCAD 场景资产（约 2 GB）
4. 生成 gRPC proto stubs

首次安装可能需要 10-20 分钟。

## 配置 VLM

```bash
cp ../../.env.example ../../.env
```

在 `rust/examples/.env` 中配置 VLM 后端：

```bash
VLM_API_KEY=sk-xxx
VLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
VLM_MODEL=qwen3-vl-plus
```

## 启动

```bash
./run.sh start
```

启动顺序：

1. `robonix-atlas` — 控制平面
2. `vlm_service` — VLM 服务（gRPC）
3. `env_node` — ManiSkill3 仿真环境（gRPC 数据接口）
4. `perception_node` — GroundingDINO 目标检测（MCP 工具）
5. `policy_node` — 策略节点（MCP 工具，Octo 或 scripted）
6. `viz_node` — Rerun 可视化（gRPC 数据服务器）+ Rerun 桌面查看器
7. `robonix-pilot` — 智能体（后台 gRPC 服务）

所有节点启动完成后，终端会打印：

```
============================================================
  All nodes running.  Use 'rbnx chat' to interact.

  rbnx chat --server 127.0.0.1:50051

  Or generate a topology graph:
  rbnx graph --server 127.0.0.1:50051 -o topology.png

  Press Ctrl+C to stop all nodes.
============================================================
```

## 交互

打开一个新终端，使用 `rbnx chat` 与 Agent 交互：

```bash
rbnx chat
```

这会启动一个终端 TUI 界面：

```
+--[ Robonix Agent ]---------------------------------+
|                                                    |
|  Connected to agent at http://localhost:XXXX.      |
|  Type a message and press Enter.                   |
|                                                    |
|  You: pick up the red cup                          |
|                                                    |
|  > [r1] detect_objects({"text_prompt":"red cup"})  |
|    = detect_objects -> [{"label":"red cup",...}]   |
|  > [r2] execute_instruction({"instruction":...})   |
|    = execute_instruction -> {"status":"done",...}  |
|  Agent: I've successfully picked up the red cup.   |
|                                                    |
+----------------------------------------------------+
| > Type message (Enter to send, Ctrl+C quit)        |
+----------------------------------------------------+
```

TUI 特性：

- 滚动消息历史（PageUp/PageDown）
- 实时显示工具调用及结果
- 颜色区分角色（用户/Agent/工具调用/工具结果/状态）
- Agent 日志输出不会干扰交互界面

### 示例指令

```
pick up the red cup
move forward
navigate to the kitchen
look around and describe what you see
```

## 拓扑可视化

生成当前系统节点拓扑的 PNG 图像：

```bash
rbnx graph -o topology.png
```

支持的输出格式：

```bash
rbnx graph -o topology.png --format png   # 默认：内置 SVG → PNG
rbnx graph -o topology.svg --format svg   # 矢量 SVG
```

图中每个节点显示为一个方框，标注节点 ID、接口名称、传输类型和端口号。已协商的通道显示为节点间的有向边。

## 节点注册信息

启动完成后系统中注册的节点：

| 节点 | node_id | namespace | 接口 | 传输 |
|------|---------|-----------|------|------|
| ManiSkill 环境 | `com.robonix.demo.maniskill` | `robonix/prm/manipulation` | `mcp_tools`, `env_data` | MCP, gRPC |
| 目标检测 | `com.robonix.demo.perception` | `robonix/prm/perception` | `mcp_tools` | MCP |
| 策略节点 | `com.robonix.demo.vla` | `robonix/prm/manipulation` | `mcp_tools` | MCP |
| VLM 服务 | `com.robonix.services.vlm` | `robonix/srv/cognition` | `reason` | gRPC |
| 可视化 | `com.robonix.demo.viz` | `robonix/viz` | — | — |
| Pilot | `com.robonix.runtime.pilot` | `robonix/srv/runtime/pilot` | `pilot` | gRPC |

## MCP 工具

Agent 通过 MCP 协议调用以下工具：

| 工具 | 来源节点 | 说明 |
|------|---------|------|
| `execute_instruction` | policy_node | 闭环执行语言指令（策略模型 → 环境步进循环） |
| `move_base` | policy_node | 直接控制底盘移动 |
| `detect_objects` | perception_node | GroundingDINO 目标检测 |
| `step_action` | env_node | 低级环境步进 |
| `get_obs` | env_node | 获取当前观测 |

`execute_instruction` 是 Agent 的主要操作接口。它接收一条自然语言指令，内部运行闭环控制循环：

1. 获取当前观测（RGB 图像、本体感知）
2. 运行策略模型预测动作
3. 在仿真环境中执行动作
4. 检查任务是否完成
5. 重复直到完成或超时

这一"技能调用"模式使 Agent 仅需一次工具调用即可完成一个完整的操作动作，VLM 无需逐步微操。

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `VLA_POLICY` | `octo` | 策略选择：`octo`（Octo 模型）或 `scripted`（脚本策略） |
| `MANISKILL_ENV_ID` | `ReplicaCADTidyHouseTrain_SceneManipulation-v1` | ManiSkill3 环境 ID |
| `MANISKILL_CAM_W` / `_CAM_H` | `640` / `480` | 相机分辨率 |
| `HF_ENDPOINT` | `https://hf-mirror.com` | HuggingFace 镜像 |
| `START_VLM_SERVICE` | `1` | 是否启动 VLM 服务 |
| `START_AGENT` | `1` | 是否启动 Agent |
| `START_VIZ` | `1` | 是否启动 Rerun 可视化 |
| `VIZ_DETECT_QUERY` | `object . cup . box` | 检测面板的默认查询词 |
| `VIZ_DETECT_DEVICE` | `auto` | viz 检测线程设备：`auto`/`cuda`/`cpu` |
| `VIZ_DETECT_FP16` | `1` | viz 检测线程使用 FP16 |
| `VIZ_NO_DETECT` | `0` | 设为 `1` 禁用 viz 内置检测（节省显存） |
| `VIZ_FPS` | `6` | 可视化帧率 |
| `DEMO_MEMORY_PROFILE` | `balanced` | `low` 降低分辨率节省显存 |
| `PERCEPTION_DEVICE` | `auto` | 检测模型设备：`auto`/`cuda`/`cpu` |
| `PERCEPTION_FP16` | `1` | 在 CUDA 上使用 FP16 节省显存 |

## Rerun 可视化

viz_node 提供四个可视化面板：

- **Robot Camera** — 实时 RGB 相机画面
- **Depth** — 深度图
- **Joint Positions** — 关节位置时间序列
- **Detections** — 目标检测叠加（来自 perception_node）

检测面板始终显示实时相机画面作为背景。viz_node 内部运行后台检测线程，持续对最新相机帧执行 GroundingDINO 推理，检测框在主循环中实时更新至 Rerun。检测线程独立于渲染帧率运行，GPU 处理越快则检测结果更新越及时。可通过 `VIZ_NO_DETECT=1` 禁用检测以节省显存。

## 故障排除

**Rerun 窗口不显示**：确认 `DISPLAY` 或 `WAYLAND_DISPLAY` 环境变量已设置。`run.sh` 会自动检测，但通过 SSH 运行时需手动配置 X11 转发。

**显存不足**：设置 `DEMO_MEMORY_PROFILE=low` 将相机分辨率降至 512x384。亦可用 `VLA_POLICY=scripted` 跳过 Octo 模型加载（节省约 2 GB）。

**GroundingDINO dtype 错误**：perception_node 会自动检测 FP16 不兼容并回退至 FP32。若仍失败，设置 `PERCEPTION_FP16=0`。

**Octo 安装失败**：确认使用 Python 3.11。Octo 依赖特定版本的 JAX (0.4.20)、TensorFlow (2.15) 和 transformers (<5.0)，`pyproject.toml` 中已固定这些约束。

**环境资产下载失败**：若 HuggingFace 不可访问，设置 `HF_ENDPOINT` 指向镜像站。亦可使用 `MANISKILL_ENV_ID=PickCube-v1` 运行无需额外资产的简单环境。
