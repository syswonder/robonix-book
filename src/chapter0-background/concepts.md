# 核心概念与 Robonix 实现对照

本节将 EAIOS 白皮书中定义的核心概念逐一对应到 Robonix 的具体实现，帮助开发者理解"概念从哪来"、"代码在哪里"。

## 概念对照表

### 行动抽象体系

| EAIOS 概念 | 含义 | Robonix 实现 | 关键代码/文档 |
|------------|------|-------------|-------------|
| 原语（Primitive） | 硬件能力的标准化映射 | 节点以 `kind: "primitive"` 注册到 `robonix-server`，声明接口并支持多种传输 | Tiago 桥接：`rust/examples/packages/tiago_sim_stack/tiago_bridge/`；接口定义：[抽象硬件原语](../chapter3-developer-guide/primitives/index.md) |
| 服务（Service） | 标准化算法能力（感知、规划等） | 节点以 `kind: "service"` 注册，提供纯算法能力 | VLM 服务：`rust/examples/packages/vlm_service/` |
| 技能（Skill） | 可复用的组合操作 | 节点以 `kind: "skill"` 注册，附带 `SKILL.md` 描述文件供 Agent 发现和调用 | 格式规范：计划中 |
| 任务（Task） | 用户层目标 | `robonix-agent`（Rust）实现 ReAct 循环：接收自然语言指令 → VLM 推理 → 调用技能/服务 → 观察结果 → 迭代 | 代码：`rust/crates/robonix-agent/src/react.rs` |

### 三大空间

| EAIOS 概念 | 含义 | Robonix 实现 | 关键代码/文档 |
|------------|------|-------------|-------------|
| 感知空间 | 环境与本体的数字化重构 | 传感器原语（Camera、Lidar、IMU 等）通过多传输通道采集数据；`robonix-server` 管理 channel 分配 | 原语定义：[camera](../chapter3-developer-guide/primitives/camera.md)、[sensor](../chapter3-developer-guide/primitives/sensor.md)；多传输：[多通道传输](../chapter3-developer-guide/multi-transport.md) |
| 认知空间 | 规划与决策 | VLM/LLM 作为系统级服务提供推理能力；`robonix-agent` 的 ReAct 循环执行多步推理；MCP 端点暴露工具供 Agent 调用 | Agent：`rust/crates/robonix-agent/` |
| 动作空间 | 行动执行 | 底盘、机械臂、夹爪等原语接收控制命令执行物理动作；通过 `NegotiateChannel` 协商控制通道 | 原语定义：[base](../chapter3-developer-guide/primitives/base.md)、[arm](../chapter3-developer-guide/primitives/arm.md)、[gripper](../chapter3-developer-guide/primitives/gripper.md) |

### 系统机制

| EAIOS 概念 | 含义 | Robonix 实现 | 关键代码/文档 |
|------------|------|-------------|-------------|
| 控制平面 | 服务注册、发现、channel 协商 | `robonix-server`：gRPC API 提供 `RegisterNode`、`DeclareInterface`、`NegotiateChannel`、`QuerySkillMd` 等 RPC | 代码：`rust/crates/robonix-server/`；契约：[命名空间与接口契约](../chapter3-developer-guide/namespace-contracts.md) |
| 多传输支持 | ROS2、gRPC、MCP、共享内存等 | 节点声明支持的传输类型，协商后分配端点 | [多通道传输](../chapter3-developer-guide/multi-transport.md) |
| 软硬解耦 | 硬件与算法独立演进 | 原语提供标准化硬件抽象；`ridlc` 从 ROS IDL 生成代码 | `rust/crates/ridlc/` |
| 世界模型 | 物理执行前的虚拟推演 | 当前原型使用 Webots 仿真环境验证端到端流程（见 [Tiago E2E Demo](../chapter1-getting-started/tiago-e2e-demo.md)） | `rust/examples/packages/tiago_sim_stack/` |
| 安全内核 | 安全约束与执行屏障 | 计划通过 syswonder 生态（hvisor + RuxOS）实现硬件级隔离与安全保障；当前版本尚未集成 | 参见 [syswonder.org](https://syswonder.org) |
| SKILL.md | 每个能力单元的 LLM 可读描述 | 每个 primitive/service/skill 附带 Markdown 格式的 SKILL.md，包含描述、参数 JSON Schema、使用示例 | 格式规范：计划中 |

## 端到端数据流

以 Tiago Webots 仿真任务为例：

1. 用户向 `robonix-agent` 发出自然语言指令（如"导航到 x=2, y=1 位置"）
2. Agent 通过 `robonix-server` 发现可用服务（VLM、Tiago MCP 工具）
3. Agent 调用 VLM 进行推理，确定执行方案（调用 `send_nav_goal`）
4. Agent 通过 MCP 协议调用 `tiago_bridge` 的 `send_nav_goal` 工具
5. 桥接内部将 MCP 调用转为 ROS 2 Nav2 action 发送给仿真环境
6. 执行结果回传至 Agent 作为 Observation
7. Agent 迭代执行直至任务完成

这一流程涵盖了 EAIOS 定义的四级抽象（原语→服务→技能→任务）和三个空间（感知→认知→动作）。
