# 核心概念与 Robonix 实现对照

本节将 EAIOS 白皮书中定义的核心概念逐一对应到 Robonix 的具体实现，帮助开发者理解"概念从哪来"、"代码在哪里"。

## 概念对照表

### 行动抽象体系

| EAIOS 概念 | 含义 | Robonix 实现 | 关键代码/文档 |
|------------|------|-------------|-------------|
| 原语（Primitive） | 硬件能力的标准化映射 | 节点以 `kind: "primitive"` 注册到 `robonix-server`，声明接口并支持多种传输 | 示例节点：`rust/examples/nodes/sim_env.py`；接口定义：[抽象硬件原语](../chapter3-developer-guide/primitives/index.md) |
| 服务（Service） | 标准化算法能力（感知、规划等） | 节点以 `kind: "service"` 注册，提供纯算法能力 | 示例：`rust/examples/packages/vlm_service/` |
| 技能（Skill） | 可复用的组合操作 | 节点以 `kind: "skill"` 注册，附带 `SKILL.md` 描述文件供 Agent 发现和调用；技能内部编排原语和服务，可隐藏内部细节 | 格式规范：[RFC005](../rfc/RFC005-SKILL-Format.md)；示例：`rust/examples/skills/*.md` |
| 任务（Task） | 用户层目标 | `robonix-agent`（Rust）实现 ReAct 循环：接收自然语言指令 → VLM 推理 → 调用技能/服务 → 观察结果 → 迭代 | 代码：`rust/crates/robonix-agent/src/react.rs` |

### 三大空间

| EAIOS 概念 | 含义 | Robonix 实现 | 关键代码/文档 |
|------------|------|-------------|-------------|
| 感知空间 | 环境与本体的数字化重构 | 传感器原语（Camera、Lidar、IMU 等）通过多传输通道采集数据；`robonix-server` 管理 channel 分配 | 原语定义：[camera](../chapter3-developer-guide/primitives/camera.md)、[sensor](../chapter3-developer-guide/primitives/sensor.md)；多传输：[RFC006](../rfc/RFC006-Multi-Transport.md) |
| 认知空间 | 规划与决策 | VLM/LLM 作为系统级服务提供推理能力；`robonix-agent` 的 ReAct 循环执行多步推理；MCP 端点暴露技能供外部 Agent 调用 | Agent：`rust/crates/robonix-agent/`；MCP：[RFC004](../rfc/RFC004-MCP-Integration.md) |
| 动作空间 | 行动执行 | 底盘、机械臂、夹爪等原语接收控制命令执行物理动作；通过 `NegotiateChannel` 协商控制通道 | 原语定义：[base](../chapter3-developer-guide/primitives/base.md)、[arm](../chapter3-developer-guide/primitives/arm.md)、[gripper](../chapter3-developer-guide/primitives/gripper.md) |

### 系统机制

| EAIOS 概念 | 含义 | Robonix 实现 | 关键代码/文档 |
|------------|------|-------------|-------------|
| 控制平面 | 服务注册、发现、channel 协商 | `robonix-server`：gRPC API 提供 `RegisterNode`、`DeclareInterface`、`NegotiateChannel`、`QuerySkillMd` 等 RPC | 代码：`rust/crates/robonix-server/`；设计：[RFC003](../rfc/RFC003-Control-Plane.md)；**注册表路径与数据面 `.proto` 的对应**：[控制平面与 Proto 契约](../chapter3-developer-guide/namespace-and-proto-contracts.md) |
| 多传输支持 | ROS2、gRPC、MCP、共享内存等 | 节点声明支持的传输类型，协商后分配端点 | [RFC006](../rfc/RFC006-Multi-Transport.md) |
| 软硬解耦 | 硬件与算法独立演进 | 原语提供标准化硬件抽象；`ridlc` 从 ROS IDL 生成代码 | `rust/crates/ridlc/` |
| 世界模型 | 物理执行前的虚拟推演 | 当前原型使用 LIBERO/MuJoCo 仿真环境验证端到端流程（详见[快速上手](../chapter1-getting-started/quickstart.md)）；未来计划接入 NVIDIA Isaac Sim | `rust/examples/nodes/sim_env.py` |
| 安全内核 | 安全约束与执行屏障 | 计划通过 syswonder 生态（hvisor + RuxOS）实现硬件级隔离与安全保障；当前版本尚未集成 | 参见 [syswonder.org](https://syswonder.org) |
| 对象（Object） | 外部世界实体的虚拟表征 | 当前仿真环境中的物体由 LIBERO 管理；后续计划扩展为完整的场景图表征 | — |
| SKILL.md | 每个能力单元的 LLM 可读描述 | 每个 primitive/service/skill 附带 Markdown 格式的 SKILL.md，包含描述、参数 JSON Schema、使用示例，与 MCP `inputSchema` 兼容 | 格式规范：[RFC005](../rfc/RFC005-SKILL-Format.md)；示例：`rust/examples/skills/*.md` |

## 端到端数据流

一个典型的仿真任务执行流程如下：

1. 用户向 `robonix-agent` 发出自然语言指令（如"pick up the red mug"）
2. Agent 通过 `robonix-server` 发现可用服务（VLM、VLA、sim_env）
3. Agent 调用 VLM 进行推理，确定执行方案
4. Agent 调用 `vla_service.execute_task`，VLA 服务内部协商 `sim_env` channel
5. VLA 服务循环执行：获取仿真观测 → SmolVLA 推理 → 执行动作 → 检查完成
6. 执行结果回传至 Agent 作为 Observation
7. Agent 迭代执行直至任务完成

这一流程涵盖了 EAIOS 定义的全部四级抽象（原语→服务→技能→任务）和三个空间（感知→认知→动作）。
