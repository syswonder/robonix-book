# Robonix Framework（robonix-core）— 模块设计

<!-- toc -->

本文档说明 Robonix Framework（代码中为 `robonix-core`，后续可能更名为 robonix-server）的模块职责、对外接口与任务执行流程。如何运行该进程见 [用户手册 - robonix-server](../chapter2-user-guide/robonix-server.md)。

## 定位与职责

在 Robonix OS 中，ROS2 之上的这一层系统服务与规范由 `robonix-core` 实现，职责包括：

- 能力注册与查询：原语、标准服务、技能的注册表；外部经 ROS2 服务查询能力名与 Topic/服务名的映射。
- 任务提交与执行：接收自然语言任务，调用规划服务得到 RTDL，按指令调度技能（当前为向 `start_topic` 发布、订阅 `status_topic`）。
- 规范校验（Spec）：原语与服务注册时按内置 Spec 校验；技能无统一 Spec。

硬件厂商对接原语见 [原语开发指南](hardware-primitives-guide.md)，算法提供者对接标准服务见 [服务开发指南](service-development-guide.md)，技能开发者对接 Topic 协议见 [技能开发指南](skill-development-guide.md)。

## robonix-core 提供的接口

`robonix-core` 以 ROS2 节点形式运行，对外暴露以下四类 API（均为 ROS2 Service，服务名前缀 `/rbnx/`）：

| 类别 | 前缀 | 说明 |
|------|------|------|
| 原语 API | `/rbnx/prm/` | 原语注册、查询；供 CLI（经 daemon）注册，供服务/技能/执行器查询 Topic 映射 |
| 服务 API | `/rbnx/srv/` | 标准服务注册、查询；供 CLI 注册，供任务规划/执行时查找实际 service 地址 |
| 技能 API | `/rbnx/skl/` | 技能注册、查询；供 CLI 注册，供任务执行器解析技能名得到 `start_topic`、`status_topic` |
| 任务 API | `/rbnx/task/` | 任务提交、状态查询、结果获取、取消；供 CLI/Web 提交自然语言任务 |

具体服务名示例：

- 原语：`/rbnx/prm/register`、`/rbnx/prm/query`
- 服务：`/rbnx/srv/register`、`/rbnx/srv/query`
- 技能：`/rbnx/skl/register`、`/rbnx/skl/query`
- 任务：`/rbnx/task/submit`、`/rbnx/task/status`、`/rbnx/task/result`、`/rbnx/task/cancel`

消息类型与 srv 定义见 [robonix-sdk](robonix-sdk.md)。

## 规范校验（Spec）

- 原语与标准服务在注册时会被 robonix-core 校验：原语名、输入/输出参数名（及后续可扩展的消息类型）、服务名与 srv 类型必须与内置 Spec 一致。
- 技能无统一规范，由开发者自定义 `name`、`start_topic`、`status_topic`、`start_args` 等；执行器按技能名查实例，再得到 start/status topic 调用。

Spec 在代码中分布在：

- `robonix-core/src/perception/specs.rs`：感知相关原语与服务（如 `prm::camera.rgb`、`srv::semantic_map`）
- `robonix-core/src/cognition/specs.rs`：认知相关（如 `prm::base.pose.cov`、`srv::task_plan`）
- `robonix-core/src/action/specs.rs`：动作相关（如 `prm::base.move`、`prm::arm.move.ee`）

## 任务执行流程（简要）

1. 用户通过 CLI 或 Web 提交自然语言任务（调用 `/rbnx/task/submit`）。
2. 规划阶段：core 调用已注册的 `task_plan` 服务，将自然语言转为 RTDL（当前为 list 形式 JSON）。
3. 执行阶段：执行器按 RTDL 指令顺序执行；当前仅支持 `type: "skill"` 的指令：根据技能名查询技能库，得到 `start_topic`、`status_topic`，向 `start_topic` 发布 JSON（含 `skill_id`、`params`），并订阅 `status_topic` 等待 `state: "finished"` 或错误。
4. 技能内部可再通过 QueryPrimitive、QueryService 获取原语或服务的 Topic/服务名，与硬件或算法服务交互。

服务如何暴露（在 manifest 中填写 `entry`）以及技能如何被触发（`start_topic`、`status_topic`）详见 [服务开发指南](service-development-guide.md) 与 [技能开发指南](skill-development-guide.md)。运行方式、环境变量与安装步骤见 [robonix-server](../chapter2-user-guide/robonix-server.md) 与 [快速开始](../chapter1-getting-started/quickstart.md)。
