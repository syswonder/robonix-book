# 服务（`robonix/srv`）

`robonix/srv/` 命名空间下注册的是 Robonix 提供的各类服务，与用户部署的原语（`robonix/prm`）和技能（`robonix/skill`）区分。已入库契约均在 `rust/contracts/sys/*.v1.toml`（完整树见 [接口目录首页 · 契约源码路径](../index.md#contract-toml-sources)）。

## 系统组件

Liaison、Pilot、Executor 是 Robonix 系统自身的组成部分，不可替换，共同构成从用户意图到工具执行的完整编排管线。这三个进程同样通过 Atlas 注册与发现彼此，使用与其他节点相同的 RegisterNode / DeclareInterface 机制，但其身份为系统骨架而非可替换服务。

| 组件 | 职责 | 文档 |
|------|------|------|
| Liaison | 交互入口：界面适配、语音服务调用、用户区分 | [Liaison](liaison.md) |
| Pilot | 推理引擎：意图理解、ReAct 推理循环、会话管理 | [Pilot](pilot.md) |
| Executor | 执行引擎：Skill Engine（工具调用 / RTDL 分发）、异常上报 | [Executor](executor.md) |

## 默认服务 vs 场景服务

Robonix 的可替换服务分为两类：

- 默认服务：每个 Robonix 部署均应具备的基础服务，提供跨场景通用能力，涵盖认知大模型（Cognition）、ASR、TTS、记忆、地图与定位、数据采集、系统监控等。
- 场景服务节点：面向具体部署的垂直能力（如工厂条码识别、家庭物品识别等），按部署私域命名空间组织，不占用 `robonix/srv/` 路径。

## 默认服务契约目录

### Cognition（认知）

| 契约 ID（`contract_id`） | 契约源码 | 文档 |
|--------------------------|----------|------|
| `robonix/srv/cognition/reason` | `rust/contracts/sys/vlm_chat.v1.toml` | [Cognition · Reason](cognition-reason.md) |
| `robonix/srv/cognition/world`（规划中） | — | — |
| `robonix/srv/cognition/code`（规划中） | — | — |

### Common（通用服务）

| 契约 ID（`contract_id`） | 契约源码 | 文档 |
|--------------------------|----------|------|
| `robonix/srv/memory/search` | `rust/contracts/sys/memory_search.v1.toml` | [Memory Search](memory-search.md) |
| `robonix/srv/memory/save` | `rust/contracts/sys/memory_save.v1.toml` | [Memory Search](memory-search.md) |
| `robonix/srv/memory/compact` | `rust/contracts/sys/memory_compact.v1.toml` | [Memory Search](memory-search.md) |

### Planning（规划，预留）

| 路径前缀 | 用途 |
|---------|------|
| `robonix/srv/planning/` | 路径 / 任务规划 |

### 其它预留命名空间

| 路径前缀 | 用途 |
|---------|------|
| `robonix/srv/common/map/` | 语义地图与定位 |
| `robonix/srv/common/data_collection/` | 数据采集（LeRobot 对接落点） |
| `robonix/srv/common/monitor/` | 系统监控 |
| `robonix/srv/debug/` | 调试工具 |

## 认知层的多角色设计（规划）

Robonix 的认知层并非单体 VLM，而是由多个分工模型分别承担不同的认知角色：

- 推理模型（reason）：接收感知数据与用户意图，进行 CoT 推理与逻辑分析
- 世界模型（world）：预测环境状态变化，辅助任务规划
- 代码模型（code）：生成结构化的 RTDL/TaskGraph 执行计划

每个角色对应一条独立的 `robonix/srv/cognition/*` 契约，Pilot 在不同场景下按角色调用不同后端。当前最简实现为单个 VLM 服务（`robonix/srv/cognition/reason`）承担全部推理工作，后续按需拆出独立的 world 和 code 契约。

## 数据采集与系统监控（规划中的默认服务）

- 数据采集服务（`robonix/srv/common/data_collection`）：Robonix 与训练框架的对接点，计划以 [LeRobot](https://github.com/huggingface/lerobot)（Hugging Face）格式沉淀机器人运行轨迹与多模态观测，使机器人可直接作为 LeRobot 数据源。
- 系统监控服务（`robonix/srv/common/monitor`）：聚合 Pilot / Executor / 各 Provider 的运行状态、资源使用与异常事件，供调试、自动恢复及运维使用。

两者均处于规划阶段。

## 新增服务

1. 在 `rust/crates/robonix-interfaces/lib/` 增加 IDL（若走 robonix-codegen）。
2. 在 `rust/contracts/sys/` 增加 `*.v1.toml`，`[contract] id` 即 `contract_id`。
3. 运行 `robonix-codegen --contracts` 更新 `robonix_proto/`（及 `robonix_contracts.proto`）。
4. 将契约 ID 加入 Atlas `ROBO_SYSTEM_INTERFACE_CATALOG`（`grpc`/`ros2` 校验）。
5. 在 `service/` 下增加一页，并把本表与侧栏 `SUMMARY.md` 链好。
