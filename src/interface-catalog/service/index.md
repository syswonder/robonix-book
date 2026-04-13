# 服务（`robonix/srv`）

Robonix 的"服务"分两类：

| 类别 | 是什么 | 谁实现 |
|---|---|---|
| **系统服务** | Robonix 操作系统本体的一部分，构成"用户意图 → 工具执行"的核心管线 | Robonix 项目本身，不可替换 |
| **用户服务** | 部署到 Robonix 之上、用 contract 暴露给 Agent 的能力 | 由用户部署。其中 VLM、memory、SLAM、navigation、perception 等通用能力 Robonix 提供**默认实现**；其它用户场景自行实现（可复用现有 contract，或自定义新 contract 后实现） |

已入库契约均在 `rust/contracts/srv/*.v1.toml`（完整树见 [接口目录首页 · 契约源码路径](../index.md#contract-toml-sources)）。

## 系统服务

Liaison、Pilot、Executor 是 Robonix 操作系统骨架。它们通过 Atlas 互相发现，但其身份是系统组成部分而非可替换组件。

| 系统服务 | 职责 | 文档 |
|------|------|------|
| Liaison | 交互入口：界面适配、语音服务调用、用户区分 | [Liaison](liaison.md) |
| Pilot | 推理引擎：意图理解、ReAct 推理循环、会话管理 | [Pilot](pilot.md) |
| Executor | 执行引擎：Skill Engine（工具调用 / RTDL 分发）、异常上报 | [Executor](executor.md) |

> 系统服务本身**强制依赖**部分用户服务才能完成功能——例如 Pilot 必须有可用的 `srv/cognition/reason`（VLM）才能推理，Liaison 通常依赖 ASR/TTS。Robonix 默认实现这部分依赖（默认 VLM provider、默认 memsearch 等），但允许用户替换。

## 用户服务

下面列出的契约都是**用户服务**——Robonix 提供契约定义和 / 或默认实现，但实际是部署到系统之上的，可以替换或扩展。

### 命名约定

- **通用基础能力**：Robonix 出默认实现，命名固定（如 `robonix/srv/cognition/reason`、`robonix/srv/memory/search`、`robonix/srv/slam/*`）
- **特定场景服务**：面向具体部署的垂直能力（如工厂条码识别、家庭物品识别），按部署私域命名空间组织，**不占用** `robonix/srv/` 路径

## 用户服务契约目录

### Cognition（认知）

| 契约 ID（`contract_id`） | 契约源码 | 文档 |
|--------------------------|----------|------|
| `robonix/srv/cognition/reason` | `rust/contracts/srv/vlm_chat.v1.toml` | [Cognition · Reason](cognition-reason.md) |
| `robonix/srv/cognition/world`（TODO） | — | — |
| `robonix/srv/cognition/code`（TODO） | — | — |

### Memory（记忆）

| 契约 ID（`contract_id`） | 契约源码 | 文档 |
|--------------------------|----------|------|
| `robonix/srv/memory/search` | `rust/contracts/srv/memory_search.v1.toml` | [Memory Search](memory-search.md) |
| `robonix/srv/memory/save` | `rust/contracts/srv/memory_save.v1.toml` | [Memory Search](memory-search.md) |
| `robonix/srv/memory/compact` | `rust/contracts/srv/memory_compact.v1.toml` | [Memory Search](memory-search.md) |

### SLAM（定位与建图）

参考实现：[`enkerewpo/mapping_rbnx`](https://github.com/enkerewpo/mapping_rbnx)（基于 FASTLIO2_ROS2 的 LiDAR-Inertial SLAM + PGO 回环 + ICP 重定位 + HBA 全局优化）。详细论证见 [SLAM 服务](slam.md)。

| 契约 ID（`contract_id`） | 契约源码 | 用途 |
|--------------------------|----------|------|
| `robonix/srv/slam/status` | `rust/contracts/srv/slam_status.v1.toml` | 查询运行模式 / 里程计频率 / 地图文件 |
| `robonix/srv/slam/save_map` | `rust/contracts/srv/slam_save_map.v1.toml` | 持久化当前 3D 点云地图 |
| `robonix/srv/slam/load_map` | `rust/contracts/srv/slam_load_map.v1.toml` | 加载已有地图并切到重定位模式 |
| `robonix/srv/slam/switch_mode` | `rust/contracts/srv/slam_switch_mode.v1.toml` | mapping / localization / idle 切换 |
| `robonix/srv/slam/set_initial_pose` | `rust/contracts/srv/slam_set_initial_pose.v1.toml` | 重定位初始位姿提示（ICP 种子） |

### Common Map（通用地图数据面）

从 SLAM 得到的空间表示，以契约形式暴露给下游（导航栈、可视化、任务规划）。

| 契约 ID（`contract_id`） | 契约源码 | 载荷 | 用途 |
|--------------------------|----------|------|------|
| `robonix/srv/common/map/pointcloud` | `rust/contracts/srv/map_pointcloud.v1.toml` | `sensor_msgs/PointCloud2` | 世界坐标系注册 3D 点云（实时流） |
| `robonix/srv/common/map/occupancy_grid` | `rust/contracts/srv/map_occupancy_grid.v1.toml` | `nav_msgs/OccupancyGrid` | 2D 栅格地图（Nav2 `static_layer` 直连） |
| `robonix/srv/common/map/scan_2d` | `rust/contracts/srv/map_scan_2d.v1.toml` | `sensor_msgs/LaserScan` | 3D 点云投影 2D scan（Nav2 `obstacle_layer` 直连） |

### Navigation（导航）

参考实现：Nav2 + `tiago_sim_stack` 的 bringup 配置。详细论证见 [导航服务](navigation.md)。

| 契约 ID（`contract_id`） | 契约源码 | 载荷 | 用途 |
|--------------------------|----------|------|------|
| `robonix/srv/navigation/navigate` | `rust/contracts/srv/navigation_navigate.v1.toml` | `geometry_msgs/PoseStamped` → `std_msgs/String` | 发送导航目标 |
| `robonix/srv/navigation/status` | `rust/contracts/srv/navigation_status.v1.toml` | `std_msgs/String` → `std_msgs/String` | 查询目标状态 |
| `robonix/srv/navigation/cancel` | `rust/contracts/srv/navigation_cancel.v1.toml` | `std_msgs/String` → `std_msgs/String` | 取消目标 |

### Perception（感知）

| 契约 ID（`contract_id`） | 契约源码 | 载荷 | 用途 |
|--------------------------|----------|------|------|
| `robonix/srv/perception/detect` | `rust/contracts/srv/perception_detect.v1.toml` | `sensor_msgs/Image` → `std_msgs/String`（JSON） | 单帧目标检测 / 场景理解 |

### 其它预留命名空间

| 路径前缀 | 用途 | 状态 |
|---------|------|------|
| `robonix/srv/planning/` | 路径 / 任务规划 | TODO |
| `robonix/srv/common/data_collection/` | 数据采集（LeRobot 对接落点） | TODO |
| `robonix/srv/common/monitor/` | 系统监控 | TODO |
| `robonix/srv/debug/` | 调试工具 | TODO |

## 认知层的多角色设计（TODO）

Robonix 的认知层并非单体 VLM，而是由多个分工模型分别承担不同的认知角色：

- 推理模型（reason）：接收感知数据与用户意图，进行 CoT 推理与逻辑分析
- 世界模型（world）：预测环境状态变化，辅助任务规划
- 代码模型（code）：生成结构化的 RTDL/TaskGraph 执行计划

每个角色对应一条独立的 `robonix/srv/cognition/*` 契约，Pilot 在不同场景下按角色调用不同后端。当前最简实现为单个 VLM 服务（`robonix/srv/cognition/reason`）承担全部推理工作，后续按需拆出独立的 world 和 code 契约。

## 数据采集与系统监控（TODO）

- 数据采集服务（`robonix/srv/common/data_collection`）：Robonix 与训练框架的对接点，计划以 [LeRobot](https://github.com/huggingface/lerobot)（Hugging Face）格式沉淀机器人运行轨迹与多模态观测，使机器人可直接作为 LeRobot 数据源。
- 系统监控服务（`robonix/srv/common/monitor`）：聚合 Pilot / Executor / 各 Provider 的运行状态、资源使用与异常事件，供调试、自动恢复及运维使用。

## 新增服务

1. 在 `rust/crates/robonix-interfaces/lib/` 增加 IDL（若走 robonix-codegen）。
2. 在 `rust/contracts/srv/` 增加 `*.v1.toml`，`[contract] id` 即 `contract_id`。
3. 运行 `robonix-codegen --contracts` 更新 `robonix_proto/`（及 `robonix_contracts.proto`）。
4. 将契约 ID 加入 Atlas `ROBO_SYSTEM_INTERFACE_CATALOG`（`grpc`/`ros2` 校验）。
5. 在 `service/` 下增加一页，并把本表与侧栏 `SUMMARY.md` 链好。
