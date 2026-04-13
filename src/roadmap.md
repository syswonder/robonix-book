# Roadmap

Robonix 处于快速迭代的早期阶段。这里把**已经跑通**、**短期任务**、**更长期的规划**各自列出来，任务尽量具体化。

> 最后更新：2026-04

## ✅ 目前实现 / 可用

### 控制面
| 项 | 状态 |
|---|---|
| Atlas：`RegisterNode` / `DeclareInterface` / `QueryNodes` / `NegotiateChannel` / `NodeHeartbeat` | OK |
| 三种 transport 声明：gRPC、ROS 2、MCP | OK |
| `ROBO_SYSTEM_INTERFACE_CATALOG` 白名单（仅 warn-level，非强制） | OK |
| `robonix-codegen`：ROS IDL + TOML → `.proto` + Python `_mcp.py` dataclass | OK |
| 全局 source path 机制（`~/.robonix/config.yaml:robonix_source_path`，`rbnx setup` / `rbnx path` / `rbnx codegen`） | OK |
| `make install` 自动 `rbnx setup` | OK |

### 推理 / 执行
| 项 | 状态 |
|---|---|
| Pilot：VLM-driven ReAct 循环 | OK |
| Pilot：OpenAI 兼容 `tool_calls` → `TaskGraph`（每轮独立，非跨轮连接） | OK |
| Pilot：vlm 后端错误 graceful 处理（text_delta + finish=error，session 不死） | OK |
| Executor：Built-in / MCP / gRPC 工具路由 | OK |
| Liaison：文本 `Intent → PilotEvent` 流 | OK |
| `rbnx chat` TUI（Esc 中断当前 turn） | OK |

### 契约目录
| 命名空间 | 状态 |
|---|---|
| `prm/base/*`（cmd, move, odom, twist_in, stop, robot/state, joint_state, pose_cov, goal_status） | OK |
| `prm/camera/*`（rgb, depth, snapshot, depth_snapshot） | OK |
| `prm/sensor/*`（imu, lidar, lidar3d, lidar_snapshot） | OK |
| `srv/cognition/reason`（VLM chat） | OK |
| `srv/memory/{search,save,compact}`（memsearch_service） | OK |
| `srv/slam/*` + `srv/common/map/*`（mapping_rbnx / FASTLIO2_ROS2） | OK |
| `srv/navigation/*`（Nav2 via tiago_bridge） | OK |
| `srv/perception/detect`（YOLOE via maniskill_vla_demo） | OK |

### 示例与集成
| 项 | 状态 |
|---|---|
| Tiago Webots E2E（Nav2 + ROS 2 ↔ MCP 桥接，一键 `run.sh`） | OK |
| ManiSkill3 VLA demo（Fetch + ReplicaCAD + YOLOE + scripted policy） | OK |
| mapping_rbnx on Jetson Orin（Docker 镜像 + 预置 Webots asset cache） | OK |
| Tiago docker 镜像含 Webots asset seed（国内首次启动不需 GitHub 代理） | OK |
| `clawhub_skills`：从 OpenClaw 导入 Agent Skills | OK |

## 🚧 进行中 / 短期任务

### Contract & codegen
- **package-local contracts/IDL 自动 union**：现在 `rbnx codegen` 只看主仓 `rust/contracts` + `rust/crates/robonix-interfaces/lib`；maniskill 的 `contracts/`、`interfaces/lib/` 目录存在但要 build.sh 手动 staging 才能合入 codegen。目标：codegen 自动识别 `<pkg>/contracts` 和 `<pkg>/interfaces/lib`，无需 build.sh 写合并代码。
- **Atlas catalog 强校验**：目前 `unknown contract` 只 warn，生产上应支持 `CATALOG_STRICT=1` 模式。
- **版本化 contract ID**：当前 `[contract] version` 字段只在 TOML 里，实际广播的是裸 id，没做版本协商。

### Pilot / Executor
- **TaskGraph / RTDL 结构化输出**：目前 VLM 是 OpenAI `tool_calls` list，拿不到顺序、分支、循环。目标：VLM 按 Robonix 定义的 `TaskGraph` / RTDL schema 返回（支持序列、并行、条件分支、BT 子结构），Executor 按图语义执行。
- **VLM 失败重试 / 退避**：目前 vlm 出错会把错误文本当成 assistant reply 进 history，下一轮照样发。有时需要智能的退避（例如同样的 pipe 连错 N 次就停）。
- **turn-level 超时**：长 tool call 卡住时用户只能按 Esc。目标：Executor 级超时 + `abort_goal` 自动派发。
- **Pilot 内部 model router**：按 intent 类型 / capability 需求从 Atlas 候选中选不同 provider（例如视觉走本地 VL 模型、代码生成走远端大模型），目前全靠 `vlm_service` 硬编码单后端。配合 `model` transport 落地。

### 部署 / 运维
- **系统服务 lifecycle**：VLM、memsearch、SLAM 这些"每次部署都该起来"的服务目前靠 run.sh 启动，没有 systemd unit / 声明式 orchestrator。目标：`rbnx deploy`（或 systemd generator）按 manifest 启动并监控。
- **package GC**：`~/.robonix/packages/` 装的包没有 uninstall 命令。
- **分布式部署**：目前大多数测试都在单机。远程 Atlas + 本地 provider 的场景要补文档 + 实测。
- **robot host rebuild**：小车上 mapping_rbnx 还没在真实 LiDAR 上端到端验证（已上 build，等传感器可用）。

### 文档
- `docs/src/interface-catalog/service/` 对 cognition-reason / executor / liaison / pilot / memory-search 几页还要照 slam / navigation 的风格补齐"为什么这些接口充分"的论证。
- 把"新增 package"的 end-to-end 流程（从 manifest + contracts + skills 到 `rbnx start`）整合成一页。

## 🔭 更长期 / 未排期

- **`model` transport（草稿设计见 [architecture/model-transport.md](architecture/model-transport.md)）**：
  - Atlas `DeclareInterface` 扩展 `metadata_json`（flavor / url / model / auth / capabilities / context_len / cost / latency_ms）
  - Pilot 增加 flavor-specific HTTP 客户端适配器：`openai` / `anthropic` / `gemini` / `ollama`
  - Pilot 路由器：capability filter + 策略插件（explicit / round_robin / cheapest_first / fastest_first）
  - 拆分 cognition 契约：`reason` / `world` / `code` / `embed` / `rerank` / `tts` / `asr`
  - `vlm_service` 退路：保留作为"把裸 OpenAI endpoint 伪装成 model provider"的适配器，不再强制经过 gRPC
- **`srv/common/data_collection`（LeRobot 集成）**：Pilot/Executor 运行期观测（obs / action / reward）落盘为 [LeRobot dataset](https://github.com/huggingface/lerobot) 格式；支持推送到 Hugging Face Hub；与现有训练 pipeline 无缝对接——把机器人变成数据源。
- **`srv/common/map/semantic`**：语义地图 —— 物体级、房间级标注。SLAM 输出几何，加 `perception/detect` 结果 + 持久化 → 语义。
- **`srv/common/data_collection`**：运行时观测沉淀到 [LeRobot](https://github.com/huggingface/lerobot) 格式；机器人变成数据源。
- **`srv/common/monitor`**：全局运行监控面板（Pilot / Executor / 所有 provider 的资源、吞吐、错误率）。
- **共享内存 transport 生产化**：`robonix-buffer` + shm transport 在 `zero_copy_demo` 里跑通过 raw SHM，但 provider/consumer 的 view 交换语义、iceoryx2 正式集成还没做。
- **多机 SLAM / 多机协作**：mapping_rbnx 只管单机；多机融合、多机任务分发是独立课题。
- **安全与权限**：当前 Atlas 是 trust-on-first-use。tool call 的 ACL、资源配额、多租户均未设计。
- **持久化 Skill Graph**：把经过验证的 TaskGraph 固化成命名 skill，Pilot 直接下发而无需再走 VLM 推理一次。
- **Skill 运行时**：把 skill 从"喂给 VLM 的文本"提升成可缓存、可调度、可隔离、可审计的 OS 资源——执行结果复用、运行前依赖检查、故障恢复、访问控制、并发保护等方向待探索。具体问题列表见 [skill-library.md](skill-library.md#未来设想把-skill-当成-os-对象管理)。
- **Nav2 以外的 navigation 后端**：导航契约是后端无关的，目前只验证了 Nav2；可以接 Isaac / Tesseract / 自研规划器。
- **Liaison 产品化**：语音/多模态输入、用户管理、会话持久化、多会话并发。

## 贡献方式

Roadmap 条目欢迎认领。看到"TODO"或者"🚧"的项：

1. 在对应的 contract 目录 / crate 下开工；
2. PR 提交到 `dev` 分支；
3. 对外契约变动要同步更新本页 + `interface-catalog/`。

未列出的新想法也可以先在 issue 里讨论。
