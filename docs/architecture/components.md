# 系统组件

Robonix 将具身智能运行时拆分为 12 项系统职责。它们是架构边界，不代表每次部署都会启动 12 个进程。本页区分当前实现与规划中的职责，避免把尚未落地的组件写成可用功能。

## 总体架构与运行流程

![Robonix 系统总体架构](/architecture/robonix-arch.png)

总览图展示 Robonix 的系统职责和主要连接关系，其中既包括当前实现，也包括仍在规划中的组件。当前已经实现的主要任务调用链如下；Chronos、Keystone 和 Sentinel 不在这条运行链中：

```text
用户 -> Liaison -> Pilot -> Executor -> MCP 能力提供方
                    |          |
                    |          +-> 技能首次调用前通过 gRPC Driver 激活
                    |
                    +-> 启动时读取 Soma 的本体描述和 URDF
                    +-> 每轮规划前读取 Soma 的本体状态
                    +-> 每轮规划前经 Executor 读取 Scene 快照
                        （不可用或过期时按未知状态处理）

Soma -> Vitals：本体健康流与阈值评估
运行组件 <-> Atlas：注册、发现、生命周期与通道记账
```

Robonix 以能力（Capability）为统一抽象。能力约定（Contract）定义能力的输入、输出和通信模式；Atlas 记录哪个能力提供方实现了该能力约定，以及应通过 gRPC、模型上下文协议（Model Context Protocol，MCP）还是 ROS 2 访问。原语（Primitive）、服务（Service）和技能（Skill）都通过这个机制注册和发现。能力约定模式、传输方式和实际调用链见[运行时通信](runtime-communication.md)。

### 任务流转

1. **Liaison** 接收文本或语音输入，统一用户身份元数据，根据配置执行用户准入检查，然后将通过的任务提交给 Pilot。Liaison 不自己实现语音识别、语音合成、麦克风、扬声器或声纹识别，而是通过 Atlas 发现对应的原语和服务。
2. **Pilot** 从 Atlas 读取当前可通过 MCP 调用的能力，构建模型提示，请求模型生成机器人任务描述语言（Robot Task Description Language，RTDL）动作树，再将动作树展开为执行方案并提交给 Executor。Pilot 启动时从 Soma 读取原始本体 YAML 和 URDF；每轮规划前再刷新 Soma 本体运行状态，并通过 Executor 调用 Scene 已声明的完整能力约定 `robonix/system/scene/get_robot_context` 读取空间快照。Soma 或 Scene 状态不可用、过期时，Pilot 将相应事实视为未知。当前版本尚未把 Vitals 的健康判断自动注入规划上下文。
3. **Executor** 验证并执行 RTDL 的 `sequence`、`parallel` 和 `do` 节点。`do` 节点携带 `provider_id` 和 `contract_id`；当前模型可调用路径固定通过 Atlas 连接该提供方的 MCP 能力，并以事件流返回节点状态和结果。Skill 仍为 `INACTIVE` 时，Executor 会在首次调用前通过其 gRPC driver 发送 `CMD_ACTIVATE`。
4. **Pilot** 根据执行结果决定结束当前任务，还是继续下一轮规划。

:::warning[当前安全边界]

Sentinel 尚未实现。当前 Executor 会将验证后的 `do` 节点直接分发给能力提供方，不会执行 Sentinel 规则。现有控制包括 Executor 的方案取消与停止点、Liaison 的用户准入，以及各原语、服务和技能自己的参数校验。这些不等价于统一的运行时安全监督。

:::

### 职责层次

- **注册、通信与日志**：Atlas 负责能力注册与发现；Nexus 是 gRPC、MCP 和 ROS 2 通信库的架构名称；Scribe 是进程内结构化日志库。
- **任务与交互**：Liaison 承接用户输入，Pilot 规划，Executor 执行方案。
- **本体、环境与健康**：Soma 提供原始本体 YAML、URDF、轮廓与本体运行状态，Scene 维护当前环境估计，Vitals 评估电源与部件健康。
- **尚未实现的规划职责**：Chronos 负责统一时间，Keystone 负责身份与策略，Sentinel 负责能力调用前的安全判定。

Soma 使用的 `soma.yaml` 是一台机器人的**本体描述文件**，不是 Soma 进程的启动配置。当前实现只强制要求可读取的 `urdf.path` 和非空 `robot.id`；若声明 `robot.footprint`，还会校验它能形成包围本体原点的有限多边形。部件树、提供方 ID、能力路径、尺寸和描述等字段会作为原始 YAML 保留，但不会被 Soma 逐项验证。完整的编写规则与示例见[机器人本体接入指南](../integration-guide/vendor-onboarding.md#32-创建-somayaml)，`system.soma` 的进程配置和自动注入规则见[系统部署与启动流程](deployment-and-startup.md#soma-sidecar-与进程配置)。

## 12 个系统组件

下表是组件职责、落地形态和源码位置的**架构摘录**，用于说明边界，不是进程启动清单或完整 API 参考。组件之间的 gRPC、MCP 与 ROS 2 连接关系以[运行时通信](runtime-communication.md)和 Atlas 实际发现结果为准。

| 组件 | 职责 | 当前状态 | 实现位置 |
|---|---|---|---|
| **Atlas** | 能力注册、能力约定目录与发现 | 已实现，独立进程 | `system/atlas` |
| **Chronos** | 统一时间与跨传感器时间对齐 | 尚未实现，目前只有设计说明 | `system/chronos/README.md` |
| **Executor** | RTDL 方案校验、执行、取消与能力分发 | 已实现，独立进程 | `system/executor` |
| **Keystone** | 用户身份、持久配置与访问策略 | 尚未实现；当前用户准入位于 Liaison | `system/keystone/README.md` |
| **Liaison** | 文本与语音交互入口、身份元数据和用户准入 | 已实现，独立进程 | `system/liaison` |
| **Nexus** | gRPC、MCP 和 ROS 2 通信库的集合 | 架构概念，不是独立进程 | `system/nexus/README.md` |
| **Pilot** | 模型驱动的推理与 RTDL 规划 | 已实现，独立进程 | `system/pilot` |
| **Scene** | 对象、空间关系、地图与当前环境估计 | 已实现，以系统软件包启动 | `system/scene` |
| **Scribe** | 进程内结构化日志 | 已实现为 Rust 库；尚无中心日志服务 | `system/scribe` |
| **Sentinel** | 能力调用前的安全策略判定 | 尚未实现，目前只有设计说明 | `system/sentinel/README.md` |
| **Soma** | 本体配置、URDF、轮廓与本体运行状态 | 已实现，独立进程 | `system/soma` |
| **Vitals** | 电源、部件与模块健康评估 | 已实现，独立进程 | `system/vitals` |

<span id="实现-vs-stub"></span>

## 当前实现状态

表中的“已实现”只表示当前源码存在可启动实现，不能单独证明某台机器人已经获得支持。机器人支持必须按下面的顺序验收；任一步失败，都不能把“进程存在”写成“整机已支持”。命令与参数来自源码中的 [`tools/rbnx/src/cmd/mod.rs`](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/tools/rbnx/src/cmd/mod.rs)，Atlas 检查项来自 [`system/atlas/proto/atlas.proto`](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/system/atlas/proto/atlas.proto)。

1. **核对源清单。** 逐项检查机器人仓库的 `robonix_manifest.yaml`，以及每个部署项通过 `path` / `url`、`branch` 和 `manifest` 选中的 `package_manifest*.yaml`。实例 `name` 必须与预期 `provider_id` 一致；包清单只能声明运行时真正提供的能力约定。
2. **验证并构建所选目标。** 对本地包先运行 `rbnx validate ./path/to/package`，再从部署目录运行 `rbnx build -f ./robonix_manifest.yaml`。后者必须成功构建部署清单实际选择的目标变体，而不只是默认包清单。
3. **前台启动并保留证据。** 运行 `rbnx boot -f ./robonix_manifest.yaml --verbose`。启动必须完成系统组件和软件包的注册/生命周期阶段，日志保存在清单目录的 `rbnx-boot/logs/`；不要只检查进程号。
4. **通过 Atlas 核对能力。** 在另一终端运行 `rbnx caps -v`。每个部署实例都必须以正确 ID 出现，原语和服务应达到 `ACTIVE`，技能初始化后可保持 `INACTIVE`；能力约定与传输必须和所选包清单及接入设计一致。
5. **检查日志。** 先用 `rbnx logs -d ./rbnx-boot/logs --list-tags` 确认实际日志标签，再用 `rbnx logs -d ./rbnx-boot/logs -l warn` 检查 warning 与 error。缺少传感器首帧、标定、依赖或 Driver 回调失败都必须处理，不能以 Atlas 已注册为由忽略。
6. **执行无运动副作用的功能测试。** 优先选择相机快照、状态读取或传感器采样，并核对真实载荷、时间戳和坐标系。包含 Pilot、Executor、提供方 `front_camera` 与 `robonix/primitive/camera/snapshot` 的部署，可用 `rbnx ask '只调用 provider_id=front_camera 的 robonix/primitive/camera/snapshot 获取一帧；不要执行任何运动。'` 验证整条调用链；不含该能力的机器人必须用其实际客户端测试另一条只读能力，不能照抄此示例。需要观察消费关系时，在调用进行期间运行 `rbnx channels`。
7. **验证安全关闭。** 从部署目录运行 `rbnx shutdown -f ./robonix_manifest.yaml`，确认生命周期关闭和包级 `stop` 完成、`rbnx-boot/state.json` 被移除，并复查日志中没有遗留设备输出或子进程错误。

完整的软件包与整机清单验收说明见[软件包与部署清单规范](../integration-guide/packaging-spec.md#5-约束与验收)。

`rbnx boot` 会启动部署清单 `system:` 中配置的系统组件。若清单包含任何原语或技能软件包，`rbnx` 还会自动注入并启动 Soma，因此这类部署可以省略显式的 `system.soma` 配置。Atlas、Executor、Pilot、Liaison、Soma 和 Vitals 是 `rbnx` 直接识别的内置 Rust 可执行程序。Scene 通过自己的软件包清单构建和启动。

Nexus 是对通信库的统称，Scribe 是由系统组件直接链接的日志库，二者都没有独立守护进程。Chronos、Keystone 和 Sentinel 尚无运行时实现，因此不能写入部署清单并期望 `rbnx boot` 启动它们。

Soma、Scene 与 Vitals 的职责不同：Soma 聚合夹爪、关节、底盘等本体事实；Scene 维护机器人周围对象、地图位姿和空间关系的当前估计；Vitals 消费 Soma 健康流并进行阈值评估，同时轮询已适配的系统模块健康接口，输出电源、部件与模块健康判断。Vitals 当前尚未根据生存时间（Time To Live，TTL）判断 Soma 快照是否过期。

## 与能力约定层的关系

Pilot、Executor、Liaison、Scene、Soma 和 Vitals 通过 Robonix 能力约定暴露可发现能力。Atlas 使用自己的 gRPC 控制面管理能力和能力约定目录，不作为普通能力提供方注册。详见 [Atlas 能力目录](atlas.md)。

启动顺序和清单解析见[系统部署与启动流程](deployment-and-startup.md)。
