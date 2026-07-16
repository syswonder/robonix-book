# 系统组件

Robonix 将具身智能运行时拆分为 12 项系统职责。它们是架构边界，不代表每次部署都会启动 12 个进程。本页区分当前实现与规划中的职责，避免把尚未落地的组件写成可用功能。

## 总体架构与运行流程

下图只表示当前已实现的主要调用链：

```text
用户 -> Liaison -> Pilot -> Executor -> MCP 能力提供方
                    |          |
                    |          +-> 技能首次调用前通过 gRPC Driver 激活
                    |
                    +-> 启动时读取 Soma 的本体描述和 URDF
                    +-> 每轮规划前读取 Soma 的本体状态
                    +-> 每轮规划前经 Executor 尝试读取 Scene 快照
                        （当前接口未声明，不能被发现）

Soma -> Vitals：本体健康流与阈值评估
运行组件 <-> Atlas：注册、发现、生命周期与通道记账
```

Chronos、Keystone 和 Sentinel 不在这条当前运行链中；它们的目标职责与实现状态在下文单独列出。

Robonix 以能力（Capability）为统一抽象。契约（Contract）定义能力的输入、输出和通信模式；Atlas 记录哪个能力提供者实现了该契约，以及应通过 gRPC、MCP 还是 ROS 2 访问。原语（Primitive）、服务（Service）和技能（Skill）都通过这个机制注册和发现。

### 任务流转

1. **Liaison** 接收文本或语音输入，统一用户身份元数据，根据配置执行用户准入检查，然后将通过的任务提交给 Pilot。Liaison 不自己实现语音识别、语音合成、麦克风、扬声器或声纹识别，而是通过 Atlas 发现对应的原语和服务。
2. **Pilot** 从 Atlas 读取当前可通过 MCP 调用的能力，构建模型提示，请求模型生成机器人任务描述语言（Robot Task Description Language，RTDL）动作树，再将动作树展开为执行方案并提交给 Executor。Pilot 启动时从 Soma 读取本体描述和 URDF；每轮规划前再刷新 Soma 本体运行状态，并通过 Executor 尝试读取 Scene 的空间快照。当前版本尚未把 Vitals 的健康判断自动注入规划上下文。
3. **Executor** 验证并执行 RTDL 的 `sequence`、`parallel` 和 `do` 节点。`do` 节点携带 `provider_id` 和 `contract_id`；当前模型可调用路径固定通过 Atlas 连接该提供方的 MCP 能力，并以事件流返回节点状态和结果。Skill 仍为 `INACTIVE` 时，Executor 会在首次调用前通过其 gRPC driver 发送 `CMD_ACTIVATE`。
4. **Pilot** 根据执行结果决定结束当前任务，还是继续下一轮规划。

:::warning 当前安全边界

Sentinel 尚未实现。当前 Executor 会将验证后的 `do` 节点直接分发给能力提供者，不会执行 Sentinel 规则。现有控制包括 Executor 的方案取消与停止点、Liaison 的用户准入，以及各原语、服务和技能自己的参数校验。这些不等价于统一的运行时安全监督。

:::

### 职责层次

- **注册、通信与日志**：Atlas 负责能力注册与发现；Nexus 是 gRPC、MCP 和 ROS 2 通信库的架构名称；Scribe 是进程内结构化日志库。
- **任务与交互**：Liaison 承接用户输入，Pilot 规划，Executor 执行方案。
- **本体、环境与健康**：Soma 提供本体描述与本体状态，Scene 维护当前环境估计，Vitals 评估电源与部件健康。
- **尚未实现的规划职责**：Chronos 负责统一时间，Keystone 负责身份与策略，Sentinel 负责能力调用前的安全判定。

## 12 个系统组件

| 组件 | 职责 | 当前状态 | 实现位置 |
|---|---|---|---|
| **Atlas** | 能力注册、契约目录与发现 | 已实现，独立进程 | `system/atlas` |
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

`rbnx boot` 只会启动部署清单 `system:` 中配置的系统组件。Atlas、Executor、Pilot、Liaison、Soma 和 Vitals 是 `rbnx` 直接识别的内置 Rust 可执行程序。Scene 通过自己的软件包清单构建和启动。

Nexus 是对通信库的统称，Scribe 是由系统组件直接链接的日志库，二者都没有独立守护进程。Chronos、Keystone 和 Sentinel 尚无运行时实现，因此不能写入部署清单并期望 `rbnx boot` 启动它们。

Soma、Scene 与 Vitals 的职责不同：Soma 聚合夹爪、关节、底盘等本体事实；Scene 维护机器人周围对象、地图位姿和空间关系的当前估计；Vitals 消费 Soma 健康流并进行阈值评估，同时轮询已适配的系统模块健康接口，输出电源、部件与模块健康判断。Soma 快照的 TTL 过期判定尚未在 Vitals 的本体健康转换路径中实现。

## 与契约层的关系

Pilot、Executor、Liaison、Scene、Soma 和 Vitals 通过 Robonix 契约暴露可发现能力。Atlas 使用自己的 gRPC 控制面管理能力和契约目录，不作为普通能力提供者注册。详见 [Atlas 能力目录](atlas.md)。

启动顺序和清单解析见[系统部署与启动流程](deployment-and-startup.md)。
