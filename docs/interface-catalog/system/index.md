# 系统

本节记录 Robonix 内置系统组件的职责与默认实现。它们通常使用 `robonix/system/*` 能力约定；`system` 是命名空间，不是 Atlas 中独立于原语、服务和技能的第四种能力提供方类型。下面列出对外暴露标准能力约定的系统组件；完整职责见[系统组件](../../architecture/components.md)。

| 组件 | 能力约定 | 当前实现数 | 角色 |
|------|------|------|------|
| [规划与决策](pilot.md) | `robonix/system/pilot`、`robonix/system/pilot/get_health` | 2 | 用户意图理解、任务规划与模块健康 |
| [任务执行](executor.md) | `robonix/system/executor/*` | 5 | 方案编排、控制、模型上下文协议（MCP）分发与内置工具 |
| [用户交互](liaison.md) | `robonix/system/liaison/*` | 5 | 带身份与访问控制的人机交互与免手持语音入口 |
| [场景理解](scene.md) | `robonix/system/scene/*` | 7 | 场景状态、语义地图与对象注册表 |
| [本体模型](soma.md) | `robonix/system/soma/*` | 5 | YAML/URDF、软件包启动、轮廓与运行状态快照 |
| [健康评估](vitals.md) | `robonix/system/vitals/*` | 3 | 本体健康阈值评估与模块健康快照 |

Atlas（能力目录本身）不走能力约定，而使用自己的 gRPC 接口，详见[能力目录](../../architecture/atlas.md)。新的系统软件包（包括 Scene）由框架自动注册共享的 `robonix/lifecycle/driver`，接收初始化、激活、停用和关闭命令，不创建新的 `robonix/system/*/driver` TOML。未实现生命周期回调时，框架记录警告并执行空操作。

:::warning[后向兼容：系统命名空间 Driver]
已有系统软件包可以暂时继续使用自己维护的唯一命名空间 Driver 和 Driver TOML，但计划迁移到共享 Driver，且不能同时注册两种 Driver。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::
