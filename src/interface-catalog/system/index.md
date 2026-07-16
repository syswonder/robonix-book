# 系统（system）

系统服务是 Robonix 自身的核心服务，仓库内置、概念上不可替换，命名空间 `robonix/system/*`。下面列的是**对外暴露能力约定**的系统组件；完整的 12 个系统组件（含不对外暴露能力约定的）见 [系统组件](../../architecture/components.md)。

| 组件 | 能力约定 | 标准约定数 | 角色 |
|------|------|------|------|
| [Pilot](pilot.md) | `robonix/system/pilot`、`robonix/system/pilot/get_health` | 2 | 任务规划 / 决策 / 模块健康 |
| [Executor](executor.md) | `robonix/system/executor/*` | 5 | 方案编排、控制与能力分发 |
| [Liaison](liaison.md) | `robonix/system/liaison/*` | 5 | 统一人机交互与免手持语音入口 |
| [Scene](scene.md) | `robonix/system/scene/*` | 7 | 场景状态 / 语义地图 / 对象注册表 |
| [Soma](soma.md) | `robonix/system/soma/*` | 7 | 本体模型 / URDF / 外参 / 本体健康汇总 |
| [Vitals](vitals.md) | `robonix/system/vitals/*` | 3 | 健康阈值评估与系统/模块健康快照 |

> Atlas（能力目录本身）不走能力约定，用自己的 gRPC 接口——见 [Atlas 能力目录](../../architecture/atlas.md)。这些 system 域都没有 `driver` 能力约定；`rbnx boot` 按部署清单启动它们，不经原语/服务使用的 `Driver(CMD_*)` 能力约定。
