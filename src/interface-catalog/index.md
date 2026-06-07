# 接口目录

[toc]

本目录罗列 Robonix 在 `capabilities/` 下随仓库分发的**标准能力约定**（standard contracts）。能力约定描述一条能力"是什么"——`contract_id`、载荷 schema、交互形态（mode）——与具体由谁实现、绑哪种 transport、endpoint 在哪无关。能力约定的完整概念见 [命名空间与能力约定](../architecture/namespace-and-contracts.md)。

每条能力约定由一份 TOML（`capabilities/<kind>/<domain>/<leaf>.v1.toml`）加一份 ROS IDL（`capabilities/lib/<…>.msg` / `.srv`）描述，`rbnx codegen` 据此投影到 gRPC / MCP / ROS 2 三种 transport（见 [Package 构建与代码生成](../integration-guide/build-and-codegen.md)）。

## 按命名空间组织

标准能力约定按一级命名空间分三类（`robonix/skill/*` 没有随仓库分发的标准能力约定——技能由用户自定义）：

| 命名空间 | 含义 | 域 |
|----------|------|----|
| `robonix/primitive/*` | 原语：低层设备抽象 | [chassis](primitive/chassis.md) · [camera](primitive/camera.md) · [lidar](primitive/lidar.md) · [imu](primitive/imu.md) · [audio](primitive/audio.md) |
| `robonix/service/*` | 服务：场景级算法/能力，可替换 | [map](service/map.md) · [navigation](service/navigation.md) · [speech](service/speech.md) · [voiceprint](service/voiceprint.md) · [memory](service/memory.md) |
| `robonix/system/*` | 系统服务：Robonix 自身核心服务 | [pilot](system/pilot.md) · [executor](system/executor.md) · [liaison](system/liaison.md) · [scene](system/scene.md) · [soma](system/soma.md) |

完整的 12 个系统组件（含不对外暴露能力约定的）见 [系统组件](../architecture/components.md)。

## 怎么读每张表

每个域页面都是一张接口表：

- **能力约定 ID**：全局唯一，提供方和消费方都按它对话。
- **模式（mode）**：交互形态——`rpc` / `rpc_server_stream` / `rpc_client_stream` / `rpc_bidirectional_stream` / `topic_out` / `topic_in`。语义见 [命名空间与能力约定](../architecture/namespace-and-contracts.md)。
- **载荷（IDL）**：消息/服务类型，解析到 `capabilities/lib/` 下的 `.msg` / `.srv`。`std_msgs` / `geometry_msgs` / `sensor_msgs` / `nav_msgs` 这些"标准"消息也由 Robonix 在 `capabilities/lib/common_interfaces/` 提供，**不取发行版定义**——这样跨部署的数据结构严格一致。
- **能力约定 TOML**：路径省略 `capabilities/` 前缀；绝对路径见 `rbnx path capabilities`。

> 每个原语/服务域里名为 `driver` 的能力约定（`<domain>/driver`，载荷 `lifecycle/Driver`）是生命周期入口——`rbnx boot` 通过 `Driver(CMD_INIT/ACTIVATE/…)` 驱动该 provider 的状态机，不是数据接口。system 域以及声纹、记忆服务没有 `driver` 能力约定。
