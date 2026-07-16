# 接口目录


本目录罗列 Robonix 在 `capabilities/` 下随仓库分发的标准能力约定。能力约定描述接口名称、数据类型和交互方式，不指定由哪个提供方实现，也不记录某次运行中的网络地址。完整概念见[命名空间与能力约定](../architecture/namespace-and-contracts.md)。

每条能力约定由 `capabilities/<kind>/` 下的一份 TOML 和 `capabilities/lib/` 下的消息或服务定义共同描述。`rbnx codegen` 根据软件包的构建目标生成 gRPC 接口、MCP 类型辅助代码或 ROS 2 消息包；一次构建只生成明确请求的产物。详见[软件包构建与代码生成](../integration-guide/build-and-codegen.md)。

## 按命名空间组织

标准能力约定按一级命名空间分三类（`robonix/skill/*` 没有随仓库分发的标准能力约定——技能由用户自定义）：

| 命名空间 | 含义 | 域 |
|----------|------|----|
| `robonix/primitive/*` | 原语：低层设备抽象 | [机械臂](primitive/arm.md) · [音频](primitive/audio.md) · [相机](primitive/camera.md) · [底盘](primitive/chassis.md) · [设备健康](primitive/health.md) · [惯性测量单元](primitive/imu.md) · [激光雷达](primitive/lidar.md) · [机器人描述](primitive/robot-description.md) |
| `robonix/service/*` | 服务：可替换的通用能力 | [地图](service/map.md) · [导航](service/navigation.md) · [语音](service/speech.md) · [声纹](service/voiceprint.md) · [记忆](service/memory.md) |
| `robonix/system/*` | 系统服务：Robonix 自身组件 | [规划器](system/pilot.md) · [执行器](system/executor.md) · [交互服务](system/liaison.md) · [场景服务](system/scene.md) · [本体服务](system/soma.md) · [健康服务](system/vitals.md) |

完整的 12 个系统组件（含不对外暴露能力约定的）见 [系统组件](../architecture/components.md)。

## 怎么读每张表

每个域页面都是一张接口表：

- **能力约定 ID**：接口名称，例如 `robonix/primitive/camera/rgb`。多个提供方可以暴露同一个接口；运行时由提供方 ID 区分具体设备。Atlas 按能力约定 ID 合并接口定义，重复定义会保留后加载者并给出警告。
- **模式**：交互方式，包括 `rpc`、`rpc_server_stream`、`rpc_client_stream`、`rpc_bidirectional_stream`、`topic_out` 和 `topic_in`。语义见[命名空间与能力约定](../architecture/namespace-and-contracts.md)。
- **载荷（IDL）**：消息或服务类型，解析到 `capabilities/lib/` 下的 `.msg` 或 `.srv`。Robonix 自带的 ROS 2 消息定义也从这里生成，使不同 ROS 2 发行版上的 Robonix 组件使用同一套数据结构。
- **能力约定 TOML**：路径省略 `capabilities/` 前缀；绝对路径见 `rbnx path capabilities`。

> 名为 `driver` 的能力约定是提供方的生命周期入口，不是业务数据接口。只有目录中实际列出该约定的软件包才通过它接收初始化、激活和关闭命令；不要据此假定所有原语或服务都必须声明独立的 `driver`。
