---
title: 本体服务
---
<span id="soma-robonixsystemsoma"></span>
# 本体服务（Soma）

本体服务是已实现的本体数据与部署启动层：它原样读取一台机器人的 Soma YAML 和 URDF，并从 YAML 中解析底盘轮廓和夹爪状态配置；当前不会解释或维护通用部件拓扑。运行时监视器从 Atlas 发现 ROS 2 `arm/joint_states` 与 `chassis/odom`，据此生成关节、夹爪和底盘运行状态快照；健康服务订阅本体服务的健康流并做阈值判断。

在 `rbnx boot` 中，本体服务负责两阶段软件包生命周期：先启动所有原语并执行 `Driver(CMD_INIT)`、`Driver(CMD_ACTIVATE)`；原语全部进入 `ACTIVE` 后，本体服务才注册自身能力并进入 `ACTIVE`。等 `rbnx` 通过私有管道发出 `stage2` 触发后，本体服务再启动技能并只执行 `Driver(CMD_INIT)`；技能的首次 `CMD_ACTIVATE` 由执行器在第一次模型上下文协议（Model Context Protocol，MCP）调用前完成。

能力约定 TOML 在 `capabilities/system/soma/`，接口定义语言（Interface Definition Language，IDL）文件在 `capabilities/lib/soma/`。

## 接口

| 能力约定 ID | 模式 | 当前实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/system/soma/footprint` | `rpc` | gRPC | [`soma/GetFootprint`](../../reference/idl.md#soma-srv-getfootprint-srv) | `system/soma/footprint.v1.toml` |
| `robonix/system/soma/get_yaml` | `rpc` | gRPC | [`soma/GetYaml`](../../reference/idl.md#soma-srv-getyaml-srv) | `system/soma/get_yaml.v1.toml` |
| `robonix/system/soma/get_urdf` | `rpc` | gRPC | [`soma/GetUrdf`](../../reference/idl.md#soma-srv-geturdf-srv) | `system/soma/get_urdf.v1.toml` |
| `robonix/system/soma/get_health` | `rpc` | gRPC | [`soma/GetHealth`](../../reference/idl.md#soma-srv-gethealth-srv) | `system/soma/get_health.v1.toml` |
| `robonix/system/soma/health` | `rpc_server_stream` | gRPC | [`soma/StreamHealth`](../../reference/idl.md#soma-srv-streamhealth-srv) | `system/soma/health.v1.toml` |

上表只列内置 `robonix-soma` 当前实际注册的 5 条能力。源码树仍包含 `robonix/system/soma/description` 的 TOML 和 `GetDescription.srv`，但 Soma 没有实现或声明它，因此它不是可调用接口。当前源码树没有 `robonix/system/soma/sensor_extrinsics` 能力约定或实现，不应把它写入新清单。

## 运行状态如何聚合

Soma 在原语全部进入 `ACTIVE` 后，通过 Atlas 发现两类**精确能力约定 ID 且传输为 ROS 2** 的提供方，并为每条已连接通道启动运行状态订阅：

| 输入能力 | 聚合结果 | 当前判定 |
|---|---|---|
| `robonix/primitive/chassis/odom` | 底盘组件、线速度、角速度和 `moving` 指标 | 样本年龄不超过 2 秒才新鲜；线速度模长大于 `0.02 m/s` 或角速度模长大于 `0.03 rad/s` 时为移动 |
| `robonix/primitive/arm/joint_states` | 机械臂、关节位置、夹爪组件和 actuator 状态 | 样本年龄不超过 2 秒才新鲜；关节名和位置按数组索引对应 |

发现只以 Atlas 中实际注册的能力为准；仅在 `soma.yaml` 的 `exports` 写出路径不会创建提供方。反过来，Soma 也不会核对 YAML 声明与 Atlas 结果是否一致。没有发现任何 ROS 2 joint-state 或 odometry 源时，本体 YAML/URDF 接口仍可工作，但日志和健康快照会明确记录缺少运行状态源。

夹爪需要额外结合 `soma.yaml`：部件 `type` 必须是 `parallel_jaw_gripper` 或 `end_effector`，自身或祖先部件必须将 `robonix/primitive/arm/joint_states` 绑定到同一 `provider_id`，并提供数值 `state.open_position_m`。`joint_name` 默认是 `gripper`，`open_tolerance_m` 默认是 `0.002` 米。新鲜位置落在标定容差内时报告 `open`，否则报告 `holding_or_partially_closed`；过期或缺少 joint 时报告未知。`likely_holding` 只是未处于标定开启位置的启发式指标，不能作为抓取成功证明。

健康快照的 TTL 为 2 秒，当前包含 body、arm、joint、gripper、chassis、actuator、速度与夹爪指标，以及新鲜度和运行状态 reader 警告。它不会从这些运动样本推断安全：`power_sources`、`safety`、`safety_endpoints` 和 `faults` 当前为空或未知。需要电源、急停、保护停止和设备故障时，应由设备健康原语提供真实样本，再由 [Vitals](vitals.md) 做规范化和阈值评估。

## 各接口的边界

- `get_yaml` 返回启动时读取的原始 Soma YAML。请求中的 `robot_id` 可以为空或等于已加载 ID；其他 ID 返回 `NOT_FOUND`。一个 Soma 进程不会在多个机器人之间路由。
- `get_urdf` 返回 `urdf.path` 指向文件的原始文本。Soma 只保证文件可读，不解析 URDF XML，也不校验 link、joint、root 或 TF 完整性。
- `footprint` 返回 YAML 中声明的多边形，以及 Soma 计算的内切、外接半径。缺少 `robot.footprint` 不影响 Soma 启动，但该 RPC 返回 `FAILED_PRECONDITION`。
- `get_health` 返回最近快照，`health` 持续推送快照。两者反映当前有限的 joint-state/odometry 聚合，不是完整设备健康或安全证明。
- `description` 只有源码约定，没有当前服务实现；`sensor_extrinsics` 不存在。传感器几何关系应由完整 URDF/TF 表达，相机兼容外参由 `robonix/primitive/camera/extrinsics` 提供。

`soma.yaml` 的强制字段、完整移动操作机器人示例和验收流程见[机器人本体接入指南](../../integration-guide/vendor-onboarding.md#32-创建-somayaml)；sidecar 自动选择、`system.soma` 参数和日志诊断见[系统部署与启动流程](../../architecture/deployment-and-startup.md#soma-sidecar-与进程配置)。

导航服务的 footprint 当前来自部署自己的 Nav2 参数文件。场景服务的机器人位姿来自地图服务，相机到世界帧的主路径来自完整 URDF/TF；地图位姿与 `robonix/primitive/camera/extrinsics` 的组合只是 TF2 不可用时的兼容回退。这些路径都不依赖未实现的 Soma 接口。

`get_health` / `health` 对外提供当前运行状态快照；[健康服务](vitals.md)消费该流、执行阈值评估，并提供规范化健康快照。当前版本不会把设备健康原语的电机温度、电源、安全状态和故障自动聚合进这条本体服务输出链路。
