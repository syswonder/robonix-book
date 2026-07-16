---
title: 本体服务
---
<span id="soma-robonixsystemsoma"></span>
# 本体服务（Soma）

本体服务是已实现的本体数据与部署启动层：它原样读取一台机器人的 Soma YAML 和 URDF，并从 YAML 中解析底盘轮廓和夹爪状态配置；当前不会解释或维护通用部件拓扑。运行时监视器从 Atlas 发现 ROS 2 `arm/joint_states` 与 `chassis/odom`，据此生成关节、夹爪和底盘运行状态快照；健康服务订阅本体服务的健康流并做阈值判断。

在 `rbnx boot` 中，本体服务负责两阶段软件包生命周期：先启动所有原语并执行 `Driver(CMD_INIT)`、`Driver(CMD_ACTIVATE)`；原语全部进入 `ACTIVE` 后，本体服务才注册自身能力并进入 `ACTIVE`。等 `rbnx` 通过私有管道发出 `stage2` 触发后，本体服务再启动技能并只执行 `Driver(CMD_INIT)`；技能的首次 `CMD_ACTIVATE` 由执行器在第一次 MCP 调用前完成。

能力约定 TOML 在 `capabilities/system/soma/`，IDL 在 `capabilities/lib/soma/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/system/soma/description` | `rpc` | [`soma/GetDescription`](../../reference/idl.md#soma-srv-getdescription-srv) | `system/soma/description.v1.toml` |
| `robonix/system/soma/footprint` | `rpc` | [`soma/GetFootprint`](../../reference/idl.md#soma-srv-getfootprint-srv) | `system/soma/footprint.v1.toml` |
| `robonix/system/soma/sensor_extrinsics` | `rpc` | [`soma/GetSensorExtrinsics`](../../reference/idl.md#soma-srv-getsensorextrinsics-srv) | `system/soma/sensor_extrinsics.v1.toml` |
| `robonix/system/soma/get_yaml` | `rpc` | [`soma/GetYaml`](../../reference/idl.md#soma-srv-getyaml-srv) | `system/soma/get_yaml.v1.toml` |
| `robonix/system/soma/get_urdf` | `rpc` | [`soma/GetUrdf`](../../reference/idl.md#soma-srv-geturdf-srv) | `system/soma/get_urdf.v1.toml` |
| `robonix/system/soma/get_health` | `rpc` | [`soma/GetHealth`](../../reference/idl.md#soma-srv-gethealth-srv) | `system/soma/get_health.v1.toml` |
| `robonix/system/soma/health` | `rpc_server_stream` | [`soma/StreamHealth`](../../reference/idl.md#soma-srv-streamhealth-srv) | `system/soma/health.v1.toml` |

上表是当前标准能力树的 7 条约定。内置 `robonix-soma` 当前实际声明 `get_yaml`、`get_urdf`、`footprint`、`get_health` 与 `health` 5 条 gRPC 能力；`description` 和 `sensor_extrinsics` 虽仍存在于能力树，但未进入该软件包的代码生成、gRPC 服务或 Atlas 声明，当前不能调用。`sensor_extrinsics` 是 [Issue #156](https://github.com/syswonder/robonix/issues/156) 提议删除的旧接口，新部署不要实现它；Scene 的当前主路径是地图位姿与 `robonix/primitive/camera/extrinsics` 的组合。

导航服务的 footprint 当前来自部署自己的 Nav2 参数文件。场景服务的机器人位姿来自地图服务。该 issue 的目标是让传感器外参优先来自完整 URDF 发布的 TF；当前实现仍优先使用 `camera/extrinsics`。不要把这些运行路径写成对本体服务的隐式依赖。

`get_health` / `health` 对外提供当前运行状态快照；[健康服务](vitals.md)消费该流、执行阈值评估，并提供规范化健康快照。仓库虽然已有 `health.rs` 的原语健康流聚合器，但当前 `main.rs` 没有调用它，所以电机温度、电源、安全状态和故障等健康原语事实尚未进入这条本体服务输出链路。
