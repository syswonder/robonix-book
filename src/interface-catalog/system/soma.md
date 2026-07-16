# Soma robonix/system/soma

Soma 是已实现的本体模型层：从 Soma YAML、URDF 和已注册 provider 汇总机器人结构、外形轮廓、传感器外参及本体健康状态。导航的 footprint、scene 的深度反投影和 Vitals 的健康评估都依赖这些本体事实。

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

上表是当前标准能力树的 7 条约定。内置 `robonix-soma` 当前实际声明 `get_yaml`、`get_urdf`、`footprint`、`get_health` 与 `health` 5 条 gRPC 能力；`description` 和 `sensor_extrinsics` 已标准化，但尚未由该内置 provider 声明。

`get_health` / `health` 汇总原语上报的本体健康事实；[Vitals](vitals.md) 消费该健康流、执行阈值评估，并对外提供规范化健康快照。二者不是同一职责。
