# Soma robonix/system/soma

Soma 是本体模型层：回答"这个机器人长什么样"——URDF、外形轮廓、各传感器相对本体的外参。导航的 footprint、scene 的深度反投影都需要这些本体参数。

> v0.1 里 soma 的 provider 仍是 stub（能力约定已定，参考实现待补）——见 [系统组件](../../architecture/components.md)。

能力约定 TOML 在 `capabilities/system/soma/`，IDL 在 `capabilities/lib/soma/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/system/soma/description` | `rpc` | [`soma/GetDescription`](../../reference/idl.md#soma-srv-getdescription-srv) | `system/soma/description.v1.toml` |
| `robonix/system/soma/footprint` | `rpc` | [`soma/GetFootprint`](../../reference/idl.md#soma-srv-getfootprint-srv) | `system/soma/footprint.v1.toml` |
| `robonix/system/soma/sensor_extrinsics` | `rpc` | [`soma/GetSensorExtrinsics`](../../reference/idl.md#soma-srv-getsensorextrinsics-srv) | `system/soma/sensor_extrinsics.v1.toml` |

`description` 返回完整 URDF + 元数据（`urdf_xml` / `model_name` / `mass_kg` / `base_frame`），消费方用自己的库解析（urdf_parser_py / KDL / Pinocchio）；URDF 启动时加载一次、运行时不变，别频繁调。
