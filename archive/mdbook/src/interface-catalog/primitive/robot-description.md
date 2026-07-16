# 机器人描述 robonix/primitive/robot_description

当前标准能力树只为机器人描述 provider 定义生命周期入口，没有定义独立的数据能力约定。本体数据的系统查询面见 [Soma](../system/soma.md)。

能力约定 TOML 在 `capabilities/primitive/robot_description/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/primitive/robot_description/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/robot_description/driver.v1.toml` |
