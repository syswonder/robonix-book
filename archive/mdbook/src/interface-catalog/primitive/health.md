# 设备健康 robonix/primitive/health

健康原语把具体设备的健康数据统一为按需快照和服务端流，供 Soma 汇总本体健康状态。

能力约定 TOML 在 `capabilities/primitive/health/`，IDL 在 `capabilities/lib/health/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/primitive/health/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/health/driver.v1.toml` |
| `robonix/primitive/health/state` | `rpc` | [`health/GetHealthState`](../../reference/idl.md#health-srv-gethealthstate-srv) | `primitive/health/state.v1.toml` |
| `robonix/primitive/health/stream` | `rpc_server_stream` | [`health/StreamHealthState`](../../reference/idl.md#health-srv-streamhealthstate-srv) | `primitive/health/stream.v1.toml` |
