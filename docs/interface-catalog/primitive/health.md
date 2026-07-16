# 设备健康

健康原语把具体设备的健康数据统一为按需快照和服务端流。按需调用方使用 `state`，持续监测方使用 `stream`。当前版本尚未把这些原语健康流聚合到 Soma 与 Vitals 的标准运行链；需要健康监控的部署必须分别验证设备健康提供方和 Vitals 实际接收到的数据。

能力约定 TOML 在 `capabilities/primitive/health/`，接口定义语言（Interface Definition Language，IDL）文件在 `capabilities/lib/health/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/primitive/health/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/health/driver.v1.toml` |
| `robonix/primitive/health/state` | `rpc` | [`health/GetHealthState`](../../reference/idl.md#health-srv-gethealthstate-srv) | `primitive/health/state.v1.toml` |
| `robonix/primitive/health/stream` | `rpc_server_stream` | [`health/StreamHealthState`](../../reference/idl.md#health-srv-streamhealthstate-srv) | `primitive/health/stream.v1.toml` |

`HealthState.voltage`、`HealthState.remaining_s` 以及各 `SensorReading` 的不可用数值都用 `-1` 表示未知。提供方必须填写稳定的传感器 `name`，不要把未知值填成 `0`；`stream` 每个采集周期发送一个 `HealthState`。

Robonix 源码树当前不包含可直接部署的真实设备健康原语。硬件部署需要按[供应商接入指南](../../integration-guide/vendor-onboarding.md)创建提供方，在 `package_manifest.yaml` 中只列出运行时真实声明的接口，并运行 `rbnx codegen -p <package>`。现有 Piper 路径中的 `health_piper` 也是部署侧软件包，不是仓库内置实现。
