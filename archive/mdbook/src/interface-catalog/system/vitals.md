# Vitals robonix/system/vitals

Vitals 消费 Soma 汇总的本体健康流，评估 CPU、GPU、NVMe、供电与关节电机等阈值，并提供规范化的当前快照、连续健康流和系统模块健康快照。生产部署依赖 Soma；开发和测试也可使用 Vitals 自带的 mock Soma 数据路径。

能力约定 TOML 在 `capabilities/system/vitals/`，IDL 在 `capabilities/lib/vitals/` 与 `capabilities/lib/module_health/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/system/vitals/get` | `rpc` | [`vitals/GetVitals`](../../reference/idl.md#vitals-srv-getvitals-srv) | `system/vitals/get.v1.toml` |
| `robonix/system/vitals/stream` | `rpc_server_stream` | [`vitals/StreamVitals`](../../reference/idl.md#vitals-srv-streamvitals-srv) | `system/vitals/stream.v1.toml` |
| `robonix/system/vitals/modules/get` | `rpc` | [`module_health/GetModuleHealthSnapshot`](../../reference/idl.md#module-health-srv-getmodulehealthsnapshot-srv) | `system/vitals/modules/get.toml` |

本体事实仍由 [Soma](soma.md) 提供；Vitals 负责在这些事实之上做健康阈值判断和系统/模块级汇总。
