---
title: 健康服务
---
# 健康服务（Vitals）

健康服务消费本体服务汇总的本体健康流，按部署选择的阈值规则生成规范化快照、连续健康流和模块健康快照。内置示例规则覆盖关节电机温度、关节驱动器温度、电池电量、电池电压和传感器温度。只有本体服务实际提供对应指标，且阈值文件包含匹配规则时，CPU、GPU、NVMe 等其他指标才会被评估。

健康服务会先注册下表 3 条 gRPC 能力，再在后台等待本体服务；本体服务不可用时进程仍在线，但没有可用的本体健康输入。`robonix-vitals --mock-soma` 会把该进程切换为模拟本体服务端，并不会同时启动健康服务，开发测试需要另起一个健康服务进程消费它。

能力约定 TOML 在 `capabilities/system/vitals/`，接口定义语言（Interface Definition Language，IDL）文件在 `capabilities/lib/vitals/` 与 `capabilities/lib/module_health/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/system/vitals/get` | `rpc` | [`vitals/GetVitals`](../../reference/idl.md#vitals-srv-getvitals-srv) | `system/vitals/get.v1.toml` |
| `robonix/system/vitals/stream` | `rpc_server_stream` | [`vitals/StreamVitals`](../../reference/idl.md#vitals-srv-streamvitals-srv) | `system/vitals/stream.v1.toml` |
| `robonix/system/vitals/modules/get` | `rpc` | [`module_health/GetModuleHealthSnapshot`](../../reference/idl.md#module-health-srv-getmodulehealthsnapshot-srv) | `system/vitals/modules/get.toml` |

本体事实仍由[本体服务](soma.md)提供；健康服务负责在这些事实之上做健康阈值判断和系统或模块级汇总。

当前本体健康转换尚未按本体服务快照的 `ttl_ms` 标记陈旧数据；如果流停止但连接未及时报错，最后一份本体健康值可能继续保留。安全监控不能把该快照当作独立的时效性保证。
