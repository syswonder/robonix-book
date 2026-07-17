# 机器人描述

机器人描述领域当前没有独立的标准业务能力约定；本体数据的系统查询面见[本体服务](../system/soma.md)。新的机器人描述提供方省略 Driver 条目时由框架自动注册共享的 `robonix/lifecycle/driver`；显式选择共享 Driver 的行为相同。未实现生命周期回调时，框架记录警告并执行空操作。

如果仿真器或厂商程序已经发布机器人统一描述格式（Unified Robot Description Format，URDF）参数 `/robot_description` 以及坐标变换（Transform，TF）话题 `/tf`、`/tf_static`，不要再启动第二个机器人描述提供方。当前 Webots Tiago Lite 已由模拟器负责这些数据，因此部署清单不应加入通用软件包。只有部署中没有现成的 URDF/TF 发布方时，才使用部署侧的 `robot-description-rbnx`；完整步骤见[供应商接入指南](../../integration-guide/vendor-onboarding.md)。

能力约定 TOML 在 `capabilities/primitive/robot_description/`。

:::warning[后向兼容：机器人描述命名空间 Driver]
`robonix/primitive/robot_description/driver`、`lifecycle/Driver` 和 `primitive/robot_description/driver.v1.toml` 只用于仍由软件包自行维护 Driver TOML 的旧实现。目前仍可使用，但计划迁移到共享 Driver；两种 Driver 不能同时注册。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
