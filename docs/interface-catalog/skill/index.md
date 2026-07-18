# 技能

技能（Skill）把可复用的机器人行为暴露给规划器。技能接口由对应软件包根据任务语义定义，使用 `robonix/skill/*` 命名空间；Robonix 不要求所有技能实现同一组全局接口，因此这里没有类似原语和服务的固定能力约定表。

开发技能时，每个自定义可调用接口都必须有对应的能力约定 TOML 和所需接口定义语言（Interface Definition Language，IDL）文件，并通过 `rbnx codegen` 生成 gRPC、模型上下文协议（Model Context Protocol，MCP）或 ROS 2 接口。`CAPABILITY.md` 强烈推荐，用于向开发者和模型解释技能用途与调用条件，但当前为可选文件；缺少它不会使软件包启动失败。完整流程见[开发者指南](../../developer-guide.md)和[构建与代码生成](../../integration-guide/build-and-codegen.md)。

当前 Executor 只把技能的 MCP 能力约定当作 RTDL `do` 节点目标；技能即使另行提供 gRPC 或 ROS 2 数据面，也不会因此自动成为模型可调用工具。Soma 在启动第二阶段只对技能执行 `Driver(CMD_INIT)`，使其保持 `INACTIVE`；Executor 首次分发该技能的 MCP 调用前，再通过共享 `robonix/lifecycle/driver` 发送 `CMD_ACTIVATE`。新技能省略 Driver 条目时由框架自动注册共享 Driver；未实现生命周期回调时，框架记录警告并执行空操作。

:::warning[后向兼容：技能命名空间 Driver]
已有技能可以暂时继续使用自己维护的唯一命名空间 Driver 和 Driver TOML，但计划迁移到共享 Driver，且不能同时注册两种 Driver。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

已经发布的技能软件包可在 [Robonix 软件包目录](https://syswonder.github.io/robonix-package-catalog/) 中按 `skill` 类型筛选。
