# 技能

技能（Skill）把可复用的机器人行为暴露给规划器。技能接口由对应软件包根据任务语义定义，使用 `robonix/skill/*` 命名空间；Robonix 不要求所有技能实现同一组全局接口，因此这里没有类似原语和服务的固定能力约定表。

开发技能时，每个自定义可调用接口都必须有对应的能力约定 TOML 和所需接口定义语言（Interface Definition Language，IDL）文件，并通过 `rbnx codegen` 生成 gRPC、模型上下文协议（Model Context Protocol，MCP）或 ROS 2 接口。`CAPABILITY.md` 强烈推荐，用于向开发者和模型解释技能用途与调用条件，但当前为可选文件；缺少它不会使软件包启动失败。完整流程见[开发者指南](../../developer-guide.md)和[构建与代码生成](../../integration-guide/build-and-codegen.md)。

已经发布的技能软件包可在 [Robonix 软件包目录](https://syswonder.github.io/robonix-package-catalog/) 中按 `skill` 类型筛选。
