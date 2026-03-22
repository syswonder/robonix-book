# 第三章：开发文档

本章面向要在 Robonix 上实现接口、编写 package 的开发者：从 RIDL 与生成代码出发，说明如何补全业务逻辑，并用 `rbnx` 完成构建与运行。若你实现的是相机、机械臂或地图等“对外提供能力”的节点，可结合**抽象原语**与**厂商接入**两节对照接口清单与推荐做法。

## 文档索引

| 文档 | 内容 |
|------|------|
| [Package 开发指南](package-development.md) | 从 RIDL 到 Python 模块与 `create_*` 创建函数、manifest 字段、典型 server/client 示例、`rbnx build`/`start` |
| [抽象硬件原语](primitives/index.md) | `robonix/prm/*` 下各类硬件的接口列表与典型组合（厂商可按能力选子集） |
| [ridlc 开发手册](ridlc.md) | ridlc 生成约定、ROS 映射，以及 **§5** 各通信语义下用户逻辑的补全位置与示例 |
| [硬件/服务厂商接入指南](vendor-integration.md) | 相机、机械臂、语义地图等场景的目录与 manifest 示例，强调按需实现、不必全量 |
