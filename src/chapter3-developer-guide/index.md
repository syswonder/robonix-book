# 第三章：开发文档

<!-- toc -->

本章说明 Robonix 各模块的设计、职责与扩展方式，以及如何开发原语、服务、技能和使用 robonix-sdk。若需了解日常使用与运维（例如如何启动系统、如何用 CLI 与 Web 管理任务），请参阅 [第二章 用户手册](../chapter2-user-guide/index.md)。

## 文档索引

| 文档 | 内容 |
|------|------|
| [Robonix Framework](robonix-framework.md) | 框架模块设计、对外接口（/rbnx/*）、规范校验与任务执行流程 |
| [原语开发指南](hardware-primitives-guide.md) | HAL 原语对接：包结构、manifest、规范表、注册与查询 |
| [服务开发指南](service-development-guide.md) | 标准服务接入：manifest、entry、服务规范表 |
| [技能开发指南](skill-development-guide.md) | 基本技能接入：start/status Topic 协议、manifest、实现要点 |
| [robonix-sdk 使用说明](robonix-sdk.md) | msg/srv 定义、Python 客户端、与 core/CLI 的关系 |
