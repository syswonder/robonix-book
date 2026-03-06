# 第二章：用户手册

<!-- toc -->

本章介绍日常使用与运维，包括系统启动、能力与任务管理。若需开发原语、服务、技能或使用 SDK，请参阅 [第三章 开发文档](../chapter3-developer-guide/index.md)。

## 概述

- robonix-server（当前二进制名为 `robonix-core`）负责能力注册与任务执行，并可选提供 Web 管理界面（Robonix Console）。详见 [robonix-server](robonix-server.md)。
- `rbnx` 负责包管理、按 `recipe` 注册与启停能力进程，以及任务的提交、查询与取消。详见 [rbnx 命令行工具](rbnx-cli.md)。

## 使用流程

1. 启动 robonix-server（可选配置 Web 端口与资源目录）。
2. 执行 `rbnx deploy register <recipe>` 注册包与能力。
3. 执行 `rbnx deploy start` 启动能力进程。
4. 使用 `rbnx task create "任务描述"` 提交任务，`rbnx task get <id>` 查看结果。

完整命令与配置说明见 [rbnx 命令行工具](rbnx-cli.md)；环境准备见 [快速开始](../chapter1-getting-started/quickstart.md)。
