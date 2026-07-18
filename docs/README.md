---
id: home
slug: /
title: Robonix 开发手册
---

# Robonix 开发手册

Robonix 是面向具身智能机器人的操作系统运行时。它把本体驱动、感知与导航服务、可复用技能和模型规划连接成一套可发现、可组合、可观测的系统，使同一套模型与技能能够部署到不同机器人上。

本手册提供从首次运行到机器人接入、能力开发和接口查询的完整路径。第一次使用时，从 Webots 示例开始；已有机器人或功能代码时，直接进入对应的接入指南。

## 从这里开始

<div class="task-grid">
  <a class="task-card" href="/getting-started/quickstart">
    <span class="task-card__eyebrow">第一次使用</span>
    <span class="task-card__title">运行 Webots 示例</span>
    <span class="task-card__description">检查主机环境，安装 Robonix，在仿真中启动完整系统并提交第一条任务。</span>
  </a>
  <a class="task-card" href="/integration-guide/vendor-onboarding">
    <span class="task-card__eyebrow">机器人集成</span>
    <span class="task-card__title">接入一个新本体</span>
    <span class="task-card__description">创建机器人部署仓库（Robot Deployment），接入底盘、相机、雷达等原语，配置本体服务（Soma）、本体坐标变换（TF）、建图与导航。</span>
  </a>
  <a class="task-card" href="/developer-guide">
    <span class="task-card__eyebrow">能力开发</span>
    <span class="task-card__title">开发一个软件包</span>
    <span class="task-card__description">从可运行模板开始，开发原语、服务或技能，并通过能力约定（Contract）暴露标准能力。</span>
  </a>
  <a class="task-card" href="/interface-catalog">
    <span class="task-card__eyebrow">接口查询</span>
    <span class="task-card__title">查标准能力与数据结构</span>
    <span class="task-card__description">按原语、服务和系统查询能力约定、传输模式、ROS 接口定义语言（IDL）与运行时职责。</span>
  </a>
</div>

## 深入了解

- [系统组件](architecture/components.md)：了解用户输入、模型规划、任务执行、本体状态和环境状态如何协作。
- [运行时通信](architecture/runtime-communication.md)：了解能力如何注册、发现，以及 ROS 2、gRPC 和模型上下文协议（MCP）各自承担什么通信任务。
- [软件包目录](integration-guide/package-catalog.md)：浏览或发布可复用的原语、服务、技能和机器人部署仓库。
- [参与维护](contributing/documentation.md)：在本地预览、检查并提交文档修改。
