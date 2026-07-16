---
id: home
slug: /
title: Robonix 开发手册
---

# Robonix 开发手册

Robonix 是面向具身智能机器人的操作系统运行时。它把本体驱动、感知与导航服务、可复用技能和模型规划连接成一套可发现、可组合、可观测的系统，使同一套模型与技能能够部署到不同机器人上。

本手册面向第一次运行 Robonix、接入新机器人、开发 Robonix 软件包（Package）以及查询标准接口的读者。软件包可以实现原语（Primitive）、服务（Service）或技能（Skill）。

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

## Robonix 中的交付单元

- **机器人部署仓库**描述一台具体机器人：完整的机器人描述文件（URDF）、本体服务配置、系统监听地址、要启动的软件包，以及本体专属的建图与导航参数。
- **原语**直接连接硬件或硬件数据源，例如底盘、相机、雷达、惯性测量单元（IMU）和音频设备。
- **服务**提供可替换的系统能力，例如建图、导航、语音与记忆。
- **技能**封装模型可调用的语义行为，是最接近上层应用的可复用能力单元。
- **能力约定** 定义能力的稳定接口与载荷；**能力目录（Atlas）** 保存运行时提供方、传输绑定、生命周期状态和通道记录；**规划器（Pilot）** 规划任务，**执行器（Executor）** 通过 MCP 执行当前模型产生的 RTDL 方案，**交互服务（Liaison）** 承接用户交互。**本体服务（Soma）** 提供本体描述和运行事实，**场景服务（Scene）** 维护当前环境估计，**健康服务（Vitals）** 输出本体与系统模块的健康判断。

第一次阅读不需要先掌握这些内部组件。先完成[快速上手](getting-started/quickstart.md)，再按实际角色进入[本体接入](integration-guide/index.md)、[开发者指南](developer-guide.md)或[接口目录](interface-catalog/index.md)。
