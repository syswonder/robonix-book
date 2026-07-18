# Robonix 开发手册

Robonix 是面向具身智能机器人的操作系统运行时。它把本体驱动、感知与导航服务、可复用技能和模型规划连接成一套可发现、可组合、可观测的系统，使同一套模型与技能能够部署到不同机器人上。

本手册服务于第一次运行 Robonix、接入新机器人、开发 Primitive / Service / Skill，以及查询标准接口的读者。所有可执行说明都以页面顶部显示的 `dev-next` 源码 revision 为校验基线。

<div class="task-grid">
  <a class="task-card" href="getting-started/quickstart.html">
    <span class="task-card__eyebrow">第一次使用</span>
    <span class="task-card__title">运行 Webots 示例</span>
    <span class="task-card__description">检查主机环境，安装 dev-next，在仿真中启动完整 Robonix 栈并提交第一条任务。</span>
  </a>
  <a class="task-card" href="integration-guide/vendor-onboarding.html">
    <span class="task-card__eyebrow">机器人集成</span>
    <span class="task-card__title">接入一个新本体</span>
    <span class="task-card__description">创建 robot deployment，接入底盘、相机、雷达等 Primitive，配置 Soma、TF、Mapping 与 Navigation。</span>
  </a>
  <a class="task-card" href="developer-guide.html">
    <span class="task-card__eyebrow">能力开发</span>
    <span class="task-card__title">开发一个 Package</span>
    <span class="task-card__description">从可运行模板开始，开发 Primitive、Service 或 Skill，并通过 contract 暴露标准能力。</span>
  </a>
  <a class="task-card" href="interface-catalog/index.html">
    <span class="task-card__eyebrow">接口查询</span>
    <span class="task-card__title">查标准能力与数据结构</span>
    <span class="task-card__description">按 Primitive、Service 和 System 查找 contract、传输模式、ROS IDL 与运行时职责。</span>
  </a>
</div>

## 先选择源码分支

| 分支 | 用途 | 本手册的关系 |
|---|---|---|
| [`dev-next`](https://github.com/syswonder/robonix/tree/dev-next) | 新功能集成与跨仓库联调，变化较快 | **本手册的实验基线** |
| [`dev`](https://github.com/syswonder/robonix/tree/dev) | 更稳定的日常开发线 | 新项目优先考虑；个别 `dev-next` 接口可能尚未进入该分支 |
| [`main`](https://github.com/syswonder/robonix/tree/main) | 重大里程碑与发布定档 | 不作为本手册持续更新的开发基线 |

> **注意**：不要混用不同分支的二进制、部署清单与接口文档。执行命令前先运行 `git branch --show-current`；页面顶部的 revision 表示该次文档构建实际校验的 Robonix commit。

## Robonix 中的交付单元

- **Robot deployment** 描述一台具体机器人：完整 URDF、Soma 本体树、系统监听地址、要启动的 Package，以及本体专属的 Mapping / Navigation 参数。
- **Primitive** 直接连接硬件或硬件数据源，例如底盘、相机、雷达、IMU 和音频设备。
- **Service** 提供可替换的系统能力，例如建图、导航、语音与记忆。
- **Skill** 封装模型可调用的语义行为，是最接近上层应用的可复用能力单元。
- **Contract** 定义能力的稳定接口与载荷；**Atlas** 保存运行时提供者、传输方式和 endpoint；**Pilot** 规划任务，**Executor** 执行 RTDL 方案，**Liaison** 承接用户交互。

第一次阅读不需要先掌握这些内部组件。先完成[快速上手](getting-started/quickstart.md)，再按实际角色进入[本体接入](integration-guide/index.md)、[开发者指南](developer-guide.md)或[接口目录](interface-catalog/index.md)。

## 文档维护原则

操作指南只保留读者需要执行的步骤；架构解释与字段参考分别放在对应章节。命令示例应给出前置条件、工作目录和预期结果，配置字段应能追溯到 `dev-next` 源码、标准 contract 或被引用 Package 的 `config.spec`。

发现说明与实现不一致时，请使用页面右上角的编辑入口提交修正，或在 [`syswonder/robonix-book`](https://github.com/syswonder/robonix-book/issues) 报告问题并附上页面、Robonix commit、执行命令和完整错误信息。
