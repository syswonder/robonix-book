# 接入路线

本章面向要把一台新机器人、一项硬件能力或一个可复用功能接入 Robonix 的开发者。先确定你交付的对象，再进入对应流程。

<div class="task-grid">
  <a class="task-card" href="/integration-guide/vendor-onboarding">
    <span class="task-card__eyebrow">机器人部署</span>
    <span class="task-card__title">接入一台完整机器人</span>
    <span class="task-card__description">组织完整的机器人描述文件（URDF）、Soma 本体树、本体原语、系统配置和本地建图与导航参数。</span>
  </a>
  <a class="task-card" href="/developer-guide">
    <span class="task-card__eyebrow">软件包</span>
    <span class="task-card__title">开发原语、服务或技能</span>
    <span class="task-card__description">从已验证的 template-rbnx 可运行软件包开始，实现标准能力约定或定义新的技能能力约定。</span>
  </a>
  <a class="task-card" href="/integration-guide/packaging-spec">
    <span class="task-card__eyebrow">部署</span>
    <span class="task-card__title">编写部署清单</span>
    <span class="task-card__description">声明系统组件、软件包来源、目标清单、实例名和运行配置。</span>
  </a>
  <a class="task-card" href="/integration-guide/package-catalog">
    <span class="task-card__eyebrow">发布与分发</span>
    <span class="task-card__title">发布到社区软件包目录</span>
    <span class="task-card__description">完成仓库元数据、许可证、配置说明、自动校验和目录收录。</span>
  </a>
</div>

## 四种状态不要混为一谈

一个仓库从“存在”到“被官方列为已支持”要经过不同检查。Robonix 软件包目录（Package Catalog）收录不等于整机适配已经通过。

| 状态 | 说明 | 最低证据 |
|---|---|---|
| 仓库已创建 | 代码有可维护的上游位置 | README、LICENSE、维护者 |
| 软件包可构建 | `package_manifest.yaml` 可解析，目标平台构建成功 | 软件包持续集成检查、构建日志 |
| 目录已索引 | 软件包目录能读取仓库元数据和清单 | 目录拉取请求与校验结果 |
| 机器人已验证 | 完整机器人部署在指定本体上完成构建、启动、接口和安全验证 | 本体型号、源码提交号、目标清单、启动日志、验收记录 |

只有最后一种状态可以作为“该机器人已支持”的依据。

## 完整机器人接入的交付物

一个机器人部署仓库（Robot Deployment）至少包含：

```text
robot-<vendor>-<model>/
├── robonix_manifest.yaml       # 整机启动入口
├── soma.yaml                   # 本体部件树、尺寸、能力归属
├── urdf/                       # 唯一、完整的机器人 URDF 及其资源
├── config/                     # 本体专属建图、导航等参数
├── scripts/                    # CAN、设备权限等启动前准备
├── .env.example                # 只列变量名和非敏感默认值
└── README.md                   # 支持矩阵、安装、构建、启动和验收
```

原语、服务和技能可以位于独立仓库，通过机器人部署清单的 `url` 和 `branch` 引用；本体专属参数留在部署仓库的 `config/` 下。软件包接受哪些 `config` 字段，以该软件包根目录的 `config.spec` 和源代码验证为准。

## 推荐完成顺序

1. 用 [template-rbnx](https://github.com/syswonder/template-rbnx/tree/60dc85834c2714022b1821e6fce6c629c0314699) 在目标计算平台跑通一个无真实硬件的最小部署。
2. 建立完整 URDF 与 Soma 描述，先让系统能准确识别本体结构和能力提供方。
3. 逐个接入原语，并分别验证数据、坐标系、生命周期和停止行为。
4. 把本体专属建图与导航参数放入部署仓库的 `config/`，再接入对应服务。
5. 依次完成仿真、架空/台架、低速空场和真实场景验证。
6. 记录被验证的源码修订号、目标清单与验收结果，再提交软件包目录。

继续阅读[本体接入指南](vendor-onboarding.md)，其中每一步都给出工作目录、输入文件、验证命令和预期结果。
