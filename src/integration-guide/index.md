# 接入路线

本章面向要把一台新机器人、一项硬件能力或一个可复用功能接入 Robonix 的开发者。先确定你交付的对象，再进入对应流程。

<div class="task-grid">
  <a class="task-card" href="vendor-onboarding.html">
    <span class="task-card__eyebrow">Robot deployment</span>
    <span class="task-card__title">接入一台完整机器人</span>
    <span class="task-card__description">组织完整 URDF、Soma、本体 Primitive、系统配置和本地 Mapping / Navigation 参数。</span>
  </a>
  <a class="task-card" href="../developer-guide.html">
    <span class="task-card__eyebrow">Package</span>
    <span class="task-card__title">开发 Primitive、Service 或 Skill</span>
    <span class="task-card__description">从 template-rbnx 的可运行 Package 开始，实现标准 contract 或定义新的 Skill contract。</span>
  </a>
  <a class="task-card" href="packaging-spec.html">
    <span class="task-card__eyebrow">Deployment</span>
    <span class="task-card__title">编写部署清单</span>
    <span class="task-card__description">声明系统组件、Package 来源、target manifest、实例名和运行时 config。</span>
  </a>
  <a class="task-card" href="package-catalog.html">
    <span class="task-card__eyebrow">Distribution</span>
    <span class="task-card__title">发布到社区 Catalog</span>
    <span class="task-card__description">完成仓库元数据、许可证、配置说明、自动校验和 Catalog 收录。</span>
  </a>
</div>

## 四种状态不要混为一谈

一个仓库从“存在”到“被官方列为已支持”要经过不同检查。Catalog 收录不等于整机适配已经通过。

| 状态 | 说明 | 最低证据 |
|---|---|---|
| 仓库已创建 | 代码有可维护的上游位置 | README、LICENSE、维护者 |
| Package 可构建 | `package_manifest.yaml` 可解析，目标平台构建成功 | Package CI、构建日志 |
| Catalog 已索引 | Catalog 能读取仓库元数据和 manifest | Catalog PR 与校验结果 |
| Robot 已验证 | 完整 deployment 在指定本体上完成 build、boot、接口和安全验证 | 本体型号、源码 commit、target、启动日志、验收记录 |

只有最后一种状态可以作为“该机器人已支持”的依据。

## 完整机器人接入的交付物

一个 robot deployment 仓库至少包含：

```text
robot-<vendor>-<model>/
├── robonix_manifest.yaml       # 整机启动入口
├── soma.yaml                   # 本体部件树、尺寸、能力归属
├── urdf/                       # 唯一、完整的机器人 URDF 及其资源
├── config/                     # 本体专属 Mapping / Navigation 等参数
├── scripts/                    # CAN、设备权限等启动前准备
├── .env.example                # 只列变量名和非敏感默认值
└── README.md                   # 支持矩阵、安装、构建、启动和验收
```

Primitive、Service 和 Skill 可以位于独立仓库，通过 deployment manifest 的 `url` 和 `branch` 引用；本体专属参数留在 deployment 的 `config/` 下。Package 接受哪些 `config` 字段，以该 Package 根目录的 `config.spec` 和源代码验证为准。

## 推荐完成顺序

1. 用 [template-rbnx](https://github.com/syswonder/template-rbnx) 在目标计算平台跑通一个无真实硬件的最小部署。
2. 建立完整 URDF 与 Soma 描述，先让系统能准确识别本体结构和能力提供者。
3. 逐个接入 Primitive，并分别验证数据、坐标系、生命周期和停止行为。
4. 把本体专属 Mapping / Navigation 参数放入 deployment `config/`，再接入对应 Service。
5. 依次完成仿真、架空/台架、低速空场和真实场景验证。
6. 固定被验证的源码 revision、target manifest 与验收结果，再提交 Catalog。

继续阅读[本体接入指南](vendor-onboarding.md)，其中每一步都给出工作目录、输入文件、验证命令和预期结果。
