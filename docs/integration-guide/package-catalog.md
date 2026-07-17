# 软件包目录发布流程

## 向目录提交

Robonix 社区软件包和整机部署配置不放进 `syswonder/robonix` 主仓库。主仓库只保留核心运行时、内置参考服务和 Webots/Tiago 示例；可复用的原语、服务、技能软件包以及机器人部署仓库通过 [`syswonder/robonix-package-catalog`](https://github.com/syswonder/robonix-package-catalog) 汇总。

软件包目录的人工输入文件是 `syswonder/robonix-package-catalog` 仓库根目录的 [`catalog.yaml`](https://github.com/syswonder/robonix-package-catalog/blob/main/catalog.yaml)。向目录提交时只改这个文件。

普通软件包写在 `packages:` 下：

```yaml
packages:
  - name: robonix.service.mapping
    repo: https://github.com/syswonder/service-map-rbnx
```

整机部署仓库写在 `robots:` 下：

```yaml
robots:
  - name: robonix.robot.agilex.ranger_mini_v3
    repo: https://github.com/syswonder/robot-agilex-ranger_mini_v3
```

提交步骤：

1. 普通软件包仓库默认分支必须有根目录 `package_manifest.yaml`。
2. 机器人部署仓库默认分支必须有根目录 `robonix_manifest.yaml`。
3. 在 `syswonder/robonix-package-catalog` 提 PR，只向 `catalog.yaml` 增加一条 `name` + `repo`。
4. 目录的持续集成检查会通过 GitHub API 读取目标仓库默认分支，不克隆仓库。
5. 持续集成检查校验元数据、生成静态 API 和页面，并部署 GitHub Pages。

`catalog.yaml` 不重复写描述、标签、维护者、能力或部署依赖。普通软件包的这些信息来自 `package_manifest.yaml`；机器人部署仓库的这些信息来自 `robonix_manifest.yaml`。

## 普通软件包元数据

普通软件包指原语、服务或技能仓库。仓库根目录必须有 `package_manifest.yaml`，并且 `package.name` 必须与 `catalog.yaml` 里的 `packages[].name` 完全一致。

示例：

```yaml
manifestVersion: 1

package:
  name: robonix.primitive.intel.realsense_d435i.camera
  version: 0.1.0
  description: Intel RealSense D435i driver wrapper.
  tags: [primitive, camera, intel, realsense_d435i, rgbd]
  maintainers:
    - wheatfox <wheatfox17@icloud.com>
  license: Apache-2.0

build: bash scripts/build.sh
start: bash scripts/start.sh
stop: bash scripts/stop.sh

capabilities:
  - name: robonix/primitive/camera/rgb
  - name: robonix/primitive/camera/depth
```

这份完整清单对应的最小仓库结构如下；三个脚本都必须以可执行模式提交，`stop.sh` 必须可重复执行：

```text
.
├── package_manifest.yaml
├── camera_driver/
│   └── main.py
└── scripts/
    ├── build.sh
    ├── start.sh
    └── stop.sh
```

如果软件包没有 Driver 关闭之外的额外清理工作，可以同时省略 `stop:` 和 `scripts/stop.sh`。

软件包目录读取以下字段：

| 字段 | 必填 | 说明 |
|---|---|---|
| `package.name` | 是 | 软件包发布名，必须与 `catalog.yaml` 的 `packages[].name` 一致 |
| `package.version` | 是 | 软件包版本 |
| `package.description` | 是 | 一句话说明这个软件包做什么 |
| `package.license` | 是 | SPDX 许可证标识 |
| `package.tags` | 是 | 字符串列表，用于页面显示和过滤 |
| `package.maintainers` | 是 | 字符串列表，每项必须是 `Name <email@domain>` |
| `capabilities[].name` | 否 | 软件包声明的 Robonix 能力约定 ID |
| `capabilities[].path` | 否 | 软件包内能力约定 TOML 的相对路径 |

旧软件包清单中的 `package.vendor` 仍可读取，以保证已有软件包能继续构建和启动；运行时会忽略它，软件包目录也不使用该字段。新软件包使用 `package.name`、`package.tags` 和 `package.maintainers` 表达名称、分类和维护归属。

新软件包通常省略 Driver 条目，由框架自动提供 `robonix/lifecycle/driver`；显式声明共享 Driver 仍有效。

:::warning[后向兼容：已有命名空间 Driver]
已有软件包可以暂时继续发布唯一的 `<provider-namespace>/driver` 及其本地 Driver TOML；目录校验不会要求立即迁移。该方式计划迁移到共享 Driver，一个清单不能同时声明两种 Driver。
:::

## 机器人部署元数据

机器人部署仓库是整机部署配置仓库，核心文件是 `robonix_manifest.yaml`。它不需要额外的 `package_manifest.yaml`。

`robonix_manifest.yaml` 通过顶层 `catalog:` 块提供与普通软件包一致的目录元数据：

```yaml
manifestVersion: 1
name: robonix-ranger-mini-deploy

catalog:
  name: robonix.robot.agilex.ranger_mini_v3
  version: 0.1.0
  description: Robonix deploy manifest for the AgileX Ranger Mini v3 robot.
  license: Apache-2.0
  tags: [robot, deploy, agilex, ranger_mini_v3]
  maintainers:
    - wheatfox <wheatfox17@icloud.com>

primitive:
  - name: ranger_chassis
    url: https://github.com/syswonder/primitive-agilex-ranger_mini_v3-chassis-rbnx
    branch: main

service:
  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    branch: main
```

机器人的 `catalog` 块使用与普通软件包相同的发布元数据字段：`name`、`version`、`description`、`license`、`tags` 和 `maintainers`。

软件包目录会解析 `primitive:`、`service:`、`skill:` 下的 `url`、`branch` 和 `name`，生成 `deploy_dependencies[]`，并在机器人详情页展示“这个整机部署由哪些普通软件包组成”。如果依赖仓库也已收录在目录中，页面会自动链接到对应软件包详情页。

## 仓库命名

推荐命名：

| 类型 | 仓库名格式 | 例子 |
|---|---|---|
| 机器人部署仓库 | `robot-[company]-[model]` | `robot-agilex-ranger_mini_v3` |
| 原语软件包 | `primitive-[company]-[model]-[primitive_type]-rbnx` | `primitive-intel-realsense_d435i-camera-rbnx` |
| 服务软件包 | `service-[name]-rbnx` | `service-navigation-rbnx` |
| 技能软件包 | `skill-[name]-rbnx` | `skill-explore-rbnx` |

`primitive_type` 要对应 Robonix 原语接口族，例如 `camera`、`lidar`、`imu`、`chassis`、`audio`。硬件型号内部可用 `_`，避免用 `-` 造成仓库名分段歧义。

## 网站和接口

软件包目录主页和页面：

- 主页：[Robonix Package Catalog](https://syswonder.github.io/robonix-package-catalog/)
- 普通软件包列表：[软件包](https://syswonder.github.io/robonix-package-catalog/packages/)
- 机器人部署列表：[机器人](https://syswonder.github.io/robonix-package-catalog/robots/)

软件包目录 API 是 GitHub Pages 上的静态 JSON API。全部接口使用 `GET`，不需要 API 密钥。由于 Pages 是静态站，不支持服务端查询参数；搜索以及按标签、类型、能力过滤都在客户端取得 JSON 后完成。

| 方法 | 路径 | 参数 | 返回 |
|---|---|---|---|
| `GET` | `/api/v1/packages.json` | 无 | 普通原语、服务和技能软件包，返回 `packages[]` |
| `GET` | `/api/v1/robots.json` | 无 | 整机部署配置，返回 `robots[]` |
| `GET` | `/api/v1/catalog.json` | 无 | 全量目录，返回 `packages[]`，包含普通软件包和机器人部署仓库 |
| `GET` | `/api/v1/search.json` | 无 | 全量目录数组，用于客户端搜索和过滤 |
| `GET` | `/api/v1/package/<name>.json` | `name` 是经过 URL 编码的完整目录名称 | 单个普通软件包或机器人部署仓库；不存在时由 GitHub Pages 返回 `404` |

完整 URL 示例：

```text
GET https://syswonder.github.io/robonix-package-catalog/api/v1/packages.json
GET https://syswonder.github.io/robonix-package-catalog/api/v1/robots.json
GET https://syswonder.github.io/robonix-package-catalog/api/v1/catalog.json
GET https://syswonder.github.io/robonix-package-catalog/api/v1/package/robonix.service.mapping.json
GET https://syswonder.github.io/robonix-package-catalog/api/v1/package/robonix.robot.agilex.ranger_mini_v3.json
```

JavaScript 使用示例：

```js
const base = 'https://syswonder.github.io/robonix-package-catalog/api/v1';

const packageCatalog = await fetch(`${base}/packages.json`).then(r => r.json());
const mapping = packageCatalog.packages.find(p => p.name === 'robonix.service.mapping');

const robots = await fetch(`${base}/robots.json`).then(r => r.json());
const ranger = robots.robots.find(r => r.name === 'robonix.robot.agilex.ranger_mini_v3');

const detail = await fetch(`${base}/package/${encodeURIComponent(ranger.name)}.json`)
  .then(r => r.json());
```

单个目录对象的字段含义：

| 字段 | 类型 | 含义 |
|---|---|---|
| `name` | string | 目录发布名，例如 `robonix.service.mapping` 或 `robonix.robot.agilex.ranger_mini_v3` |
| `version` | string | 版本 |
| `description` | string | 一句话描述 |
| `license` | string | SPDX 许可证标识 |
| `tags` | string[] | 页面和客户端过滤标签 |
| `maintainers` | string[] | `Name <email@domain>` 格式维护者 |
| `repo` | string | GitHub 仓库 URL |
| `repo_name` | string | 仓库名 |
| `default_branch` | string | 目录索引时读取的默认分支 |
| `kind` | string | `primitive` / `service` / `skill` / `robot` |
| `catalog_type` | string | `package` 或 `robot` |
| `manifest` | string | 索引的源文件，通常是 `package_manifest.yaml` 或 `robonix_manifest.yaml` |
| `preview_image_url` | string | 软件包目录卡片使用的预览图 URL；机器人仓库通常来自 `assets/robot.jpg`，缺失时为空字符串 |
| `capabilities` | string[] | 普通软件包声明的 Robonix 能力约定 ID |
| `deploy_dependencies` | object[] | 机器人部署仓库依赖的原语、服务和技能条目 |
| `readme_url` | string | 被索引分支上的 README URL |
