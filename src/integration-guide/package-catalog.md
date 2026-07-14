# Package Catalog 发布流程

## 向 catalog 提交

Robonix 社区 package 和整机部署配置不放进 `syswonder/robonix` 主仓库。主仓库只保留核心运行时、内置参考服务和 Webots/Tiago 示例；可复用的 primitive / service / skill package 以及 robot deployment 仓库通过 [`syswonder/robonix-package-catalog`](https://github.com/syswonder/robonix-package-catalog) 汇总。

Catalog 的人工输入文件是 `syswonder/robonix-package-catalog` 仓库根目录的 [`catalog.yaml`](https://github.com/syswonder/robonix-package-catalog/blob/main/catalog.yaml)。提交 catalog 时只改这个文件。

普通 package 写在 `packages:` 下：

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

1. 普通 package 仓库默认分支必须有根目录 `package_manifest.yaml`。
2. Robot deployment 仓库默认分支必须有根目录 `robonix_manifest.yaml`。
3. 在 `syswonder/robonix-package-catalog` 提 PR，只向 `catalog.yaml` 增加一条 `name` + `repo`。
4. Catalog CI 会通过 GitHub API 读取目标仓库默认分支，不 clone 仓库。
5. CI 校验 metadata、生成静态 API、生成页面，并部署 GitHub Pages。

`catalog.yaml` 不重复写 description、tags、maintainers、capabilities 或部署依赖。普通 package 的这些信息来自 `package_manifest.yaml`；robot deployment 的这些信息来自 `robonix_manifest.yaml`。

## 普通 package metadata

普通 package 指 primitive / service / skill 仓库。仓库根目录必须有 `package_manifest.yaml`，并且 `package.name` 必须与 `catalog.yaml` 里的 `packages[].name` 完全一致。

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

capabilities:
  - name: robonix/primitive/camera/driver
  - name: robonix/primitive/camera/rgb
  - name: robonix/primitive/camera/depth
```

Catalog 读取字段：

| 字段 | 必填 | 说明 |
|---|---|---|
| `package.name` | 是 | package 发布名，必须与 `catalog.yaml` 的 `packages[].name` 一致 |
| `package.version` | 是 | package 版本 |
| `package.description` | 是 | 一句话说明这个 package 做什么 |
| `package.license` | 是 | SPDX license 字符串 |
| `package.tags` | 是 | 字符串列表，用于页面显示和过滤 |
| `package.maintainers` | 是 | 字符串列表，每项必须是 `Name <email@domain>` |
| `capabilities[].name` | 否 | package 声明的 Robonix contract ID |

旧 package manifest 中的 `package.vendor` 仍可由 `dev-next` 读取，以保证已有 package 能继续构建和启动；Catalog 不使用该字段。新 package 使用 `package.name`、`package.tags` 和 `package.maintainers` 表达名称、分类和维护归属。

## Robot deployment metadata

Robot deployment 仓库是整机部署配置仓库，核心文件是 `robonix_manifest.yaml`。它不需要额外的 `package_manifest.yaml`。

`robonix_manifest.yaml` 里用顶层 `catalog:` 块提供和普通 package 一致的 catalog metadata：

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

Robot 的 `catalog` 块使用与普通 package 相同的发布元数据字段：`name`、`version`、`description`、`license`、`tags` 和 `maintainers`。

Catalog 会解析 `primitive:`、`service:`、`skill:` 下的 `url` / `branch` / `name`，生成 `deploy_dependencies[]`，并在 robot 详情页展示“这个整机部署由哪些普通 package 组成”。如果依赖仓库也已收录在 catalog 中，页面会自动链接到对应 package 详情页。

## 仓库命名

推荐命名：

| 类型 | 仓库名格式 | 例子 |
|---|---|---|
| robot deployment | `robot-[company]-[model]` | `robot-agilex-ranger_mini_v3` |
| primitive package | `primitive-[company]-[model]-[primitive_type]-rbnx` | `primitive-intel-realsense_d435i-camera-rbnx` |
| service package | `service-[name]-rbnx` | `service-navigation-rbnx` |
| skill package | `skill-[name]-rbnx` | `skill-explore-rbnx` |

`primitive_type` 要对应 Robonix primitive 接口族，例如 `camera`、`lidar`、`imu`、`chassis`、`audio`。硬件型号内部可用 `_`，避免用 `-` 造成仓库名分段歧义。

## 网站和 API

Catalog 主页和页面：

- 主页：<https://syswonder.github.io/robonix-package-catalog/>
- 普通 package 列表：<https://syswonder.github.io/robonix-package-catalog/packages/>
- Robot deployment 列表：<https://syswonder.github.io/robonix-package-catalog/robots/>

Catalog API 是 GitHub Pages 上的静态 JSON API。全部接口使用 `GET`，不需要 API key。由于 Pages 是静态站，不支持服务端 query 参数；搜索、按 tag / kind / capability 过滤都在客户端拿到 JSON 后完成。

| Method | Path | 参数 | 返回 |
|---|---|---|---|
| `GET` | `/api/v1/packages` | 无 | 普通 primitive / service / skill package，返回 `packages[]` |
| `GET` | `/api/v1/robots` | 无 | 整机部署配置，返回 `robots[]` |
| `GET` | `/api/v1/catalog` | 无 | 全量 catalog，返回 `packages[]`，包含普通 package 和 robot deployment |
| `GET` | `/api/v1/search` | 无 | 全量 catalog 数组，用于客户端搜索/过滤 |
| `GET` | `/api/v1/package/<name>` | `name` 是完整 catalog name，URL-encoded | 单个普通 package 或 robot deployment；不存在时由 GitHub Pages 返回 `404` |

完整 URL 示例：

```text
GET https://syswonder.github.io/robonix-package-catalog/api/v1/packages
GET https://syswonder.github.io/robonix-package-catalog/api/v1/robots
GET https://syswonder.github.io/robonix-package-catalog/api/v1/catalog
GET https://syswonder.github.io/robonix-package-catalog/api/v1/package/robonix.service.mapping
GET https://syswonder.github.io/robonix-package-catalog/api/v1/package/robonix.robot.agilex.ranger_mini_v3
```

JavaScript 使用示例：

```js
const base = 'https://syswonder.github.io/robonix-package-catalog/api/v1';

const packageCatalog = await fetch(`${base}/packages`).then(r => r.json());
const mapping = packageCatalog.packages.find(p => p.name === 'robonix.service.mapping');

const robots = await fetch(`${base}/robots`).then(r => r.json());
const ranger = robots.robots.find(r => r.name === 'robonix.robot.agilex.ranger_mini_v3');

const detail = await fetch(`${base}/package/${encodeURIComponent(ranger.name)}`)
  .then(r => r.json());
```

单个 catalog 对象字段含义：

| 字段 | 类型 | 含义 |
|---|---|---|
| `name` | string | catalog 发布名，例如 `robonix.service.mapping` 或 `robonix.robot.agilex.ranger_mini_v3` |
| `version` | string | 版本 |
| `description` | string | 一句话描述 |
| `tags` | string[] | 页面和客户端过滤标签 |
| `maintainers` | string[] | `Name <email@domain>` 格式维护者 |
| `repo` | string | GitHub 仓库 URL |
| `repo_name` | string | 仓库名 |
| `default_branch` | string | catalog 索引时读取的默认分支 |
| `kind` | string | `primitive` / `service` / `skill` / `robot` |
| `catalog_type` | string | `package` 或 `robot` |
| `manifest` | string | 索引的源文件，通常是 `package_manifest.yaml` 或 `robonix_manifest.yaml` |
| `capabilities` | string[] | 普通 package 声明的 Robonix contract ID |
| `deploy_dependencies` | object[] | robot deployment 依赖的 primitive / service / skill 条目 |
| `readme_url` | string | 被索引分支上的 README URL |
