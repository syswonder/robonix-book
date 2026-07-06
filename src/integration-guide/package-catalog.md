# Package Catalog 发布流程

Robonix 社区 package 不放进 `syswonder/robonix` 主仓库。主仓库只保留核心运行时、内置参考服务和 Webots/Tiago 示例；可复用的 primitive / service / skill package 放在各自 GitHub 仓库，通过 [`syswonder/robonix-package-catalog`](https://github.com/syswonder/robonix-package-catalog) 汇总。

Catalog 主页和 API：

- 主页：<https://syswonder.github.io/robonix-package-catalog/>

Catalog API 是 GitHub Pages 上的静态 JSON API。全部接口使用 `GET`，不需要 API key。由于 Pages 是静态站，不支持服务端 query 参数；搜索、按 tag / kind / capability 过滤都在客户端拿到 JSON 后完成。

| Method | Path | 参数 | 返回 |
|---|---|---|---|
| `GET` | `/api/v1/packages` | 无 | catalog 对象，包含 `api_version`、`generated_at`、`packages[]` |
| `GET` | `/api/v1/search` | 无 | package 对象数组，用于客户端搜索/过滤 |
| `GET` | `/api/v1/package/<package-name>` | `package-name` 是完整 `package.name`，URL-encoded | 单个 package 对象；不存在时由 GitHub Pages 返回 `404` |

完整 URL 示例：

```text
GET https://syswonder.github.io/robonix-package-catalog/api/v1/packages
GET https://syswonder.github.io/robonix-package-catalog/api/v1/search
GET https://syswonder.github.io/robonix-package-catalog/api/v1/package/robonix.service.mapping
```

JavaScript 使用示例：

```js
const base = 'https://syswonder.github.io/robonix-package-catalog/api/v1';

const catalog = await fetch(`${base}/packages`).then(r => r.json());
const mapping = catalog.packages.find(p => p.name === 'robonix.service.mapping');

const detail = await fetch(`${base}/package/${encodeURIComponent(mapping.name)}`)
  .then(r => r.json());
```

`GET /api/v1/packages` 返回结构：

```json
{
  "api_version": "1",
  "generated_at": "2026-07-06T12:00:00+00:00",
  "packages": [
    {
      "name": "robonix.service.mapping",
      "version": "0.4.0",
      "description": "Map and SLAM service package for Robonix.",
      "tags": ["service", "mapping", "slam"],
      "maintainers": ["wheatfox <wheatfox17@icloud.com>"],
      "repo": "https://github.com/syswonder/service-map-rbnx",
      "repo_name": "service-map-rbnx",
      "default_branch": "main",
      "kind": "service",
      "capabilities": ["robonix/service/map/save_map"],
      "readme_url": "https://github.com/syswonder/service-map-rbnx/blob/main/README.md"
    }
  ]
}
```

单个 package 对象字段含义：

| 字段 | 类型 | 含义 |
|---|---|---|
| `name` | string | package 发布名，例如 `robonix.service.mapping` |
| `version` | string | package 版本 |
| `description` | string | 一句话描述 |
| `tags` | string[] | 页面和客户端过滤标签 |
| `maintainers` | string[] | `Name <email@domain>` 格式维护者 |
| `repo` | string | GitHub 仓库 URL |
| `repo_name` | string | 仓库名 |
| `default_branch` | string | catalog 索引时读取的默认分支 |
| `kind` | string | `primitive` / `service` / `skill` |
| `capabilities` | string[] | package manifest 声明的 Robonix contract ID |
| `readme_url` | string | 被索引分支上的 README URL |

## 仓库命名

推荐命名：

| 类型 | 仓库名格式 | 例子 |
|---|---|---|
| primitive package | `primitive-[company]-[model]-[primitive_type]-rbnx` | `primitive-intel-realsense_d435i-camera-rbnx` |
| service package | `service-[name]-rbnx` | `service-navigation-rbnx` |
| skill package | `skill-[name]-rbnx` | `skill-explore-rbnx` |

`primitive_type` 要对应 Robonix primitive 接口族，例如 `camera`、`lidar`、`imu`、`chassis`、`audio`。硬件型号内部可用 `_`，避免用 `-` 造成仓库名分段歧义。

## Package manifest 的 catalog 元数据

package 仓库根目录必须有 `package_manifest.yaml`。Catalog CI 通过 GitHub API 读取默认分支上的这个文件，不 clone 仓库。

用于 catalog 的 `package` 元数据：

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

字段说明：

| 字段 | 必填 | 说明 |
|---|---|---|
| `package.name` | 是 | package 发布名，必须与 catalog 里的 `name` 完全一致 |
| `package.version` | 是 | package 版本 |
| `package.description` | 是 | 一句话说明这个 package 做什么，用于 catalog 页面 |
| `package.tags` | 是 | 字符串列表，用于页面显示和过滤 |
| `package.maintainers` | 是 | 字符串列表，每项必须是 `Name <email@domain>` |
| `package.license` | 否 | SPDX license 字符串 |

## 向 catalog 提交

Catalog 的人工输入只有 `catalog.yaml`：

```yaml
packages:
  - name: robonix.service.mapping
    repo: https://github.com/syswonder/service-map-rbnx
```

提交步骤：

1. 确认 package 仓库默认分支有根目录 `package_manifest.yaml`。
2. 确认 `package.name`、`version`、`description`、`tags`、`maintainers` 都已填写。
3. 在 `syswonder/robonix-package-catalog` 提 PR，只给 `catalog.yaml` 添加一条 `name` + `repo`。
4. Catalog CI 会通过 GitHub API 读取 package manifest，检查：
   - `catalog.yaml` 里 `name` / `repo` 不重复；
   - `package_manifest.yaml` 存在；
   - `package.name` 与 catalog 条目一致；
   - catalog 必需元数据齐全；
   - `capabilities[]` 每项都有 `name`。
5. CI 生成 `generated/api/*`、`public/api/*`、package 详情页和 README，并部署 Pages。

package 描述、tags、maintainers、capabilities 都来自 package 仓库自己的 manifest。不要在 catalog 里重复写这些信息。
