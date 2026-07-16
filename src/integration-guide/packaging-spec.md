# 包与部署清单规范

[toc]

Robonix 用两类 manifest 分离“一个可复用 package 如何构建/启动”和“一台机器人选择哪些 package、以什么参数运行”。两者都使用 `manifestVersion: 1`，但作用域不同。

| 文件 | 所有者 | 作用 | 主要命令 |
|---|---|---|---|
| `package_manifest.yaml` | Primitive / Service / Skill package | 发布元数据、构建/启动入口、实现的 contract、源码依赖 | `rbnx validate`、`rbnx build -p`、`rbnx start -p` |
| `robonix_manifest.yaml` | Robot deployment | 系统组件、package 实例、目标 manifest、实例参数和环境 | `rbnx build -f`、`rbnx boot -f` |

> **配置所有权**　共享 package 只提供实现、`config.spec` 和可复制的模板。机器人尺寸、传感器选择、frame、Mapping/Nav2 参数等运行配置保存在 robot deployment 仓库，不写回上游 package。

## 1. Package manifest

下面是当前推荐结构：

```yaml
manifestVersion: 1

package:
  name: robonix.primitive.acme.rover.chassis
  version: 0.1.0
  description: ACME Rover chassis driver.
  tags: [primitive, chassis, acme, rover]
  maintainers:
    - Your Name <you@example.com>
  license: Apache-2.0

build: bash scripts/build.sh
start: bash scripts/start.sh
stop: bash scripts/stop.sh

capabilities:
  - name: robonix/primitive/chassis/driver
  - name: robonix/primitive/chassis/move
  - name: robonix/primitive/chassis/twist_in
  - name: robonix/primitive/chassis/odom

depends: []
```

### 1.1 字段

| 字段 | 要求 | 含义 |
|---|---|---|
| `manifestVersion` | 必填，当前为 `1` | manifest 结构版本 |
| `package.name` | 必填 | package 发布名；进入 Catalog 时必须与 Catalog name 一致 |
| `package.version` | 必填 | package 版本 |
| `package.description` | 必填 | 一句话说明 package 的功能 |
| `package.license` | 必填 | SPDX license 标识 |
| `package.tags` | 发布时必填 | Catalog 分类和过滤标签 |
| `package.maintainers` | 发布时必填 | `Name <email@domain>` 字符串列表 |
| `build` | 可选 | 在 package 根目录执行的 shell；预编译 package 可省略 |
| `start` | 必填 | 在 package 根目录执行的唯一启动入口 |
| `stop` | 可选 | package 自有的额外清理入口；不能替代 Driver shutdown |
| `capabilities` | 按实现填写 | package 启动后实际声明的 contract |
| `depends` | 可选 | 构建/导入所需的 package 源码依赖，不表示启动顺序 |

`package.vendor`、`package.id`、`nodes[]`、`build: { script: ... }` 和 package 内旧文件名 `robonix_manifest.yaml` 仍被 `dev-next` 读取，以保证旧 package 可以启动；`rbnx` 会打印迁移 warning。新 package 使用上面的结构。

验证 manifest：

```bash
rbnx validate ./path/to/package
```

成功时输出：

```text
✓ Manifest validation passed
```

### 1.2 `capabilities`

标准 Primitive、Service 和 System contract 来自 Robonix 源码树。package 复用标准接口时只写 contract id：

```yaml
capabilities:
  - name: robonix/primitive/camera/rgb
  - name: robonix/primitive/camera/depth
```

当 package 确实定义尚未进入主仓库的接口时，可用 `path` 指向 package 本地 TOML：

```yaml
capabilities:
  - name: robonix/skill/inventory/count
    path: capabilities/count.v1.toml
```

本地 contract 引用的 IDL 放在同一 package 的 `capabilities/lib/`。`rbnx codegen` 会同时解析官方和 package 本地 contract。未实现的接口不能预先写进 `capabilities`；manifest 声明必须与 Atlas 运行时声明一致。

### 1.3 `depends`

`depends` 描述构建或导入依赖：

```yaml
depends:
  - name: robonix.library.example
    path: ../example
  - name: robonix.library.remote
    url: https://github.com/example/remote-rbnx
    branch: main
```

每项必须有 `name`，`path` 和 `url` 二选一；两者都省略表示依赖已安装或在 `PYTHONPATH`。它不是运行时拓扑，不能用来表达“Mapping 必须先于 Navigation ACTIVE”。运行依赖通过 Atlas contract 发现与 Driver 的 `Deferred` 状态处理。

### 1.4 多平台 manifest

一个仓库可以为不同架构和运行方式提供多个 manifest：

```text
package_manifest.yaml
package_manifest.x86-native.yaml
package_manifest.jetson-native.yaml
package_manifest.jetson-docker.yaml
```

每个文件都是完整 package manifest，可以选择不同 `build`、`start` 和系统依赖，但 package identity 与 contract 面应保持一致。Robot deployment 通过条目的 `manifest:` 选择具体文件；指定文件不存在时直接失败，不会回退到默认 manifest。

## 2. 配置说明文件

### 2.1 `config.spec`

`config.spec` 位于 package 根目录，说明 deployment 条目中 `config:` 接受什么。它是人类和模型可读的 YAML 说明，不是 Robonix 运行时 schema，也不会替 provider 做校验。

```yaml
config:
  # string, default: can0.
  # SocketCAN interface. It must exist and be UP before CMD_INIT.
  can_port: can0

  # float, seconds, default: 30.0; must be > 0.
  # Maximum wait for the first valid hardware feedback frame.
  sentinel_timeout_s: 30.0
```

每个公开字段应写明类型、单位、默认值、范围、用途、失败条件和示例。没有配置的 package 明确写 `config: {}`。Provider 仍须在 `on_init` 中验证输入并返回可定位的错误。`template-rbnx` 给出了 Primitive、Service 和 Skill 的三种写法。

### 2.2 `CAPABILITY.md`

`CAPABILITY.md` 是可选的 provider 使用手册，适合描述调用顺序、约束、长任务的 status/cancel 语义和典型错误。文件存在时，Robonix API 在注册阶段读取其内容并交给 Atlas；Pilot 通过 provider id 调用 `read_capability_doc` 按需读取，不依赖共享文件系统路径。

推荐 frontmatter 只保留一句摘要：

```markdown
---
description: Navigate a mobile robot to a map-frame pose and expose status/cancel.
---

# Navigation

正文说明调用条件、参数、状态和恢复方式。
```

Provider 的 id、kind 和 namespace 由 Atlas 注册信息负责，不在 Markdown 中重复维护。

## 3. Robot deployment manifest

`robonix_manifest.yaml` 是一台机器人或一个仿真场景的启动入口：

```yaml
manifestVersion: 1
name: robonix-acme-rover

catalog:
  name: robonix.robot.acme.rover
  version: 0.1.0
  description: Robonix deployment for ACME Rover.
  license: Apache-2.0
  tags: [robot, deploy, acme, rover]
  maintainers:
    - Your Name <you@example.com>

env:
  ROS_DOMAIN_ID: "0"
  RMW_IMPLEMENTATION: rmw_zenoh_cpp

system:
  atlas:
    listen: 0.0.0.0:50051
  soma:
    listen: 127.0.0.1:50091
    robot_yaml: ${ROBONIX_DEPLOY_DIR}/soma.yaml
  vitals:
    listen: 0.0.0.0:50093
  executor:
    listen: 0.0.0.0:50061
  pilot:
    listen: 127.0.0.1:50071
    vlm:
      upstream: ${VLM_BASE_URL}
      api_key: ${VLM_API_KEY}
      model: ${VLM_MODEL}
  liaison:
    listen: 0.0.0.0:50081
  scene:
    camera_provider_id: front_camera

primitive:
  - name: base_chassis
    url: https://github.com/acme/primitive-acme-rover-chassis-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config:
      can_port: can0

service:
  - name: memory
    path: ${ROBONIX_SOURCE_PATH}/services/memsearch
    config: {}

  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config:
      params_file: config/rtabmap_params.yaml
      sensor_providers:
        lidar3d: roof_lidar
        odom: base_chassis
        rgb: front_camera
        depth: front_camera

  - name: nav2
    url: https://github.com/syswonder/service-navigation-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config:
      params_file: config/nav2_params.yaml
      provider_ids:
        map: mapping
        odom: base_chassis
        scan_cloud: roof_lidar

skill:
  - name: explore
    url: https://github.com/syswonder/skill-explore-rbnx
    branch: main
    config: {}
```

### 3.1 顶层字段

| 字段 | 含义 |
|---|---|
| `name` | 这次整机部署的运行名称 |
| `catalog` | Robot deployment 发布元数据；进入 Catalog 时六个字段都必须填写 |
| `env` | 在解析 manifest 前展开，并传给 `rbnx` 启动的子进程 |
| `system` | Robonix 系统组件及其配置 |
| `primitive` | 直接连接硬件或仿真设备的 package 实例 |
| `service` | Mapping、Navigation、Speech、Memory 等服务实例 |
| `skill` | 模型可调用的技能实例；boot 后通常保持 `INACTIVE`，按需激活 |

`${ROBONIX_SOURCE_PATH}` 由 `rbnx setup` 登记的源码根目录提供；`${ROBONIX_DEPLOY_DIR}` 是当前 deployment manifest 所在目录。两者用于避免不可移植的绝对路径。

### 3.2 Package instance 字段

| 字段 | 要求 | 含义 |
|---|---|---|
| `name` | 必填 | 实例名、期望 provider id 和日志标签 |
| `path` / `url` | 二选一 | 本地 package 路径，或 Git 仓库 URL |
| `branch` | `url` 可选 | branch、tag 或 commit；省略时取远端默认分支 |
| `manifest` | 可选 | 选择 package 内的目标 manifest 文件 |
| `config` | 可选，默认 `{}` | Driver `CMD_INIT` 的实例配置 |

`path` 相对 deployment manifest 目录解析。`url` package 构建时克隆到 `rbnx-boot/cache/<repository-name>/`；缓存目录来自 URL 的仓库名，不来自实例 `name`。同一仓库的多个实例复用源码 checkout，但各自启动独立进程并接收独立配置。

`rbnx boot` 设置 `RBNX_INSTANCE_NAME=<name>`。Package 代码应使用 Robonix API 的实例 id 处理或该变量，使同一 package 可注册为 `front_camera`、`rear_camera` 等不同 provider；不要在 `config` 中再发明 `provider_id` 字段。

### 3.3 `config` 传递

`rbnx boot` 将每个实例的 `config` 序列化为 Driver `CMD_INIT.config_json`。Robonix API 把它解析为 `dict` 传给 handler：

```python
from robonix_api import Err, Ok, Service

mapping = Service(id="mapping", namespace="robonix/service/map")

@mapping.on_init
def init(cfg: dict):
    params_file = cfg.get("params_file")
    if not params_file:
        return Err("params_file is required")
    return Ok()
```

单 package 调试使用同一条配置路径：

```bash
rbnx start -p ./services/example \
  --endpoint 127.0.0.1:50051 \
  --config ./local-config.yaml \
  --set timeout_s=30
```

`--set` 覆盖配置文件，值会尽量按 JSON 解析。Provider 不应通过私有环境变量重复接收同一实例参数；环境变量保留给进程级选择和 secret。

### 3.4 部署拥有算法参数

Mapping 和 Navigation 的 `params_file` 都相对 deployment manifest 目录解析：

```text
robot-acme-rover/
├── robonix_manifest.yaml
└── config/
    ├── rtabmap_params.yaml
    └── nav2_params.yaml
```

Mapping 上游的 `config/rtabmap_params.template.yaml` 和 Navigation 上游的参数文件只作为复制起点，运行时不会自动加载。新部署必须把修改后的完整文件提交到机器人仓库。Mapping 还允许 `rtabmap_params` 在 manifest 中做少量最终覆盖。

兼容字段仍能启动并发出 warning：

| 旧字段 | 新字段 |
|---|---|
| Mapping `rtabmap_profile` | deployment 自有 `params_file`，必要时加 `rtabmap_params` |
| Mapping `sensors` | `sensor_providers` |
| Navigation `params_profile` | deployment 自有 `params_file` |
| Navigation flat scan aliases | `scan_projection.*` |

完整配置面以每个上游仓库根目录 `config.spec` 为准；本页不复制所有算法参数，避免与实现漂移。

## 4. 生命周期与启动责任

所有 Primitive、Service 和 Skill 都通过同类 Driver contract 接收生命周期命令：

```text
CMD_INIT       parse and validate config, resolve dependencies
CMD_ACTIVATE   acquire hot resources and begin serving
CMD_DEACTIVATE release hot resources while remaining registered
CMD_SHUTDOWN   stop workers, child processes and hardware output
```

Primitive/Service 在 boot 时走向 `ACTIVE`；Skill 完成 `CMD_INIT` 后通常保持 `INACTIVE`，第一次需要时再激活。重复 `ACTIVATE` 必须幂等；`CMD_SHUTDOWN` 必须停止 package 启动的子进程和设备输出。

`rbnx boot` 的配置与进程责任如下：

1. 读取 manifest，应用 `env` 并展开字符串变量；
2. 启动系统组件；
3. 通过 Soma/`rbnx start` 启动 package，等待它向 Atlas 注册；
4. 找到该 provider 的 Driver，发送 `CMD_INIT(config_json)` 并等待目标状态；
5. 将系统和 provider 日志写入 deployment 的 `rbnx-boot/logs/`；
6. 收到 Ctrl-C 或 `rbnx shutdown` 后按生命周期关闭整套部署。

不要依赖列表顺序替代真实依赖处理。Consumer 应按 contract 和指定 provider id 从 Atlas 发现 endpoint；依赖暂未 ACTIVE 时返回可恢复的 deferred 状态。

## 5. 约束与验收

以下规则保证 package 可以跨机器人复用：

- 跨 package 数据通过 Atlas contract 与 endpoint 绑定；通用 Service 不硬编码某台机器的话题名。
- 设备专属的 frame、消息修正和 SDK 启动留在 Primitive；Mapping/Nav2 不包含 `if robot == ...`。
- `package_manifest.yaml::capabilities` 只列运行时真实声明的接口。
- 同一 Service 的不同算法后端维持相同的公开 contract，后端差异通过内部 adapter 消化。
- Secrets 由操作者环境提供，不提交到 manifest 或 `.env.example` 的真实值。

提交前执行：

```bash
rbnx validate ./path/to/package
rbnx build -p ./path/to/package
rbnx build -f ./robonix_manifest.yaml
rbnx boot -f ./robonix_manifest.yaml
rbnx caps -v
rbnx shutdown
```

Package 发布与整机 Catalog metadata 见 [Package Catalog 发布流程](package-catalog.md)；从硬件 SDK 到完整机器人部署的逐步流程见[机器人本体接入指南](vendor-onboarding.md)。
