# 包与部署清单规范


Robonix 用两类 manifest 分离“一个可复用 package 如何构建/启动”和“一台机器人选择哪些 package、以什么参数运行”。两者都使用 `manifestVersion: 1`，但作用域不同。

| 文件 | 所有者 | 作用 | 主要命令 |
|---|---|---|---|
| `package_manifest.yaml` | Primitive / Service / Skill package | 发布元数据、构建/启动入口、实现的 contract、源码依赖 | `rbnx validate`、`rbnx build -p`、`rbnx start -p` |
| `robonix_manifest.yaml` | Robot deployment | 系统组件、package 实例、目标 manifest、实例参数和环境 | `rbnx build -f`、`rbnx boot -f` |

> **配置所有权**　共享 package 只提供实现、`config.spec` 和可复制的模板。机器人尺寸、传感器选择、frame、Mapping/Nav2 参数等运行配置保存在 robot deployment 仓库，不写回上游 package。

## 1. 软件包清单

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
| `manifestVersion` | 必填，值不小于 `1`；新清单写 `1` | 清单结构版本 |
| `package.name` | 必填 | package 发布名；进入 Catalog 时必须与 Catalog name 一致 |
| `package.version` | 必填 | package 版本 |
| `package.description` | 必填 | 一句话说明 package 的功能 |
| `package.license` | 必填 | SPDX license 标识 |
| `package.tags` | 发布时必填 | Catalog 分类和过滤标签 |
| `package.maintainers` | 发布时必填 | `Name <email@domain>` 字符串列表 |
| `build` | 可选 | 在 package 根目录执行的 shell；预编译 package 可省略 |
| `start` | 必填 | 在 package 根目录执行的唯一启动入口 |
| `stop` | 可选 | package 自有的额外清理入口；不能替代 Driver shutdown |
| `capabilities` | 按实现填写 | 软件包预期导出的能力元数据 |
| `depends` | 可选 | 依赖元数据；当前不自动下载或注入路径 |

`package.id`、`nodes[]`、`build: { script: ... }` 和软件包内旧文件名 `robonix_manifest.yaml` 仍可读取，以保证旧软件包可以启动；`rbnx` 会为这些旧结构打印迁移警告。旧 `package.vendor` 字段也可以继续解析，但不参与软件包身份或 Catalog 元数据，也不会单独触发警告。新软件包使用上面的结构。

验证 manifest：

```bash
rbnx validate ./path/to/package
```

成功时输出：

```text
✓ Manifest validation passed
```

### 1.2 `capabilities`

标准原语、服务和系统能力约定来自 Robonix 源码树。软件包复用标准接口时记录能力约定 ID：

```yaml
capabilities:
  - name: robonix/primitive/camera/rgb
  - name: robonix/primitive/camera/depth
```

当软件包定义尚未进入主仓库的接口时，可用 `path` 记录软件包本地 TOML：

```yaml
capabilities:
  - name: robonix/skill/inventory/count
    path: capabilities/count.v1.toml
```

本地能力约定引用的 IDL 放在同一软件包的 `capabilities/lib/`。`rbnx codegen` 扫描官方能力约定根和软件包的 `capabilities/`，并不依赖本字段决定生成范围。运行时能力同样由代码中的装饰器和声明 API 注册，不由清单自动创建。未实现的接口不能预先写进 `capabilities`；本字段应与 Atlas 中的实际声明一致。

### 1.3 `depends`

`depends` 记录构建或导入依赖：

```yaml
depends:
  - name: robonix.library.example
    path: ../example
  - name: robonix.library.remote
    url: https://github.com/example/remote-rbnx
    branch: main
```

当前运行时只解析和展示这些记录，不会自动克隆远程依赖、校验 `path` 与 `url` 是否互斥，也不会加入 `PYTHONPATH`。构建脚本仍需显式准备依赖。它不是运行时拓扑，不能用来表达“建图必须先于导航激活”；运行依赖通过 Atlas 能力发现与 Driver 的 `Deferred` 状态处理。

### 1.4 多平台清单

一个仓库可以为不同架构和运行方式提供多个 manifest：

```text
package_manifest.yaml
package_manifest.x86-native.yaml
package_manifest.jetson-native.yaml
package_manifest.jetson-docker.yaml
```

每个文件都是完整 package manifest，可以选择不同 `build`、`start` 和系统依赖，但 package identity 与 contract 面应保持一致。Robot deployment 通过条目的 `manifest:` 选择具体文件；指定文件不存在时直接失败，不会回退到默认 manifest。

> **已知上游阻塞：** Mapping 与 Navigation 当前部分 Jetson manifest 的 `package.name`、`version` 或 contract 元数据仍与默认 manifest 不一致。这些变体在对齐前不能作为正确示例，也不能视为可发布目标。修复时必须保持同一 package identity、版本和公开 contract，仅让 `build`、`start`、`stop` 与系统依赖随目标变化。

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

## 3. 机器人部署清单

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
    robot_yaml: soma.yaml
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
    manifest: package_manifest.jetson-native.yaml

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
    manifest: package_manifest.yaml
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
    manifest: package_manifest.yaml
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
| `env` | YAML 读入后先导出到 `rbnx` 进程环境，再用于展开清单中的 `$VAR` / `${VAR}`；启动的子进程继承这些变量 |
| `system` | Robonix 系统组件及其配置 |
| `primitive` | 直接连接硬件或仿真设备的 package 实例 |
| `service` | Mapping、Navigation、Speech、Memory 等服务实例 |
| `skill` | 模型可调用的技能实例；boot 后通常保持 `INACTIVE`，按需激活 |

`${ROBONIX_SOURCE_PATH}` 由 `rbnx setup` 登记的源码根目录提供。部署清单内的 `path`、Soma 的 `robot_yaml` 等部署路径以清单所在目录为基准，不需要额外的部署目录环境变量。

### 3.2 软件包实例字段

| 字段 | 要求 | 含义 |
|---|---|---|
| `name` | 必填 | 实例名、期望 provider id 和日志标签 |
| `path` / `url` | 二选一 | 本地 package 路径，或 Git 仓库 URL |
| `branch` | `url` 可选 | branch 或 tag；省略时取远端默认分支。当前不承诺任意 commit SHA |
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

Scene 是当前例外：它没有 Driver 配置通道，`system.scene` 目前只识别 `manifest`，其他运行参数不会由 `rbnx boot` 送达。部署仓库应保存 `config/scene.yaml`，并在启动 wrapper 中设置绝对路径：

```bash
DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
export RBNX_CONFIG_FILE="$DEPLOY_DIR/config/scene.yaml"
exec rbnx boot -f "$DEPLOY_DIR/robonix_manifest.yaml" "$@"
```

长期边界是让 Scene 接入 Driver `CMD_INIT(config_json)`，或提供统一、明确的无 Driver 系统配置通道；在此之前不要把 `camera_provider_id`、`web_port` 等字段写进 `system.scene` 并假定它们会生效。

### 3.4 部署拥有算法参数

机器人专属的建图与导航参数应提交到部署仓库：

```text
robot-acme-rover/
├── robonix_manifest.yaml
└── config/
    ├── rtabmap_params.yaml
    └── nav2_params.yaml
```

Mapping 上游的 `config/rtabmap_params.template.yaml` 只作为复制起点，运行时不会自动加载。Mapping 会把相对 `params_file` 按部署目录解析，并允许 `rtabmap_params` 做少量最终覆盖。旧 `rtabmap_profile` 和 `sensors` 仍可兼容读取并输出迁移提示。

Navigation 同样从部署目录解析相对 `params_file`。该文件必须包含完整的 Nav2 参数；可选的 `bt_xml_file` 选择部署仓库内的行为树。使用三维雷达时，通过 `provider_ids.scan_cloud` 绑定点云提供者，并显式配置 `scan_projection`；原生二维激光雷达则使用 `provider_ids.scan`，不需要投影配置。

`params_profile`、`rtabmap_profile` 等旧字段只用于已有部署迁移。兼容代码会继续读取已知旧值并打印迁移提示；新部署应保存本体参数文件并使用 `params_file`。

完整配置面以所用软件包版本的 README、`config.spec` 和实际解析代码为准；没有 `config.spec` 时，应先向上游补齐说明，再把字段写入部署手册。

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

1. 把 manifest 读为通用 YAML 值，应用 `env`，递归展开所有标量中的 `$VAR` / `${VAR}`，再解码为部署字段；
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
- 同一 package 的目标 manifest 必须保持一致的 `package.name`、`version` 和公开 contract；存在 identity 漂移时阻止发布。
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
