# 软件包与部署清单规范


Robonix 用两类清单分离“一个可复用软件包如何构建和启动”与“一台机器人选择哪些软件包、以什么参数运行”。两者都使用 `manifestVersion: 1`，但作用域不同。

| 文件 | 所有者 | 作用 | 主要命令 |
|---|---|---|---|
| `package_manifest.yaml` | 原语、服务或技能软件包 | 发布元数据、构建与启动入口、实现的能力约定、源码依赖 | `rbnx validate`、`rbnx build -p`、`rbnx start -p` |
| `robonix_manifest.yaml` | 机器人部署仓库 | 系统组件、软件包实例、目标软件包清单、实例参数和环境 | `rbnx build -f`、`rbnx boot -f` |

> **配置所有权**　共享软件包只提供实现、`config.spec` 和可复制的模板。机器人尺寸、传感器选择、坐标系、建图与 Nav2 参数等运行配置保存在机器人部署仓库，不写回上游软件包。

## 1. 软件包清单

下面是当前推荐结构。这个清单、紧随其后的目录树与三个脚本组成一组自洽的完整打包示例；`acme_chassis/main.py` 中的厂商 SDK 业务实现不在本页展开。

```yaml title="package_manifest.yaml（完整文件）"
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
  - name: robonix/primitive/chassis/move
  - name: robonix/primitive/chassis/twist_in
  - name: robonix/primitive/chassis/odom

depends: []
```

```text title="软件包目录（完整打包结构）"
primitive-acme-rover-chassis-rbnx/
├── acme_chassis/
│   ├── __init__.py
│   └── main.py
├── scripts/
│   ├── build.sh
│   ├── start.sh
│   └── stop.sh
├── config.spec
├── package_manifest.yaml
├── pyproject.toml
└── README.md
```

```bash title="scripts/build.sh"
#!/usr/bin/env bash
set -euo pipefail

python3 -m venv .venv
.venv/bin/python -m pip install --disable-pip-version-check -e .
```

```bash title="scripts/start.sh"
#!/usr/bin/env bash
set -euo pipefail

mkdir -p rbnx-build/run
export ACME_SDK_SOCKET="$PWD/rbnx-build/run/acme-chassis.sock"
exec .venv/bin/python -m acme_chassis.main
```

```bash title="scripts/stop.sh"
#!/usr/bin/env bash
set -euo pipefail

# CMD_SHUTDOWN 负责先停止设备输出和工作线程；stop 只清理包自有残留。
# rm -f 使脚本在文件不存在或被重复调用时仍然成功。
rm -f -- rbnx-build/run/acme-chassis.sock
```

三个脚本应以可执行模式提交，例如运行 `chmod 0755 scripts/build.sh scripts/start.sh scripts/stop.sh` 后确认版本库记录为 `100755`。`stop.sh` 必须幂等，并且只清理本软件包拥有的资源；它不能用宽泛的进程名杀死其它实例，也不能替代 Driver 的 `CMD_SHUTDOWN`。

:::note[示例范围]

除代码块标题明确写出“完整文件”或“完整打包结构”外，本页后续 YAML、TOML、Python 和目录树均为**局部片段**，会省略已经讲解过的必填外层字段。不要把省略项理解为可选项。第 3 节的 ACME 部署是结构模板，其中的仓库地址、凭据变量和本体参数是占位值，替换并验证之前不能直接用于真实机器人。

:::

### 1.1 字段

| 字段 | 要求 | 含义 |
|---|---|---|
| `manifestVersion` | 必填，值不小于 `1`；新清单写 `1` | 清单结构版本 |
| `package.name` | 必填 | 软件包发布名；运行时要求非空，不要求与本地目录名一致；发布到软件包目录时必须与 `catalog.yaml` 的 `packages[].name` 一致 |
| `package.version` | 必填 | 软件包版本 |
| `package.description` | 必填 | 一句话说明软件包的功能 |
| `package.license` | 必填 | SPDX 许可证标识 |
| `package.tags` | 发布时必填 | 软件包目录的分类和过滤标签 |
| `package.maintainers` | 发布时必填 | `Name <email@domain>` 字符串列表 |
| `build` | 可选 | 在软件包根目录执行的 shell 命令；预编译软件包可省略 |
| `start` | 必填 | 在软件包根目录执行的唯一启动入口 |
| `stop` | 可选 | 软件包自有的额外清理入口；不能替代 Driver 关闭流程 |
| `capabilities` | 按实现填写 | 软件包预期导出的能力元数据 |
| `depends` | 可选 | 依赖元数据；当前不自动下载或注入路径 |

验证软件包清单：

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

软件包依赖的解析、安装和构建排序仍在设计中，当前没有对应执行流程。运行时只解析并展示 `depends` 记录，不会自动克隆远程依赖、校验 `path` 与 `url` 是否互斥、加入 `PYTHONPATH`，也不会据此排序构建或启动。构建脚本仍需显式准备依赖。`depends` 也不是运行时拓扑；运行依赖通过 Atlas 能力发现与 Driver 的 `Deferred` 状态处理。

### 1.4 多平台清单

一个仓库可以为不同架构和运行方式提供多份软件包清单：

```text
package_manifest.yaml
package_manifest.x86-native.yaml
package_manifest.jetson-native.yaml
package_manifest.jetson-docker.yaml
```

每个文件都是完整的软件包清单，可以选择不同 `build`、`start`、`stop` 和由 README/build 脚本管理的平台前置条件，但软件包身份与公开能力约定应保持一致。机器人部署清单通过条目的 `manifest:` 选择具体文件；指定文件不存在时直接失败，不会回退到默认清单。

:::warning[发布多平台变体前先校验一致性]

同一软件包的所有目标清单必须保持一致的 `package.name`、`version`、发布元数据和公开能力约定。只有 `build`、`start`、`stop` 与平台前置条件可以随目标改变。提交目录前，用 `rbnx validate <package-dir>` 校验默认清单；再在代表性部署清单中逐一选择各目标变体，执行 `rbnx build -f <deploy-manifest>` 并核对能力列表。

:::

## 2. 配置说明文件

### 2.1 `config.spec`

`config.spec` 位于软件包根目录，说明部署条目中 `config:` 接受什么。它是人类和模型可读的 YAML 说明，不是 Robonix 运行时模式定义，也不会替提供方做校验。

```yaml
config:
  # string, default: can0.
  # SocketCAN interface. It must exist and be UP before CMD_INIT.
  can_port: can0

  # float, seconds, default: 30.0; must be > 0.
  # Maximum wait for the first valid hardware feedback frame.
  sentinel_timeout_s: 30.0
```

每个公开字段应写明类型、单位、默认值、范围、用途、失败条件和示例。没有配置的软件包明确写 `config: {}`。提供方仍须在 `on_init` 中验证输入并返回可定位的错误。`template-rbnx` 给出了原语、服务和技能三种写法。

### 2.2 `CAPABILITY.md`

`CAPABILITY.md` 是可选的提供方使用手册，适合描述调用顺序、约束、长任务的状态与取消语义和典型错误。文件存在时，Robonix API 在注册阶段读取其内容并交给 Atlas；Pilot 通过提供方 ID 调用 `read_capability_doc` 按需读取，不依赖共享文件系统路径。

推荐 frontmatter 只保留一句摘要：

```markdown
---
description: Navigate a mobile robot to a map-frame pose and expose status/cancel.
---

# Navigation

正文说明调用条件、参数、状态和恢复方式。
```

提供方的 ID、类型和命名空间由 Atlas 注册信息负责，不在 Markdown 中重复维护。

## 3. 机器人部署清单

`robonix_manifest.yaml` 是一台机器人或一个仿真场景的启动入口。下面的 `acme` 仓库地址是结构占位符，接入时必须替换为实际发布的软件包及其目标清单；示例中的实例名保持自洽，展示 Mapping 和 Navigation 如何绑定同一组传感器提供方：

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

  - name: front_camera
    url: https://github.com/acme/primitive-acme-rover-camera-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config: {}

  - name: front_lidar
    url: https://github.com/acme/primitive-acme-rover-lidar-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config: {}

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
        lidar2d: front_lidar
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
        scan: front_lidar

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
| `catalog` | 机器人部署仓库的发布元数据；进入软件包目录时六个字段都必须填写 |
| `env` | YAML 读入后先导出到 `rbnx` 进程环境，再用于展开清单中的 `$VAR` / `${VAR}`；启动的子进程继承这些变量 |
| `system` | Robonix 系统组件及其配置 |
| `primitive` | 直接连接硬件或仿真设备的软件包实例 |
| `service` | Mapping、Navigation、Speech、Memory 等服务实例 |
| `skill` | 模型可调用的技能实例；boot 后通常保持 `INACTIVE`，按需激活 |

`${ROBONIX_SOURCE_PATH}` 由 `rbnx setup` 登记的源码根目录提供。部署清单内的 `path`、Soma 的 `robot_yaml` 等部署路径以清单所在目录为基准，不需要额外的部署目录环境变量。

### 3.2 软件包实例字段

| 字段 | 要求 | 含义 |
|---|---|---|
| `name` | 必填 | 实例名、期望提供方 ID 和日志标签 |
| `path` / `url` | 二选一 | 本地软件包路径，或 Git 仓库 URL |
| `branch` | `url` 可选 | branch 或 tag；省略时取远端默认分支。当前不承诺任意 commit SHA |
| `manifest` | 可选 | 选择软件包内的目标清单文件 |
| `config` | 可选，默认 `{}` | 实例配置；通过该提供方唯一的共享生命周期 Driver 的 `CMD_INIT` 发送 |

`path` 相对机器人部署清单目录解析。使用 `url` 的软件包在构建时克隆到 `rbnx-boot/cache/<repository-name>/`；缓存目录来自 URL 的仓库名，不来自实例 `name`。同一仓库的多个实例复用源码检出，但各自启动独立进程并接收独立配置。

`rbnx boot` 设置 `RBNX_INSTANCE_NAME=<name>`。软件包代码应使用 Robonix API 的实例 ID 处理或该变量，使同一软件包可注册为 `front_camera`、`rear_camera` 等不同提供方；不要在 `config` 中再发明 `provider_id` 字段。

### 3.3 `config` 传递

`rbnx boot` 将该实例的 `config` 序列化为 `CMD_INIT.config_json`，并通过该提供方唯一的共享 `robonix/lifecycle/driver` 发送。Robonix API 把配置解析为 `dict` 传给 handler：

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

单个软件包调试使用同一条配置路径：

```bash
rbnx start -p ./services/example \
  --endpoint 127.0.0.1:50051 \
  --config ./local-config.yaml \
  --set timeout_s=30
```

`--set` 覆盖配置文件，值会尽量按 JSON 解析。提供方不应通过私有环境变量重复接收同一实例参数；环境变量保留给进程级选择和敏感信息。

Scene 使用同一条共享 Driver 配置通道。它的目标清单和实例配置都写在机器人部署清单中：

```yaml
system:
  scene:
    manifest: package_manifest.jetson-native.yaml
    config:
      camera_provider_id: front_camera
      web_port: 50107
```

`rbnx boot` 通过 `robonix/lifecycle/driver` 的 `CMD_INIT` 发送 `config`。旧的扁平 `system.scene` 字段和 `RBNX_CONFIG_FILE` 仅用于已有部署迁移，并会产生弃用提示；新部署不再创建 `config/scene.yaml`。

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

导航服务同样从部署目录解析相对 `params_file`。该文件必须包含完整的 Nav2 参数。使用自定义行为树时，将 `bt_xml_file` 指向部署仓库内的 XML，并在 Nav2 参数文件对应的 BehaviorTree 路径中写入 `__ROBONIX_BT_XML__`；导航服务只会在该占位符出现时注入 `bt_xml_file`，仅设置字段不会切换行为树。使用三维雷达时，通过 `provider_ids.scan_cloud` 绑定点云提供方，并显式配置 `scan_projection`；原生二维激光雷达则使用 `provider_ids.scan`，不需要投影配置。

`params_profile`、`rtabmap_profile` 等旧字段只用于已有部署迁移。兼容代码会继续读取已知旧值并打印迁移提示；新部署应保存本体参数文件并使用 `params_file`。

完整配置面以所用软件包版本的 README、`config.spec` 和实际解析代码为准；没有 `config.spec` 时，应先向上游补齐说明，再把字段写入部署手册。

## 4. 生命周期与启动责任

每个原语、服务和技能提供方都使用一条生命周期 Driver 接收部署配置和状态命令。新软件包不需要在清单中声明 Driver；框架会自动选择并注册共享的 `robonix/lifecycle/driver`。显式声明共享 Driver仍受支持，但运行行为相同。一个提供方始终只有一个生命周期入口。

```text
CMD_INIT       parse and validate instance config
CMD_ACTIVATE   acquire hot resources and begin serving
CMD_DEACTIVATE release hot resources while remaining registered
CMD_SHUTDOWN   stop workers, child processes and hardware output
```

原语或服务在启动时依次接收 `CMD_INIT` 和 `CMD_ACTIVATE`；技能完成 `CMD_INIT` 后保持 `INACTIVE`，第一次需要时再由 Executor 激活。生命周期回调可以暂时不实现：框架会为缺失的回调记录 warning，并执行安全的空操作；缺少 `on_init` 和 `on_activate` 的原语或服务仍会在两条命令后进入 `ACTIVE`。回调显式返回错误时，启动仍应失败。重复 `ACTIVATE` 必须幂等；实现了 `on_shutdown` 时，它必须停止软件包启动的子进程和设备输出。

单独执行 `rbnx start` 时，`rbnx` 使用共享 `robonix/lifecycle/driver`。它发送 `CMD_INIT`，并对非技能提供方继续发送 `CMD_ACTIVATE`；技能停在 `INACTIVE`。清单省略 Driver 条目时，省略的是清单声明，不是运行时 Driver。由 `rbnx boot` 管理时，这段生命周期由启动流程与 Soma 统一承担，避免 `rbnx start` 重复发送命令。

`rbnx boot` 的配置与进程责任如下：

1. 把部署清单读为通用 YAML 值，应用 `env`，递归展开所有标量中的 `$VAR` / `${VAR}`，再解码为部署字段；
2. 启动系统组件；
3. 通过 Soma 或 `rbnx start` 启动软件包，等待它向 Atlas 注册；
4. 确认提供方只注册了共享 Driver，发送 `CMD_INIT(config_json)` 并等待目标状态；
5. 将系统和提供方日志写入部署目录的 `rbnx-boot/logs/`；
6. 收到 Ctrl-C 或 `rbnx shutdown` 后按生命周期关闭整套部署。

不要依赖列表顺序替代真实依赖处理。能力消费者应按能力约定和指定提供方 ID 从 Atlas 发现端点；依赖暂未进入 `ACTIVE` 时返回可恢复的延迟状态。

### 4.1 新软件包自动使用共享 Driver

新软件包只列出它实际实现的业务能力约定。清单中没有 Driver 条目时，框架会自动选择并注册 `robonix/lifecycle/driver`；软件包不创建 Driver TOML：

```yaml title="package_manifest.yaml"
capabilities:
  - name: robonix/primitive/chassis/odom
```

按需要实现 `on_init`、`on_activate`、`on_deactivate` 和 `on_shutdown`；暂未实现的回调由框架 warning 后执行空操作。软件包也可以显式加入 `name: robonix/lifecycle/driver`，其行为与自动选择相同。新清单不得声明其他 Driver，也不能包含多条 Driver 约定。

### 4.2 已有命名空间 Driver 的兼容流程

:::warning[后向兼容：已有命名空间 Driver]
早期软件包通常把 Driver TOML 保存在自己的仓库中，并使用 `<provider-namespace>/driver` 作为能力约定 ID。这种完整的旧实现目前仍可构建和启动，但计划逐步迁移到共享 Driver。维护旧包时保留原有 Driver 条目和 TOML，不要同时追加 `robonix/lifecycle/driver`。

兼容握手只接受精确的主命名空间旧 ID，并分成两种合法结果：旧 Servicer 与注册函数完整存在时继续注册旧 Driver；旧生成服务完全不存在而共享服务完整存在时，由 `rbnx` 或 Soma 设置的兼容标记允许单向改用共享 Driver，并输出迁移警告。部分旧服务、部分共享服务、无关 ID、零条或多条 Driver 都拒绝启动。
:::

以下内容仅用于维护和迁移已有软件包。新软件包直接按 4.1 节使用共享 Driver。

例如，已有底盘原语可能包含：

```text
primitive-acme-chassis-rbnx/
├── capabilities/
│   └── driver.v1.toml
├── scripts/
│   ├── build.sh
│   ├── start.sh
│   └── stop.sh              # 仅在有额外外部资源时需要
├── acme_chassis/
│   ├── __init__.py
│   └── main.py
├── config.spec              # 推荐：记录实例 config 字段
├── package_manifest.yaml
└── README.md
```

```toml title="capabilities/driver.v1.toml"
[contract]
id = "robonix/primitive/chassis/driver"
version = "1"
kind = "primitive"
idl = "lifecycle/srv/Driver.srv"
description = "Lifecycle control for this legacy chassis provider."

[mode]
type = "rpc"
```

```yaml title="package_manifest.yaml"
capabilities:
  - name: robonix/primitive/chassis/driver
    path: capabilities/driver.v1.toml
  - name: robonix/primitive/chassis/odom
```

旧包的 Python 入口不需要为了 Book 更新而重写。当前 Robonix API 优先绑定清单精确选择且完整存在的旧生成服务；如果该旧服务完全不存在，并且受管启动明确标记这是旧清单迁移，它才绑定完整的共享生成服务并向 Atlas 注册 `robonix/lifecycle/driver`：

```python title="acme_chassis/main.py"
from robonix_api import Ok, Primitive

chassis = Primitive(
    id="base_chassis",
    namespace="robonix/primitive/chassis",
)

@chassis.on_init
def init(config: dict):
    print(f"received config: {config}")
    return Ok()

@chassis.on_shutdown
def shutdown():
    return Ok()

if __name__ == "__main__":
    chassis.run()
```

使用其他语言或自行实现 gRPC 服务时，最直接的兼容方式仍是注册本地 TOML 所引用的 `lifecycle/srv/Driver.srv` 服务，并向 Atlas 声明完全相同的旧 Driver ID。若实现选择共享运行时升级，只能在 `rbnx` 或 Soma 根据精确旧清单启用兼容标记的受管启动中注册唯一的 `robonix/lifecycle/driver`；不能把这个方向反过来，也不能同时声明新旧两条 Driver。

不使用正向升级时，兼容流程要求以下四处一致：

1. `driver.v1.toml` 的 `contract.id`；
2. `package_manifest.yaml` 中 Driver 的 `name` 和 `path`；
3. 提供方主命名空间与旧 ID 的前缀，例如 `robonix/primitive/chassis` 对应 `robonix/primitive/chassis/driver`；
4. 代码生成的 gRPC 服务和运行时向 Atlas 注册的 Driver ID。

正向升级是唯一例外：前 3 项仍精确指向 `<provider-namespace>/driver`，旧生成服务必须完全不存在，完整共享服务改为注册 `robonix/lifecycle/driver`，而启动器必须显式带上旧清单兼容标记。直接调用 SDK、继承到一个环境变量、或仅发现共享 stub 都不能自行取得升级权限。

不要在保留旧 Driver 的同时显式声明 `robonix/lifecycle/driver`。启动器要求每个提供方只有一个生命周期入口；两条 Driver 会直接导致启动失败。维护旧软件包时仍应保留现有生命周期回调和本地 TOML，不要只修改清单名称。历史环境变量 `ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK=1` 的名称容易误导；当前它只表示启动器已确认“精确旧清单可以正向使用共享运行时”，绝不允许共享选择降级到旧 Driver。

已有软件包无需立即迁移。先在原结构上验证兼容链路：

```yaml title="local-config.yaml"
odom_topic: /odom
```

终端 A：

```bash
rbnx validate ./primitive-acme-chassis-rbnx
rbnx build -p ./primitive-acme-chassis-rbnx
rbnx start -p ./primitive-acme-chassis-rbnx --config ./local-config.yaml
```

保持终端 A 运行，在终端 B 检查该提供方和 warning：

```bash
rbnx caps -v | rg -A 8 -B 2 'base_chassis|robonix/(primitive/chassis|lifecycle)/driver'
rbnx logs -l warn | rg 'legacy manifest|shared runtime Driver|lifecycle contract|deprecated|migration'
```

验收时 `base_chassis` 必须只注册一条 Driver：旧生成服务仍完整时为 `robonix/primitive/chassis/driver`；旧服务完全不存在且共享服务完整时，可以是 `robonix/lifecycle/driver`，并伴随“旧清单 → 共享运行时”的定向迁移 warning。两种情况下 `on_init` 都应收到 `odom_topic: /odom`，原语或服务最终进入 `ACTIVE`。技能在 `CMD_INIT` 后保持 `INACTIVE`，首次调用时再激活。出现旧清单却没有兼容标记、部分生成服务、共享选择降级、零条或两条 Driver 时都不得验收。

若要验证整机关闭路径，把该包放入最小部署清单后在两个终端分别运行 `rbnx boot -f robonix_manifest.yaml` 与 `rbnx shutdown -f robonix_manifest.yaml`，确认 `on_shutdown` 和可选 `stop` 都被调用。单独的前台 `rbnx start` 不能替代这项关闭验收。

### 4.3 可选迁移到自动共享 Driver

需要迁移时按以下顺序操作：

1. 删除清单中的旧 Driver 条目，不再添加另一条 Driver；
2. 删除软件包内只为生命周期准备的 `driver.v1.toml`；
3. 使用当前 Robonix 重新运行代码生成并构建软件包；
4. 保留原有生命周期回调，不修改实例 `config` 的字段含义；
5. 启动后确认框架自动注册了唯一的共享 Driver，并验证初始化、激活和关闭。

终端 A：

```bash
rbnx validate ./path/to/package
rbnx build -p ./path/to/package
rbnx start -p ./path/to/package --config ./local-config.yaml
```

保持终端 A 运行，在终端 B 检查：

```bash
rbnx caps -v | rg -A 8 -B 2 'base_chassis|robonix/.+/driver'
```

迁移成功时，该提供方只应显示 `robonix/lifecycle/driver`，清单中也不再有旧 Driver。只显示共享 Driver 但仍出现“旧清单 → 共享运行时”warning，表示当前仍处于兼容桥接阶段而不是清单迁移完成；仍显示命名空间 Driver 则表示旧生成服务和旧清单仍在生效。

## 5. 约束与验收

以下规则保证软件包可以跨机器人复用：

- 跨软件包数据通过 Atlas 能力约定与端点绑定；通用服务不硬编码某台机器的话题名。
- 设备专属的坐标系、消息修正和 SDK 启动留在原语；建图与 Nav2 不包含 `if robot == ...`。
- `package_manifest.yaml::capabilities` 只列运行时真实声明的接口。
- 同一服务的不同算法后端维持相同的公开能力约定，后端差异通过内部适配层消化。
- 同一软件包的目标清单必须保持一致的 `package.name`、`version` 和公开能力约定；存在身份漂移时阻止发布。
- 敏感信息由操作者环境提供，不把真实值提交到清单或 `.env.example`。

机器人支持不能由“包已编译”或“进程还在运行”单独判定。下面的命令均来自当前 [`rbnx` 子命令定义](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/tools/rbnx/src/cmd/mod.rs)；从机器人部署目录按顺序保存清单、构建、启动、Atlas 和日志证据。

### 5.1 清单与构建

先核对 `robonix_manifest.yaml` 中每个 `name`、`path` / `url`、`branch`、`manifest` 与 `config`，再打开其实际选中的 `package_manifest*.yaml`，确认公开能力约定和运行时实现一致。对本地软件包验证默认清单，并构建单包与整机所选目标：

```bash
rbnx validate ./path/to/package
rbnx build -p ./path/to/package
rbnx build -f ./robonix_manifest.yaml
```

`rbnx validate` 只验证软件包默认清单；`rbnx build -f` 还必须成功读取并构建部署条目通过 `manifest:` 选择的平台变体。任一目标清单缺失或构建失败都阻止验收。

### 5.2 启动、发现与日志

在终端 A 前台启动并显示追加式日志：

```bash
rbnx boot -f ./robonix_manifest.yaml --verbose
```

`rbnx boot` 保持在前台并等待关闭信号，结构化日志写入清单目录的 `rbnx-boot/logs/`。保持终端 A 运行，在终端 B 检查 Atlas 和日志：

```bash
rbnx caps -v
rbnx logs -d ./rbnx-boot/logs --list-tags
rbnx logs -d ./rbnx-boot/logs -l warn
```

验收结果必须同时满足：

- 每个部署实例以清单 `name` 对应的 `provider_id` 出现；
- 原语和服务完成初始化、激活并进入 `ACTIVE`，技能可在初始化后保持 `INACTIVE`；
- Atlas 展开的能力约定、传输与所选包清单及接入设计一致；
- 日志出现各传感器或服务的就绪证据，且没有未处理的 Driver、首帧、标定或依赖错误。

### 5.3 无运动功能测试与关闭

先验证不会驱动执行器的真实功能，例如相机快照、状态读取或传感器采样，并核对返回载荷、时间戳和坐标系。若部署包含 Pilot、Executor、提供方 `front_camera` 与 `robonix/primitive/camera/snapshot`，可运行下面的限定提示验证完整调用链；其它机器人必须改用它实际声明的只读能力，不能照抄不存在的接口。

```bash
rbnx ask '只调用 provider_id=front_camera 的 robonix/primitive/camera/snapshot 获取一帧；不要执行任何运动。'
```

长时间调用进行期间，可在另一个终端运行 `rbnx channels` 检查 Atlas 记录的消费关系。首次上机验收不要从底盘移动或机械臂动作开始；只有只读路径、急停和现场安全边界确认后，才能按厂商测试计划逐步放开有限动作。

最后从终端 B 关闭同一份部署清单：

```bash
rbnx shutdown -f ./robonix_manifest.yaml
```

关闭成功必须触发 Driver `CMD_SHUTDOWN` 和可选的幂等 `stop`，移除该部署的 `rbnx-boot/state.json`，且不遗留设备输出或软件包子进程错误。详细运行链路和 Atlas 通道语义见[运行时通信](../architecture/runtime-communication.md)与 [Atlas 能力目录](../architecture/atlas.md)。

软件包发布与整机目录元数据见[软件包目录发布流程](package-catalog.md)；从硬件 SDK 到完整机器人部署的逐步流程见[机器人本体接入指南](vendor-onboarding.md)。
