# 系统部署与启动流程


本页解释 `rbnx build` 和 `rbnx boot` 如何把一份机器人部署清单变成正在运行的 Robonix 系统。它面向需要定位启动失败、能力未注册或配置未生效问题的开发者。

## 两层清单各自负责什么

| 文件 | 读取方 | 负责内容 |
|---|---|---|
| `<deploy>/robonix_manifest.yaml` | `rbnx build`、`rbnx boot`、Soma | 选择系统组件、原语、服务和技能实例；指定软件包来源、分支、目标清单和实例配置 |
| `<package>/package_manifest.yaml` | `rbnx build`、`rbnx start` | 定义一个软件包的元数据、构建命令、启动命令、停止命令和能力约定列表 |

原语、服务和技能部署项的 `name` 是运行时提供方 ID；非内置系统软件包使用 `system:` 下的键名。一个软件包可以在同一部署中出现多次，但每个实例必须使用不同的 ID。对于 `url:` 软件包，代码缓存目录按 Git 仓库名创建，而不是按实例名创建；多个实例可以复用同一份代码检出。

```yaml
service:
  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config:
      params_file: config/rtabmap_params.yaml
```

这里的 `manifest:` 选择该软件包目录中的完整软件包清单。构建使用其中的 `build:`，启动使用 `start:`，受管关闭流程保存并使用同一文件中的可选 `stop:`；指定文件不存在时直接失败，不会回退到默认清单。

## 构建阶段

在部署目录运行：

```bash
rbnx build -f robonix_manifest.yaml
```

`rbnx build` 会完成以下工作：

1. 解析部署清单并展开其中的环境变量。
2. 将 `url:` 软件包克隆到 `<deploy>/rbnx-boot/cache/<repository-name>/`；已有源码检出会被复用。
3. 对每个软件包读取部署项选定的软件包清单。
4. 在软件包根目录执行该清单的 `build:` 命令。
5. 构建成功后写入 `<package>/rbnx-build/.rbnx-built`。

`rbnx boot` 发现缺少源码检出或构建完成标记时会警告并尝试补做克隆和构建。这是首次启动的容错路径，不应代替显式的 `rbnx build`。

## 启动阶段

当前启动顺序由 `rbnx` 与 Soma 共同完成。

### 1. 启动内置系统组件

`rbnx boot` 按下面的固定顺序启动部署清单中声明的内置二进制：

1. Atlas
2. Executor
3. Soma
4. Vitals
5. Pilot
6. Liaison

每个内置系统块都会整体编码为 `--config-json`；当前二进制仍单独读取的字段（例如 `listen`、Atlas 地址和 Pilot 的 VLM 配置）还会转换为对应命令行参数。监听端口已被占用或 Pilot 必填参数为空时，`rbnx boot` 会在启动该组件前失败，避免连接到遗留进程。

#### Soma 本体描述与进程配置 \{#soma-sidecar-与进程配置}

部署目录中的 `soma.yaml` 描述机器人本体，`robonix_manifest.yaml` 中的 `system.soma` 只配置 Soma 进程。只要部署包含 `primitive:` 或 `skill:`，即使没有写 `system.soma`，`rbnx boot` 也会自动补出该系统块。同目录存在 `soma.yaml` 时，启动器会自动把它作为本体描述传给 Soma，不需要在清单中重复填写路径。

正常部署通常只需要按网络可达性覆盖监听地址和日志级别：

```yaml
system:
  soma:
    listen: 0.0.0.0:50091
    log: info
```

只有本体描述文件不叫 `soma.yaml` 时，才在 `system.soma.robot_yaml` 中写相对部署目录的路径。`rbnx boot` 会把本次 `-f` 选中的部署清单路径自动传给 Soma，避免 Soma 误读同目录中的另一个清单。Soma 默认连接 `127.0.0.1:50051` 的 Atlas，默认监听 `127.0.0.1:50091`，默认提供方 ID 为 `soma`。

Soma 会根据 Atlas 中发现的里程计和关节状态能力，在 `rbnx-boot/logs/soma-runtime/` 下生成临时 ROS 2 订阅脚本和数据源 JSON，再自动启动读取器。这些文件是运行产物，不是开发者需要维护的配置文件。

:::info[Webots 的跨容器特例]
Webots 示例把 ROS 2 图放在仿真容器中，而 Soma 主进程运行在宿主机，因此该示例在自己的 `system.soma.runtime_reader_command` 中使用 `docker exec`，让自动生成的读取器在仿真容器内运行。原生 ROS 2 机器人不需要这个字段；只有把 Soma 与 ROS 2 图刻意分到不同运行环境时，才需要按对应容器或远程执行环境覆盖它。
:::

启动诊断应同时检查前台阶段、Atlas 能力和 Soma 自己的日志：

```bash
rbnx boot -v -f robonix_manifest.yaml
rbnx caps -v
rbnx inspect
rbnx logs --list-tags
rbnx logs -t soma -l warn
rbnx logs -t soma -f
```

从其他目录读取既有日志时显式指定部署日志目录：

```bash
rbnx logs -d /path/to/deploy/rbnx-boot/logs -t soma --json
```

成功启动后，`rbnx caps -v` 应显示提供方 `soma` 及其 5 条本体接口；还要核对由 Soma 第一阶段管理的原语是否已经进入 `ACTIVE`。只有进程存在、但能力缺失或日志持续出现 `soma/state` 警告，不算本体服务就绪。

### 2. 本体服务启动原语

只要部署声明了 `primitive:` 或 `skill:`，`rbnx boot` 就会确保本体服务（Soma）已配置为读取当前选中的部署清单。本体服务在第一阶段按部署清单启动原语，并等待它们完成注册和生命周期初始化。`rbnx` 通过 Atlas 观察这些提供方的状态，全部就绪后才继续。

### 3. 启动其余系统软件包和服务

不属于内置二进制的 `system:` 项，以及 `service:` 中的包，由 `rbnx` 逐个启动：

1. 记录启动前的 Atlas 提供方集合。
2. 执行 `rbnx start -p <package>`。
3. 等待恰好一个新提供方注册。
4. 校验注册的提供方 ID 与部署项 `name` 一致。
5. 选择共享 `robonix/lifecycle/driver`，确认提供方只注册这一条 Driver，调用 `Driver(CMD_INIT)`，并把部署项 `config:` 编码为 `config_json`。
6. 对非技能提供方再调用 `Driver(CMD_ACTIVATE)`，等待其进入 `ACTIVE`。

生命周期 Driver 是每个受管提供方的标准管理接口。新软件包不需要声明或编写 Driver TOML；框架自动注册唯一的共享 `robonix/lifecycle/driver`。软件包也可以显式选择这条共享 Driver，运行结果相同。生命周期回调按需实现；缺少某个回调时，框架记录警告并执行空操作，原语或服务仍可在初始化和激活后进入 `ACTIVE`。

:::warning[后向兼容：已有命名空间 Driver]
早期软件包可能在自己的 `capabilities/` 目录中保存 Driver TOML，并在清单中声明唯一的 `<provider-namespace>/driver`。这种完整的旧实现目前仍可继续构建和启动，但计划逐步迁移到共享 Driver；维护旧仓库时不要同时追加 `robonix/lifecycle/driver`。

受管启动还支持一个单向迁移场景：旧清单仍精确声明命名空间 Driver，但旧生成服务完全不存在，运行时只注册带兼容标记的共享 Driver，此时启动器会输出迁移警告。旧服务只存在一部分、Driver ID 与提供方命名空间不匹配，或同一提供方注册多条 Driver，都会使启动失败。旧包的完整维护和迁移步骤见[软件包与部署清单规范](../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

某个非内置软件包启动失败时，`rbnx boot` 会将它列入最终 `failures` 段，并保留其他已成功启动的组件。内置系统组件的启动前校验或进程创建失败会终止本次启动并清理已启动的子进程。除 Soma 的第一阶段就绪检查外，当前启动器不会统一等待每个内置组件的业务健康接口，因此“进程已创建”不等于“业务已就绪”。

### 4. 本体服务启动技能

系统软件包和服务启动结束后，`rbnx` 通过启动本体服务时预留的私有管道发送第二阶段触发信号。本体服务启动并初始化 `skill:` 软件包；带生命周期驱动的技能停在 `INACTIVE`，首次实际调用时由执行器激活。该触发信号不是 Atlas RPC。

## 配置如何到达提供方

部署项的 `config:` 不是一组自动导出的环境变量。`rbnx` 或本体服务将该配置序列化为 JSON，并通过共享 Driver 的 `Driver(CMD_INIT, config_json)` 发送；提供方实现了 `on_init` 时在其中解析，未实现时框架记录警告并忽略空操作中的配置。内置系统二进制继续使用 `--config-json` 和类型化命令行参数。

`rbnx` 直接管理的非内置系统软件包和服务会在 `<deploy>/rbnx-boot/instances/<provider-id>.json` 保存一份实例配置，供诊断使用；该文件路径不会传给提供方。本体服务直接管理的原语和技能从部署清单内存值发送配置，当前不依赖该实例文件。

单独调试软件包时，可使用：

```bash
rbnx start -p /path/to/package \
  --endpoint 127.0.0.1:50051 \
  --config /path/to/instance.yaml \
  --set camera.width=640
```

`--set` 覆盖 `--config` 中的同名字段；两者最终仍通过 `Driver(CMD_INIT)` 发送。

Scene 作为非内置系统软件包使用同一条 Driver 配置路径。配置直接写在机器人部署清单中，不需要单独的 Scene 配置文件：

```yaml
system:
  scene:
    manifest: package_manifest.jetson-native.yaml
    config:
      camera_provider_id: front_camera
      web_port: 50107
```

`manifest` 选择 Scene 的目标软件包清单；`config` 在 `CMD_INIT` 中送到 Scene。旧的扁平 `system.scene` 字段和 `RBNX_CONFIG_FILE` 只用于既有部署迁移，并会输出弃用提示。

## 日志与状态

默认日志目录为：

```text
<deploy>/rbnx-boot/logs/
```

软件包日志文件名使用部署项的提供方 ID，例如 `mapping.log`。系统启动器日志写入 `bootstrap.log`。`rbnx logs` 默认读取当前工作目录下的 `./rbnx-boot/logs`，或环境变量 `SCRIBE_LOG_DIR` 指定的目录；它不会根据部署清单自动寻找目录。从其他目录检查日志时必须传 `-d`。

先读取已经写入的历史记录：

```bash
cd /path/to/robot-deploy
rbnx logs --list-tags
rbnx logs -t mapping -l warn
rbnx logs -d /path/to/robot-deploy/rbnx-boot/logs -t mapping --json
```

不带 `-f` 时，`rbnx logs` 合并已有的 Scribe JSON-lines 记录并按时间排序。`-t` 可以重复使用，多个标签按“或”过滤；`-l` 接受 `debug`、`info`、`warn` 和 `error`。跟随模式只显示命令启动后追加到当时已经存在的日志文件中的新记录，不回放历史，也不会自动发现之后才创建的日志文件：

```bash
rbnx logs -t mapping -t nav2 -l info -f
```

每次 `rbnx boot` 会清空本次日志目录中已有的 `*.log`；需要保留故障现场时，应在下一次启动前复制该目录。

普通模式使用可回写的启动动画。需要在启动终端实时查看逐行状态时使用：

```bash
rbnx boot -v -f robonix_manifest.yaml
```

`-v` 会关闭 spinner 和光标回写，并实时输出 INFO、WARN 和 ERROR；它不会启用 DEBUG。普通模式与 `-v` 模式仍写入同一组 Scribe 日志文件。

## Webots 示例的实际进程边界

Webots 示例需要先启动仿真，再构建和启动 Robonix：

```bash
# 终端 1：Webots、ROS 2 通信图、Zenoh 路由器和 RViz
bash examples/webots/sim/start.sh

# 终端 2：Robonix
cd examples/webots
export VLM_BASE_URL=https://api.openai.com/v1
export VLM_API_KEY=...
export VLM_MODEL=...
rbnx build -f robonix_manifest.yaml
rbnx boot -f robonix_manifest.yaml

# 终端 3：交互与诊断
rbnx caps -v
rbnx tools
rbnx chat
```

Tiago 底盘、相机和激光雷达提供方由软件包的 `start.sh` 使用 `docker exec` 放进仿真容器；本地音频提供方运行在宿主机；建图、导航和场景服务等软件包按各自软件包清单决定运行在容器还是宿主机。`rbnx chat` 通过 Atlas 发现交互服务，再由交互服务连接规划器，不是直接连接规划器。

所有参与同一 ROS 2 通信图的进程必须使用一致的 `RMW_IMPLEMENTATION`。Webots 示例默认使用 `rmw_zenoh_cpp`，仿真容器负责启动 `rmw_zenohd`。原语和服务本身仍使用标准 ROS 2 接口；需要跨容器时，它们的启动脚本负责转发 `RMW_IMPLEMENTATION` 和相应的 Zenoh 会话配置。

## 停止

正常部署使用：

```bash
rbnx shutdown
```

前台 `rbnx boot` 也可用 `Ctrl-C` 停止。Webots 仿真结束后再运行：

```bash
bash examples/webots/sim/stop.sh
```

该脚本只停止 Webots 示例自己的 Compose 项目和启动脚本记录的 RViz2 进程，不停止 Atlas、Pilot、能力提供方或其他 Robonix 进程，因此不能代替 `rbnx shutdown`。真实机器人部署应使用 `rbnx shutdown`。受管关闭依次尝试 Driver `CMD_SHUTDOWN`、可选的 `stop:` hook，再以 SIGTERM/SIGKILL 终止受管进程组。没有额外外部资源的软件包可以省略 `stop:`；创建了脱离受管进程组的守护进程、容器或设备会话时，必须由 Driver 或 `stop:` 可靠清理。
