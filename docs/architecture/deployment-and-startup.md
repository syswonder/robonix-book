# 系统部署与启动流程


本页解释 `rbnx build` 和 `rbnx boot` 如何把一份机器人部署清单变成正在运行的 Robonix 系统。它面向需要定位启动失败、能力未注册或配置未生效问题的开发者。

## 两层清单各自负责什么

| 文件 | 读取方 | 负责内容 |
|---|---|---|
| `<deploy>/robonix_manifest.yaml` | `rbnx build`、`rbnx boot`、Soma | 选择系统组件、原语、服务和技能实例；指定软件包来源、分支、目标清单和实例配置 |
| `<package>/package_manifest.yaml` | `rbnx build`、`rbnx start` | 定义一个软件包的元数据、构建命令、启动命令、停止命令和能力约定列表 |

原语、服务和技能部署项的 `name` 是运行时提供方 ID；非内置系统软件包使用 `system:` 下的键名。一个软件包可以在同一部署中出现多次，但每个实例必须使用不同的 ID。对于 `url:` 软件包，代码缓存目录按 Git 仓库名创建，而不是按实例名创建；多个实例可以复用同一份代码检出。

```yaml
system:
  scene:
    manifest: package_manifest.jetson-native.yaml
```

这里的 `manifest:` 选择该软件包目录中的软件包清单文件。文件不存在时，构建和启动都会直接报错，不会回退到默认清单。Scene 当前不会读取 `system.scene` 中除 `manifest` 以外的运行参数；相机提供方、`web_port` 等配置必须通过下文的 `RBNX_CONFIG_FILE` 启动包装脚本传入。

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

### 2. 本体服务启动原语

只要部署声明了 `primitive:` 或 `skill:`，`rbnx boot` 就会确保本体服务（Soma）已配置为读取当前选中的部署清单。本体服务在第一阶段按部署清单启动原语，并等待它们完成注册和生命周期初始化。`rbnx` 通过 Atlas 观察这些提供方的状态，全部就绪后才继续。

### 3. 启动其余系统软件包和服务

不属于内置二进制的 `system:` 项，以及 `service:` 中的包，由 `rbnx` 逐个启动：

1. 记录启动前的 Atlas 提供方集合。
2. 执行 `rbnx start -p <package>`。
3. 等待恰好一个新提供方注册。
4. 校验注册的提供方 ID 与部署项 `name` 一致。
5. 如果提供方声明了 `*/driver` gRPC 能力约定，调用 `Driver(CMD_INIT)`，并把部署项 `config:` 编码为 `config_json`。
6. 对非技能提供方再调用 `Driver(CMD_ACTIVATE)`，等待其进入 `ACTIVE`。

没有生命周期驱动能力约定的软件包必须自行完成就绪判断与状态上报。启动器看到提供方注册后会直接显示 `ACTIVE (no driver)`；这个标签只证明注册成功，不证明其服务接口已经可调用。此类软件包必须提供自己的健康检查，并在验收时单独调用。

某个非内置软件包启动失败时，`rbnx boot` 会将它列入最终 `failures` 段，并保留其他已成功启动的组件。内置系统组件的启动前校验或进程创建失败会终止本次启动并清理已启动的子进程。除 Soma 的第一阶段就绪检查外，当前启动器不会统一等待每个内置组件的业务健康接口，因此“进程已创建”不等于“业务已就绪”。

### 4. 本体服务启动技能

系统软件包和服务启动结束后，`rbnx` 通过启动本体服务时预留的私有管道发送第二阶段触发信号。本体服务启动并初始化 `skill:` 软件包；带生命周期驱动的技能停在 `INACTIVE`，首次实际调用时由执行器激活。该触发信号不是 Atlas RPC。

## 配置如何到达提供方

部署项的 `config:` 不是一组自动导出的环境变量。对声明了 `*/driver` 的软件包提供方，`rbnx` 或本体服务将该配置序列化为 JSON，并通过 `Driver(CMD_INIT, config_json)` 发送；提供方应在 `on_init` 中解析。没有生命周期驱动能力约定的软件包不会从这条路径收到 `config:`，需要由其启动实现明确定义配置入口。内置系统组件则使用上述 `--config-json` 和类型化命令行参数，不经过 `Driver(CMD_INIT)`。

`rbnx` 直接管理的非内置系统软件包和服务会在 `<deploy>/rbnx-boot/instances/<provider-id>.json` 保存一份实例配置，供诊断使用；该文件路径不会传给提供方。本体服务直接管理的原语和技能从部署清单内存值发送配置，当前不依赖该实例文件。

单独调试软件包时，可使用：

```bash
rbnx start -p /path/to/package \
  --endpoint 127.0.0.1:50051 \
  --config /path/to/instance.yaml \
  --set camera.width=640
```

`--set` 覆盖 `--config` 中的同名字段；两者最终仍通过 `Driver(CMD_INIT)` 发送。

Scene 当前没有 Driver 配置通道，是上述规则的例外。把运行参数保存在部署仓库的 `config/scene.yaml`：

```yaml
camera_provider_id: front_camera
web_port: 50107
```

再由部署仓库的启动包装脚本导出绝对路径：

```bash
DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
export RBNX_CONFIG_FILE="$DEPLOY_DIR/config/scene.yaml"
exec rbnx boot -f "$DEPLOY_DIR/robonix_manifest.yaml" "$@"
```

不要把 `camera_provider_id`、`web_port` 等字段写进 `system.scene` 并假定它们会生效。

## 日志与状态

默认日志目录为：

```text
<deploy>/rbnx-boot/logs/
```

软件包日志文件名使用部署项的提供方 ID，例如 `mapping.log`。系统启动器日志写入 `bootstrap.log`。推荐的检查顺序是：

```bash
rbnx caps -v
rbnx tools
rg -n "ERROR|FAIL|timeout|Deferred" rbnx-boot/logs
```

普通模式使用可回写的启动动画。需要完整逐行日志时使用：

```bash
rbnx boot -v -f robonix_manifest.yaml
```

`-v` 会关闭启动动画和光标回写，并将日志组件（Scribe）的输出追加到终端。

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

该脚本只停止 Webots 示例自己的 Compose 项目和启动脚本记录的 RViz2 进程，不停止 Atlas、Pilot、能力提供方或其他 Robonix 进程，因此不能代替 `rbnx shutdown`。真实机器人部署应使用 `rbnx shutdown`，并由各软件包的 `stop:` 或关闭处理函数释放硬件资源。
