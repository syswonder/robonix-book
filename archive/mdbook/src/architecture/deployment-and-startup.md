# 系统部署与启动流程

[toc]

本页解释 `rbnx build` 和 `rbnx boot` 如何把一份机器人部署清单变成正在运行的 Robonix 系统。它面向需要定位启动失败、能力未注册或配置未生效问题的开发者。

## 两层清单各自负责什么

| 文件 | 读取方 | 负责内容 |
|---|---|---|
| `<deploy>/robonix_manifest.yaml` | `rbnx build`、`rbnx boot`、Soma | 选择系统组件、Primitive、Service 和 Skill 实例；指定包来源、分支、目标 manifest 和实例配置 |
| `<package>/package_manifest.yaml` | `rbnx build`、`rbnx start` | 定义一个包的元数据、构建命令、启动命令、停止命令和 contract 列表 |

部署清单中的 `name` 是运行时 provider ID。一个包可以在同一部署中出现多次，但每个实例必须使用不同的 `name`。对于 `url:` 包，代码缓存目录按 Git 仓库名创建，而不是按实例名创建；多个实例可以复用同一份 checkout。

```yaml
service:
  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config:
      params_file: config/rtabmap_params.yaml
      sensor_providers:
        lidar3d: roof_lidar
        odom: base_chassis
```

这里的 `manifest:` 选择该包目录中的 package manifest 文件。文件不存在时，构建和启动都会直接报错，不会回退到默认 manifest。

## 构建阶段

在部署目录运行：

```bash
rbnx build -f robonix_manifest.yaml
```

`rbnx build` 会完成以下工作：

1. 解析部署清单并展开其中的环境变量。
2. 将 `url:` 包克隆到 `<deploy>/rbnx-boot/cache/<repository-name>/`；已有 checkout 会被复用。
3. 对每个包读取部署项选定的 package manifest。
4. 在包根目录执行该 manifest 的 `build:` 命令。
5. 构建成功后写入 `<package>/rbnx-build/.rbnx-built`。

`rbnx boot` 发现缺少 checkout 或 build sentinel 时会警告并尝试补做克隆和构建。这是首次启动的容错路径，不应代替显式的 `rbnx build`。

## 启动阶段

当前 `dev-next` 的启动顺序由 `rbnx` 与 Soma 共同完成。

### 1. 启动内置系统组件

`rbnx boot` 按下面的固定顺序启动部署清单中声明的内置二进制：

1. Atlas
2. Executor
3. Soma
4. Vitals
5. Pilot
6. Liaison

每个系统块的字段会转换为对应二进制的命令行参数。监听端口已被占用或必填参数为空时，`rbnx boot` 会在启动该组件前失败，避免连接到遗留进程。

### 2. Soma 启动 Primitive

只要部署声明了 `primitive:` 或 `skill:`，`rbnx boot` 就会确保 Soma 已配置为读取当前选中的部署清单。Soma 在第一阶段按部署清单启动 Primitive，并等待它们完成注册和生命周期初始化。`rbnx` 通过 Atlas 观察这些 provider 的状态，全部就绪后才继续。

### 3. 启动其余 System package 和 Service

不属于内置二进制的 `system:` 项，以及 `service:` 中的包，由 `rbnx` 逐个启动：

1. 记录启动前的 Atlas provider 集合。
2. 执行 `rbnx start -p <package>`。
3. 等待恰好一个新 provider 注册。
4. 校验注册的 provider ID 与部署项 `name` 一致。
5. 如果 provider 声明了 `*/driver` gRPC contract，调用 `Driver(CMD_INIT)`，并把部署项 `config:` 编码为 `config_json`。
6. 对非 Skill provider 再调用 `Driver(CMD_ACTIVATE)`，等待其进入 `ACTIVE`。

没有 driver contract 的 package 必须自行完成 readiness 与状态上报；注册完成后，启动器将其显示为 `ACTIVE (no driver)`。

某个非内置 package 启动失败时，`rbnx boot` 会将它列入最终 `failures` 段，并保留其他已成功启动的组件。Atlas、Executor 等内置系统组件启动失败则会终止本次启动并清理已启动的子进程。

### 4. Soma 启动 Skill

System package 和 Service 启动结束后，`rbnx` 通知 Soma 进入 Skill 阶段。Soma 启动并初始化 `skill:` 包；带 driver 的 Skill 停在 `INACTIVE`，首次实际调用时由 Executor 激活。

## 配置如何到达 provider

部署项的 `config:` 不是一组自动导出的环境变量。`rbnx boot` 将它序列化为 JSON，并通过 provider 的 `Driver(CMD_INIT, config_json)` 发送。provider 应在 `on_init` 中解析配置。

`rbnx boot` 会在 `<deploy>/rbnx-boot/instances/<provider-id>.json` 保存一份实例配置，供诊断使用；该文件路径不会传给 provider。

单独调试 package 时，可使用：

```bash
rbnx start -p /path/to/package \
  --endpoint 127.0.0.1:50051 \
  --config /path/to/instance.yaml \
  --set camera.width=640
```

`--set` 覆盖 `--config` 中的同名字段；两者最终仍通过 `Driver(CMD_INIT)` 发送。

## 日志与状态

默认日志目录为：

```text
<deploy>/rbnx-boot/logs/
```

package 日志文件名使用部署项的 provider ID，例如 `mapping.log`。系统启动器日志写入 `bootstrap.log`。推荐的检查顺序是：

```bash
rbnx caps -v
rbnx tools
rg -n "ERROR|FAIL|timeout|Deferred" rbnx-boot/logs
```

普通模式使用可回写的启动动画。需要完整逐行日志时使用：

```bash
rbnx boot -v -f robonix_manifest.yaml
```

`-v` 会关闭 spinner 和光标回写，并将 Scribe 组件日志追加输出到终端。

## Webots 示例的实际进程边界

Webots 示例需要先启动仿真，再构建和启动 Robonix：

```bash
# 终端 1：Webots、ROS 2 graph、Zenoh router 和 RViz
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

Tiago chassis、camera 和 lidar provider 由 package 的 `start.sh` 使用 `docker exec` 放进仿真容器；本地音频 provider 运行在宿主机；Mapping、Navigation、Scene 等 package 按各自 package manifest 决定运行在容器还是宿主机。`rbnx chat` 通过 Atlas 发现 Liaison，再由 Liaison 连接 Pilot，不是直接连接 Pilot。

所有参与同一 ROS 2 graph 的进程必须使用一致的 `RMW_IMPLEMENTATION`。Webots 示例默认使用 `rmw_zenoh_cpp`，仿真容器负责启动 `rmw_zenohd`。Primitive、Service 本身仍使用标准 ROS 2 API；需要跨容器时，它们的启动脚本负责转发 `RMW_IMPLEMENTATION` 和相应的 Zenoh session 配置。

## 停止

正常部署使用：

```bash
rbnx shutdown
```

前台 `rbnx boot` 也可用 `Ctrl-C` 停止。Webots 仿真结束后再运行：

```bash
bash examples/webots/sim/stop.sh
```

该脚本清理仿真容器、RViz 和示例 package 可能遗留的进程。真实机器人部署应优先使用 `rbnx shutdown`，并由各 package 的 `stop:` 或 shutdown hook 释放硬件资源。
