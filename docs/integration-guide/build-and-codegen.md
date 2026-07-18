# 软件包构建与代码生成


本页说明如何把一个 Robonix 软件包（Package）从清单校验、能力约定代码生成一直构建到可单独启动。

## 前置：登记 Robonix 源码树

代码生成需要读取 Robonix 主仓库中的标准能力约定、接口定义语言（Interface Definition Language，IDL）和 Atlas 协议缓冲区（Protocol Buffers，Protobuf）定义。先在主仓库根目录运行：

```bash
cd /path/to/robonix
rbnx setup
```

也可以显式传入路径：

```bash
rbnx setup /absolute/path/to/robonix
```

`rbnx setup` 从给定目录向上查找同时包含以下内容的仓库根：

```text
Cargo.toml
capabilities/
capabilities/lib/
```

成功后，它将绝对路径写入 `${ROBONIX_HOME:-~/.robonix}/config.yaml` 的 `robonix_source_path`。

在 Robonix 仓库根目录执行 `make install` 时，Makefile 会在安装二进制后自动运行一次 `rbnx setup`。如果自动登记失败，安装过程会给出 warning；此时仍需手动运行上述命令。

检查当前值：

```bash
rbnx config --show
rbnx path root
rbnx path capabilities
rbnx path interfaces-lib
```

旧配置缺少 `robonix_source_path`，或登记的目录已被移动时，`build`、`start`、`validate`、`install` 和 `codegen` 会拒绝继续，并提示重新执行 `rbnx setup`。

## 1. 校验软件包清单

```bash
rbnx validate /path/to/my_package
```

也可以在软件包目录内运行：

```bash
rbnx validate
```

`validate` 的软件包路径是位置参数，不使用 `-p`。成功输出为：

```text
✓ Manifest validation passed
```

## 2. 生成能力约定绑定

最常用的命令是：

```bash
rbnx codegen -p /path/to/my_package
rbnx codegen -p /path/to/my_package --mcp
rbnx codegen -p /path/to/my_package --mcp --ros2
rbnx codegen -p /path/to/my_package --mcp --ros2 --clean
```

省略 `-p` 时，`rbnx` 从当前目录向上查找软件包清单。

| 选项 | 产物 |
|---|---|
| 无额外选项 | Protobuf 描述和 Python Protobuf/gRPC 存根 |
| `--mcp` | 额外生成 MCP 输入输出类型 |
| `--ros2` | 额外生成 ROS 2 接口叠加层源码 |
| `--clean` | 先删除本次代码生成管理的旧产物 |

默认目录：

```text
<package>/
└── rbnx-build/
    ├── proto-staging/
    └── codegen/
        ├── proto_gen/
        ├── robonix_mcp_types/     # 使用 --mcp 时
        └── ros2_idl/              # 使用 --ros2 时
```

`--out-dir <path>` 可以改变 `proto_gen/` 和其他代码生成产物的根目录。默认启动路径只会自动注入 `rbnx-build/codegen/`；使用自定义目录时，软件包的 `start:` 必须自行设置导入路径。

### 代码生成的输入合并规则

`rbnx codegen` 合并两组能力约定与 IDL：

1. Robonix 主仓库的 `capabilities/` 和 `capabilities/lib/`。
2. 当前软件包可选的 `capabilities/` 和 `capabilities/lib/`。

同一能力约定 ID 的软件包局部描述文件会覆盖全局描述文件。这样可以先在软件包中实验接口，再将稳定接口提交到 Robonix 主仓库。

随后 `rbnx`：

1. 将生成的 proto 暂存到 `rbnx-build/proto-staging/`。
2. 使用 `python3 -m grpc_tools.protoc` 生成 `proto_gen/`。
3. 根据选项生成 MCP 类型和 ROS 2 叠加层源码。

运行代码生成的 Python 环境必须安装 `grpcio-tools`。缺失时命令会失败，不会继续生成不完整的绑定代码。

### 构建 ROS 2 叠加工作区

`--ros2` 生成的是 colcon 工作区源码，不是已安装的 ROS 2 软件包。Robonix 固定使用这套生成结果作为自身 ROS 2 消息和服务类型的唯一来源，不直接使用发行版中可能存在的同名接口包。软件包的 `build.sh` 需要先加载目标机器安装的 ROS 2 环境，再构建生成的叠加层：

```bash
source /opt/ros/humble/setup.bash
cd /path/to/package/rbnx-build/codegen/ros2_idl
colcon build
```

运行时必须先加载系统 ROS 2 环境，再加载 Robonix 生成的叠加层：

```bash
source /opt/ros/humble/setup.bash
source /path/to/package/rbnx-build/codegen/ros2_idl/install/setup.bash
```

目标平台可以使用不同 ROS 2 发行版；第一行应指向该平台实际安装的 ROS 2 `setup.bash`。第二行不能省略，并且必须位于系统 ROS 2 之后，使 Robonix 代码生成的软件包在叠加层中优先解析。这样不同平台使用的 `rclpy`、`rclcpp` 和 ROS 中间件实现（ROS Middleware Implementation，RMW）可以来自各自发行版，但 Robonix 能力约定引用的 ROS 2 线上数据类型始终来自同一份标准 IDL。

## 3. 编写构建脚本

纯 Python 提供方（Provider）的最小构建脚本：

```bash
#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

FLAGS=(--mcp)
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[build] done"
```

`rbnx build` 会把软件包根目录写入 `RBNX_PACKAGE_ROOT`。使用 `rbnx build --clean` 时还会设置 `RBNX_BUILD_CLEAN=1`；脚本负责决定需要清理哪些软件包自有产物。

需要 Cargo、colcon、Python 依赖、Docker 镜像或模型权重的软件包，在代码生成后增加自己的构建步骤。构建阶段不应依赖一个正在运行的机器人系统。

## 4. 软件包局部能力约定

软件包需要官方仓库尚未定义的接口时，可使用：

```text
my_package/
├── capabilities/
│   ├── inspect.v1.toml
│   └── lib/
│       └── inspection/
│           └── srv/
│               └── Inspect.srv
├── package_manifest.yaml
└── scripts/
    ├── build.sh
    └── start.sh
```

软件包清单 `package_manifest.yaml`：

```yaml
capabilities:
  - name: robonix/skill/inspection/inspect
    path: capabilities/inspect.v1.toml
```

清单只列业务能力约定。框架会自动选择并注册共享的 `robonix/lifecycle/driver`；不要在 `capabilities/` 中创建 Driver TOML。生命周期回调按需实现，缺失回调由框架 warning 后执行空操作。业务接口仍由软件包内的 `inspect.v1.toml` 定义。

:::warning[后向兼容：已有命名空间 Driver]
已有软件包若使用 `<provider-namespace>/driver` 和本地 Driver TOML，应暂时保留原 `name` 与 `path`，代码生成仍会读取它。该方式计划迁移到共享 Driver；不要在保留旧 Driver 的同时追加共享 Driver。兼容验证和迁移步骤见[软件包与部署清单规范](packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

软件包内能力约定描述中的：

```toml
idl = "inspection/srv/Inspect.srv"
```

解析为：

```text
<package>/capabilities/lib/inspection/srv/Inspect.srv
```

## 5. 构建与单软件包启动

```bash
rbnx build -p /path/to/my_package
rbnx start -p /path/to/my_package --endpoint 127.0.0.1:50051
```

`rbnx start` 会：

- 加载 `<package>/rbnx-build/ws/install/setup.bash`，如果该文件存在；
- 将软件包根目录、`rbnx-build/codegen/proto_gen/` 和 `robonix_mcp_types/` 加入 `PYTHONPATH`；
- 设置 `ROBONIX_ATLAS` 和 Scribe 日志目录；
- 执行软件包清单的 `start:`。

使用实例配置：

```bash
rbnx start -p /path/to/my_package \
  --config /path/to/config.yaml \
  --set camera.width=640
```

配置最终通过该提供方唯一的 Driver 的 `CMD_INIT(config_json)` 发送，不会以一组任意环境变量注入提供方。本节的新包应在 `rbnx caps -v` 中只显示 `robonix/lifecycle/driver`，并同时列出 `robonix/skill/inspection/inspect`；清单显式声明与运行时注册不一致会使初始化失败。已有包则核对其唯一的命名空间 Driver。

## 6. 构建整个机器人部署

```bash
cd /path/to/robot-deploy
rbnx build -f robonix_manifest.yaml
```

部署项的 `manifest:` 会同时选择软件包的构建与运行变体。下面的机器人描述原语在本机 ROS 2 环境中运行，因此选择仓库提供的原生目标：

```yaml
primitive:
  - name: robot_description
    url: https://github.com/syswonder/primitive-robot-description-rbnx
    branch: main
    manifest: package_manifest.native.yaml
    config: {}
```

`rbnx` 把包含 `robonix_manifest.yaml` 的目录记录为本次启动目录。软件包可通过 `RBNX_INVOCATION_CWD` 解析部署仓库中的相对文件；Navigation 与 Mapping 的 `params_file` 都使用这一基准。参数文件归机器人部署仓库所有，上游软件包只提供可复制的示例或模板。

远端软件包缓存在：

```text
<deploy>/rbnx-boot/cache/<git-repository-name>/
```

缓存名来自 URL 的仓库名，不是部署实例 `name`。多个实例引用同一 URL 时复用同一源码检出。

<span id="7-更新远端-package"></span>
## 7. 更新远端软件包

`rbnx build` 和 `rbnx boot` 默认检查已缓存的远端软件包是否落后，并给出非阻塞提示；`--no-update-check` 可在离线或固定快照时关闭远端获取检查。

更新一个 deployment：

```bash
cd /path/to/robot-deploy
rbnx update -f robonix_manifest.yaml
```

更新单个源码检出：

```bash
rbnx update -p rbnx-boot/cache/service-map-rbnx
```

`rbnx update` 先获取远端并显示差异，再询问 `y/N`。它只执行快进更新；源码检出已分叉时会跳过或报错，不会强制重置，也不会覆盖本地修改。更新后重新运行 `rbnx build -f robonix_manifest.yaml`。

## 常用路径

```bash
rbnx path root
rbnx path rust
rbnx path capabilities
rbnx path interfaces-lib
rbnx path runtime-proto
rbnx path robonix-api
```

当前 `rust` 是仓库根目录的兼容别名。新脚本应优先使用表达实际意图的 key，例如 `root`、`capabilities` 或 `robonix-api`。
