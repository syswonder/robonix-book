# Package 构建与代码生成

[toc]

本页说明如何把一个 Robonix package 从 manifest 校验、contract codegen 构建到可单独启动。命令与目录结构对应当前 `dev-next` 的 `rbnx`。

## 前置：登记 Robonix 源码树

package codegen 需要读取 Robonix 主仓库中的标准 contract、IDL 和 Atlas proto。先在主仓库根目录运行：

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

## 1. 校验 package manifest

```bash
rbnx validate /path/to/my_package
```

也可以在 package 目录内运行：

```bash
rbnx validate
```

`validate` 的 package 路径是位置参数，不使用 `-p`。成功输出为：

```text
✓ Manifest validation passed
```

## 2. 生成 contract bindings

最常用的命令是：

```bash
rbnx codegen -p /path/to/my_package
rbnx codegen -p /path/to/my_package --mcp
rbnx codegen -p /path/to/my_package --mcp --ros2
rbnx codegen -p /path/to/my_package --mcp --ros2 --clean
```

省略 `-p` 时，`rbnx` 从当前目录向上查找 package manifest。

| 选项 | 产物 |
|---|---|
| 无额外选项 | protobuf 描述和 Python protobuf/gRPC stubs |
| `--mcp` | 额外生成 MCP 输入输出类型 |
| `--ros2` | 额外生成 ROS 2 interface overlay 源码 |
| `--clean` | 先删除本次 codegen 管理的旧产物 |

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

`--out-dir <path>` 可以改变 `proto_gen/` 和其它 codegen 产物的根目录。默认启动路径只会自动注入 `rbnx-build/codegen/`；使用自定义目录时，package 的 `start:` 必须自行设置 import path。

### Codegen 的输入合并规则

`rbnx codegen` 合并两组 contract 与 IDL：

1. Robonix 主仓库的 `capabilities/` 和 `capabilities/lib/`。
2. 当前 package 可选的 `capabilities/` 和 `capabilities/lib/`。

同 contract ID 的 package-local descriptor 覆盖全局 descriptor。这样可以先在 package 中实验接口，再将稳定接口提交到 Robonix 主仓库。

随后 `rbnx`：

1. 将生成的 proto 暂存到 `rbnx-build/proto-staging/`。
2. 使用 `python3 -m grpc_tools.protoc` 生成 `proto_gen/`。
3. 根据选项生成 MCP 类型和 ROS 2 overlay 源码。

运行 codegen 的 Python 环境必须安装 `grpcio-tools`。缺失时命令会失败，不会继续生成不完整 stubs。

### ROS 2 overlay 还需要构建

`--ros2` 生成的是 colcon workspace 源码，不是已安装的 ROS 2 package。package 的 `build.sh` 需要在目标 ROS 2 环境中执行：

```bash
source /opt/ros/humble/setup.bash
cd /path/to/package/rbnx-build/codegen/ros2_idl
colcon build
```

运行时 source：

```bash
source /path/to/package/rbnx-build/codegen/ros2_idl/install/setup.bash
```

如果 package 使用另一 ROS distro，应改为该目标实际安装的 setup 文件。

## 3. 编写 build.sh

纯 Python provider 的最小构建脚本：

```bash
#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

FLAGS=(--mcp)
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[build] done"
```

`rbnx build` 会把 package 根目录写入 `RBNX_PACKAGE_ROOT`。使用 `rbnx build --clean` 时还会设置 `RBNX_BUILD_CLEAN=1`；脚本负责决定需要清理哪些 package 自有产物。

需要 cargo、colcon、Python 依赖、Docker image 或模型权重的 package，在 codegen 后增加自己的构建步骤。构建阶段不应依赖一个正在运行的 robot stack。

## 4. Package-local contract

package 需要官方仓库尚未定义的接口时，可使用：

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

`package_manifest.yaml`：

```yaml
capabilities:
  - name: robonix/skill/inspection/inspect
    path: capabilities/inspect.v1.toml
```

package-local descriptor 中的：

```toml
idl = "inspection/srv/Inspect.srv"
```

解析为：

```text
<package>/capabilities/lib/inspection/srv/Inspect.srv
```

## 5. 构建与单 package 启动

```bash
rbnx build -p /path/to/my_package
rbnx start -p /path/to/my_package --endpoint 127.0.0.1:50051
```

`rbnx start` 会：

- source `<package>/rbnx-build/ws/install/setup.bash`，如果该文件存在；
- 将 package 根目录、`rbnx-build/codegen/proto_gen/` 和 `robonix_mcp_types/` 加入 `PYTHONPATH`；
- 设置 `ROBONIX_ATLAS` 和 Scribe 日志目录；
- 执行 package manifest 的 `start:`。

使用实例配置：

```bash
rbnx start -p /path/to/my_package \
  --config /path/to/config.yaml \
  --set camera.width=640
```

配置最终通过 `Driver(CMD_INIT, config_json)` 发送，不会以一组任意环境变量注入 provider。

## 6. 构建整个 deployment

```bash
cd /path/to/robot-deploy
rbnx build -f robonix_manifest.yaml
```

部署项的 `manifest:` 会同时选择 package 的构建与运行变体：

```yaml
service:
  - name: nav2
    url: https://github.com/syswonder/service-navigation-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config:
      params_file: config/nav2_params.yaml
```

远端 package 缓存在：

```text
<deploy>/rbnx-boot/cache/<git-repository-name>/
```

缓存名来自 URL 的仓库名，不是部署实例 `name`。多个实例引用同一 URL 时复用同一 checkout。

## 7. 更新远端 package

`rbnx build` 和 `rbnx boot` 默认检查已缓存的远端 package 是否落后，并给出非阻塞提示；`--no-update-check` 可在离线或固定快照时关闭 fetch 检查。

更新一个 deployment：

```bash
cd /path/to/robot-deploy
rbnx update -f robonix_manifest.yaml
```

更新单个 checkout：

```bash
rbnx update -p rbnx-boot/cache/service-map-rbnx
```

`rbnx update` 先 fetch 和显示差异，再询问 `y/N`。它只执行 fast-forward；checkout 已分叉时会跳过或报错，不会强制 reset，也不会覆盖本地修改。更新后重新运行 `rbnx build -f robonix_manifest.yaml`。

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
