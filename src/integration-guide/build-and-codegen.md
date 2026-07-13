# Package 构建与代码生成

[toc]

讲怎么用 `rbnx` 把一个 package 从源码构建到可运行——登记 Robonix 源码根、生成 stubs、`build.sh` 模板、package 自带 contract / IDL。

## 第一步：登记 Robonix 源码根（`rbnx setup`）

把当前 clone 登记为 robonix 源码根目录，写入 `~/.robonix/config.yaml`：

```bash
cd /path/to/robonix     # 仓库根目录
rbnx setup
```

`rbnx setup` 在当前目录及其上级查 `Cargo.toml`、`capabilities`、`capabilities/lib` 三个 marker 都齐了才登记，缺一个直接报错。也支持显式传参 `rbnx setup /abs/path/to/robonix`。

**`make install` 会自动跑一次 `rbnx setup`**——clone 完只要 `cd robonix/rust && make install` 即可。

### 旧配置文件迁移

如果 `~/.robonix/config.yaml` 没有 `robonix_source_path` 字段（老版本留下的），后续 `rbnx build / start / validate / install / codegen` 会立刻停下并提示：

```
[rbnx] config is missing robonix_source_path (legacy config from before the `rbnx setup` migration).
Fix:  cd /path/to/robonix && rbnx setup
```

跑一次即可。

## 第二步：生成 package 的 stubs（`rbnx codegen`）

```bash
rbnx codegen -p /path/to/my_package                 # 生成 proto + Python stubs
rbnx codegen -p /path/to/my_package --mcp           # 同时生成 robonix_mcp_types/（MCP 工具包需要）
rbnx codegen -p /path/to/my_package --ros2          # 同时生成 ros2_idl/（ROS 2 话题/服务需要）
rbnx codegen -p /path/to/my_package --mcp --clean   # 先清理旧产物再生成
rbnx codegen -p /path/to/my_package --mcp --out-dir bridge   # 产物落到 <pkg>/bridge/ 子目录
```

三种传输的产物都落在 `<pkg>/rbnx-build/codegen/` 下，约定一致：

- `proto_gen/`：gRPC 的 Python stubs（始终生成）
- `robonix_mcp_types/`：MCP 的 dataclass（加 `--mcp`）
- `ros2_idl/`：ROS 2 的 canonical 消息包（加 `--ros2`）——这是源码，需在 ROS 2 环境里 `colcon build`，再 `source install/setup.bash`；之后节点的消息类型即来自 Robonix 的定义

做的事（全在 `rbnx` 内部）：

1. 把 robonix 源码仓库的 `capabilities/` 和 package 自己的 `capabilities/` 合并，扫出当前 package 能引用的 contract 全集（同 `contract_id` 包内覆盖全局）
2. 调 `robonix-codegen` 生成 proto 描述
3. 若加 `--mcp`，再生成 MCP dataclass 到 `<pkg>/rbnx-build/codegen/robonix_mcp_types/`
4. 调 `grpc_tools.protoc` 把 proto 翻成 Python stubs，落到 `<pkg>/rbnx-build/codegen/proto_gen/`
5. 若加 `--ros2`，生成 ROS 2 canonical 消息包到 `<pkg>/rbnx-build/codegen/ros2_idl/`
6. 不需要写 `PYTHONPATH`——`robonix_api` 第一次 import 时会自动把 `proto_gen` / `robonix_mcp_types` 加进 `sys.path`（`ros2_idl` 由 `colcon` + `source` 接管）

## 第三步：package 的 build.sh 模板

绝大多数 package 的 `build.sh` 仅需一行：

```bash
#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
# 用了 @<provider>.mcp(...) 的包加上 --mcp
FLAGS+=(--mcp)

rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[build] done."
```

需要额外构建（cargo 编 Rust 库、`pip install -e`、docker compose build 等）的，加在 `rbnx codegen` 之后即可。

### 参考：仓库内现有 build.sh

仓库自带的 webots 例子 + system 服务都遵循上面这套模板：

| Package | 路径 | 额外步骤 |
|---|---|---|
| `tiago_chassis` / `tiago_camera` / `tiago_lidar` / `audio_driver` | `examples/webots/primitives/` | 无 |
| `nav2` | manifest 远端包 `nav2_wrapper_rbnx` | ROS 2/Nav2 wrapper 构建 |
| `scene` | `system/scene` | docker compose build（perception 容器内跑 YOLO-World + MobileSAM）|
| `memory`（memsearch） | `services/memsearch` | 无 |
| `speech` | `services/speech` | 无（首次跑会下 FunASR / TTS 权重）|

## 进阶：package 自带 contract / IDL

如果包要暴露一个**官方仓库没有**的 contract（典型：skill 包），把能力约定写在包自己的 `capabilities/` 下：

```
my_package/
├── capabilities/                # 包内 contract
│   ├── my_custom.v1.toml
│   └── lib/                     # 包内 ROS IDL（msg/ 与 srv/ 在此之下）
│       └── my_custom/
│           └── srv/
├── package_manifest.yaml
└── scripts/build.sh
```

`package_manifest.yaml` 里 `capabilities:` 引用时**带 `path:`** 指向包内 TOML：

```yaml
capabilities:
  - name: robonix/skill/my_stack/my_custom
    path: capabilities/my_custom.v1.toml
```

`rbnx codegen` 扫描两种 `capabilities/` 根（robonix 源码 + 包内）合并，包内 contract 跟官方 contract 一样进 stub。无需在 `build.sh` 里手动 staging。

IDL 路径解析规则：

- **官方 TOML**（在 robonix 源码 `capabilities/` 里）：`[contract] idl = "lidar/srv/Foo.srv"` → 去 `capabilities/lib/lidar/srv/Foo.srv` 找
- **包内 TOML**（在你 package 的 `capabilities/` 里）：`[contract] idl = "my_custom/srv/Foo.srv"` → 去**包的 `capabilities/lib/my_custom/srv/Foo.srv`** 找

能力提供者的 namespace 是主分类，不是授权边界。普通 contract id 应使用该 namespace 前缀；`DeclareCapability` 对不一致会接受并记录 warning。共享 contract 可在 TOML 中设置 `cross_namespace = true`，由不同 namespace 的能力提供者实现而不产生提示。

## 常用命令速查

```bash
# 一次性初始化
cd /path/to/robonix && rbnx setup       # make install 已自动跑

# 改了 contract/IDL 后重生所有 stubs
rbnx codegen -p /path/to/my_package --mcp --clean

# 查询当前配置
rbnx config --show

# 取某个路径（用于自己的脚本）
$(rbnx path root)
$(rbnx path capabilities)
$(rbnx path interfaces-lib)
$(rbnx path robonix-api)
```

合法 path key：`root` / `rust` / `capabilities` / `interfaces-lib` / `runtime-proto` / `robonix-api`。

## rbnx update 同步远端 provider

部署 manifest 里用 `url:` 引入的 provider（如 `mapping`、`nav2`、`explore`）在第一次 `rbnx boot` / `rbnx build` 时被 clone 到 `<manifest 目录>/rbnx-boot/cache/<name>/`，之后一直**复用这份 checkout、不会自己更新**——所以上游仓库更新后，本地可能在跑一份过时的代码而不自知。

`rbnx boot` 和 `rbnx build` 每次都会检查所有已 clone 的远端 provider，若本地落后于远端分支，会打印一条**非阻塞**提示（落后几个 commit、远端最新 commit 的日期与一行简介），并给出更新命令。要同步到最新用 `rbnx update`（本质就是对各 checkout 做一次 fast-forward `git pull`）：

```bash
# 在部署目录（有 robonix_manifest.yaml）：更新该部署的所有远端 provider
rbnx update                     # 先打印每个包的落后概览，再问 y/N 才拉

# 只更新某一个包：-p 指定目录，或在该包 checkout 里直接运行
rbnx update -p examples/webots/rbnx-boot/cache/nav2
cd examples/webots/rbnx-boot/cache/mapping && rbnx update

# 指定别的部署 manifest
rbnx update -f path/to/robonix_manifest.yaml
```

更新前一定会打印 overview（当前 commit、远端 commit、落后数、最新 commit 简介）并等 `y` 确认；本地有分叉（diverged）的 checkout 只提示、跳过，不会强制 reset。这样上游仓库更新后，每个使用方都能自己决定更不更新。
