# rbnx 命令行工具

<!-- toc -->

`rbnx` 是 Robonix 的包管理与部署命令行工具，支持安装、列出与编译包，配置路径，按 `recipe` 注册与启停原语/服务/技能进程，以及提交、查询与取消任务。若要开发原语、服务或技能，请参阅 [第三章 开发文档](../chapter3-developer-guide/index.md)。

## 安装与配置

- 安装：在 `robonix/rust` 执行 `make install`（`rbnx`、`rbnx-daemon` 安装至 `~/.cargo/bin`）；`make install-core` 安装 `robonix-core`。
- robonix-sdk 路径：使用依赖 ROS2 的包或通过 rbnx 启动 provider 能力前需配置，否则 start 脚本无法 source SDK。执行 `rbnx config --set-sdk-path /path/to/robonix-sdk`（路径下须存在 `install/setup.bash`），配置写入 `~/.robonix/config.yaml`，rbnx 与 provider 内 start 脚本会据此设置 `ROBONIX_SDK_PATH`。查看当前值：`rbnx config --show`。详见 [robonix-sdk 使用说明 - 用户配置 SDK 路径](../chapter3-developer-guide/robonix-sdk.md)。
- 包目录：默认 `~/.robonix/packages`。使用仓库 provider 时在 `rust/` 下执行 `make setup-dev` 软链本地 provider。

## 命令总览

### 配置（Config）

| 命令 | 说明 |
|------|------|
| `rbnx config --show` / `-s` | 显示当前配置（包路径、SDK 路径等） |
| `rbnx config --set-storage-path <path>` | 设置包存储路径 |
| `rbnx config --set-sdk-path <path>` | 设置 robonix-sdk 路径（需存在 install/setup.bash） |

### 包管理（Package）

| 命令 | 说明 |
|------|------|
| `rbnx package install --path <dir>` | 从本地目录安装包 |
| `rbnx package install --github <url> [--branch <branch>]` | 从 GitHub 克隆并安装（可选分支） |
| `rbnx package list` | 列出已安装的包 |
| `rbnx package info <name>` | 查看指定包的详细信息（含 manifest 中的原语/服务/技能） |
| `rbnx package search cap <name>` | 按能力名搜索包 |
| `rbnx package search skill <name>` | 按技能名搜索包 |
| `rbnx package build [all\|包名]` | 编译包；默认 `all` 表示所有已安装包 |

### 部署（Deploy）

建议顺序：先 `register`，再 `build`（可选），再 `start`；停止时 `stop`，最后可 `unregister`。

| 命令 | 说明 |
|------|------|
| `rbnx deploy register <recipe.yaml>` | 按 recipe 注册到 robonix-server（不启动进程） |
| `rbnx deploy build [all\|包名]` | 编译 recipe 中的包 |
| `rbnx deploy start [all\|pattern]` | 启动已注册项对应的进程（执行各 start_script）；`pattern` 可为 `cap::vision.*`、`*.pick` 等 |
| `rbnx deploy stop [all\|pattern]` | 停止已启动的进程 |
| `rbnx deploy restart [all\|pattern]` | 先 stop 再 start |
| `rbnx deploy status` | 查看当前注册与运行状态（各项是否在运行） |
| `rbnx deploy unregister <target>` | 从 core 注销；`target` 可为 recipe 文件路径、包名、或 `包名.能力名`；需先 `deploy stop` |
| `rbnx deploy clean` | 清理 `~/.robonix/packages/logs` 下的日志文件 |

### 守护进程（Daemon）

注册/注销会通过 `rbnx-daemon` 调用 robonix-server 的 ROS2 服务；若未启动 daemon，执行 register 时会自动拉起。

| 命令 | 说明 |
|------|------|
| `rbnx daemon start` | 启动 daemon |
| `rbnx daemon stop` | 停止 daemon |
| `rbnx daemon status` | 查看 daemon 状态 |
| `rbnx daemon restart` | 重启 daemon |

### 任务（Task）

| 命令 | 说明 |
|------|------|
| `rbnx task create "自然语言描述"` | 提交自然语言任务 |
| `rbnx task get <task_id>` | 查看任务状态与结果 |
| `rbnx task list` | 列出任务 |
| `rbnx task cancel <task_id>` | 取消任务 |

## 使用示例

```bash
# 1. 配置并查看
rbnx config --set-sdk-path $(pwd)/robonix-sdk
rbnx config --show

# 2. 可选：make setup-dev 后 rbnx package list 列出本地 provider
rbnx package list

# 3. 注册并启动（需已启动 robonix-server）
rbnx deploy register demo_recipe.yaml
rbnx deploy build
rbnx deploy start
rbnx deploy status

# 4. 提交任务
rbnx task create "Pick up the red box on the table"
rbnx task get task_0

# 5. 停止并注销
rbnx deploy stop
rbnx deploy unregister demo_recipe.yaml
```

开发与扩展说明请参阅 [第三章 开发文档](../chapter3-developer-guide/index.md)。
