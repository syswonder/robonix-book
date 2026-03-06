# 快速开始 — 在 Docker 与 Webots 中尝试 Robonix

<!-- toc -->

本节在 Docker 与 Webots 环境下，带你完成环境准备、构建安装、能力注册与任务提交的完整流程。

## 环境要求

- ubuntu 22.04 + ROS2 Humble
- docker
- 图形界面环境

## 1. Docker 环境

使用仓库提供的 ROS2 开发镜像与 `run.sh` 进入容器。

### 进入容器

```bash
cd robonix/docker
./run.sh          # 拉取镜像并进入容器（若无镜像则拉取；可用 Ctrl+P Ctrl+Q 脱离不关容器）
./run.sh -b       # 若修改过 Dockerfile，先本地构建镜像再进入
```

- 默认镜像：`docker.io/enkerewpo/robonix_ros:latest`；`-b` 为本地构建 `robonix_ros`。
- 容器内已配置 ROS2 Humble、X11；挂载见 `run.sh`。

### 软链本地 provider

在容器内执行，使 `~/.robonix/packages` 指向仓库 `provider` 目录：

```bash
cd /root/workspace/robonix/rust
make setup-dev
```

执行后 `rbnx package list` 将列出 provider 下包（如 tiago_demo_package、demo_service_provider、navigation_skills_provider）。需从网络安装其他包时使用 `rbnx package install --github <url>`。

## 2. 构建与安装

在容器内 `robonix/rust` 目录：

```bash
make build-sdk
make build
make install
```

配置 CLI 的 SDK 路径：

```bash
eval $(make source-sdk)   # 当前 shell 生效
rbnx config --set-sdk-path $(pwd)/robonix-sdk
rbnx config --show
```

## 3. Webots 与能力启动顺序

本节步骤以仓库内 Tiago 演示包为准，请同时参考 `robonix/rust/provider/tiago_demo_package/README.md`。正确顺序：先启动 Webots 仿真与机器人能力（Nav2 等），再启动 robonix-core，最后执行 `rbnx deploy register` 与 `rbnx deploy start`。

### 启动 Webots 仿真

按 tiago_demo_package 的 README：在包目录下执行 `./run.sh`（会启动 Webots 并加载世界与机器人）。世界文件示例：`robonix/rust/provider/tiago_demo_package/eaios_webots/worlds/office.wbt`。Docker 内需已配置 X11 或虚拟显示（`run.sh` 含相关映射）。

### 启动机器人能力（Nav2 等）

按 tiago_demo_package 的 README，在新终端进入 `robonix/rust/provider/tiago_demo_package/nav2_webots_tiago` 执行 `./run.sh` 启动 Nav2。可选：在包目录执行 `./start_rviz.sh` 启动 RViz2。确保原语对应 Topic 有数据，例如 `prm::base.pose.cov`（如 `/amcl_pose`）、`prm::base.navigate`（如 `/goal_pose`）、`prm::camera.rgb` / `prm::camera.depth`。

### 启动 robonix-core

在 rust 目录下新开终端：

```bash
cd /root/workspace/robonix/rust
eval $(make source-sdk)
ROBONIX_WEB_ASSETS_DIR="$(pwd)/robonix-core/web" \
ROBONIX_WEB_PORT=8000 \
RUST_LOG=robonix_core=info \
robonix-core
```

或执行 `./core.sh`（后台、带 Web UI）。

### 注册并启动能力

```bash
cd /root/workspace/robonix/rust
eval $(make source-sdk)
rbnx deploy register demo_recipe.yaml
rbnx deploy build
rbnx deploy start
rbnx deploy status
```

确认状态；失败时查看 `~/.robonix/packages/logs` 或 Console View Log。

### 提交任务

demo_recipe 仅注册导航类技能（`skl::wandering`、`skl::move_to_object`），示例任务须与之匹配，例如巡视或“移动到某物体”。语义地图等就绪后：

```bash
rbnx task create "在房间里巡视一圈"
rbnx task get task_0
```

或使用可与语义地图物体关联的指令（由 task_plan 解析为 move_to_object 等）。执行器按 RTDL 依次调用已注册技能。

启动顺序：先 Webots 仿真与机器人能力，再启动 `robonix-core`，然后执行 `rbnx deploy register` 与 `rbnx deploy start`，最后用 `rbnx task create` 提交任务。

## 4. LLM 与 API Key

demo_service_provider（语义地图、任务规划）依赖大模型 API。包根目录为仓库内 `robonix/rust/provider/demo_service/`；若已执行 `make setup-dev`，则 `~/.robonix/packages` 指向 `provider`，包根为 `~/.robonix/packages/demo_service/`（目录名为 demo_service，与 manifest 中的包名 demo_service_provider 不同）。在该包根下创建 `.env`：

```bash
DASHSCOPE_API_KEY=sk-xxxxxxxxxxxxxxxx
```

可选兼容变量：`QWEN3_VL_API_KEY`。API Key 在 [阿里云 DashScope](https://dashscope.aliyun.com/) 控制台创建；勿提交到版本库。任务规划若用同一或其它模型，以该包代码为准。

## 5. 常用命令速查

| 操作 | 命令 |
|------|------|
| 查看配置 | `rbnx config --show` |
| 列出包 | `rbnx package list` |
| 注册 recipe | `rbnx deploy register demo_recipe.yaml` |
| 编译包 | `rbnx deploy build` |
| 启动能力进程 | `rbnx deploy start` |
| 查看状态 | `rbnx deploy status` |
| 停止能力进程 | `rbnx deploy stop` |
| 提交任务 | `rbnx task create "任务描述"` |
| 查看任务 | `rbnx task get <task_id>` |
| 取消任务 | `rbnx task cancel <task_id>` |

详见 [rbnx 命令行工具](../chapter2-user-guide/rbnx-cli.md)。

## 6. 故障排查

- robonix-core 未运行：`ros2 service list | grep rbnx` 无输出时，先启动 robonix-core 并 source ROS2 与 SDK。
- 能力未就绪：`rbnx deploy status` 中某项未运行时，查看 Robonix Console 或 `~/.robonix/packages/logs`；服务类检查 .env 与 API Key。
- 仿真未就绪：先确认 Webots 仿真、AMCL/Nav2/相机已启动，再执行 deploy start 与 task create。
