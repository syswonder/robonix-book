# Webots 快速上手

[toc]

本教程在一台 Linux x86_64 主机上启动 Tiago Webots 仿真、Robonix 系统组件、Primitive、Service 和 Skill，然后通过 Liaison 提交一条自然语言任务。第一次执行会编译 Rust workspace、构建容器并下载依赖；后续复用缓存时才是快速启动流程。

<div class="procedure-meta">
  <div><strong>目标平台</strong>Ubuntu 22.04 或 Debian 13，x86_64</div>
  <div><strong>运行方式</strong>Docker Compose + Webots GUI</div>
  <div><strong>源码基线</strong>robonix / dev-next</div>
</div>

## 1. 检查主机

需要本地图形桌面、Git、Make、Python 3.10+、Rust stable、uv、Docker Engine 和 Compose v2。NVIDIA GPU 不是必需项；没有 GPU 时 Webots 使用 CPU 渲染，速度会明显降低。

在 Ubuntu / Debian 上安装基础工具：

```bash
sudo apt update
sudo apt install -y build-essential git curl ca-certificates python3 python3-pip
```

安装 Rust stable：

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
```

安装 uv：

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
```

Docker 使用官方的 [Ubuntu](https://docs.docker.com/engine/install/ubuntu/) 或 [Debian](https://docs.docker.com/engine/install/debian/) 安装步骤。安装后确认当前用户可以直接运行 Docker；如果刚加入 `docker` 组，需要重新登录当前桌面会话。

```bash
git --version
make --version | head -n 1
python3 --version
rustc --version
cargo --version
uv --version
docker version --format '{{.Server.Version}}'
docker compose version
```

<div class="expected-result">
以上命令均以状态码 0 退出；Python 版本不低于 3.10，Docker 命令不需要 <code>sudo</code>，<code>docker compose version</code> 显示 Compose v2。
</div>

## 2. 取得 dev-next 并安装 Robonix

本手册不使用 GitHub 默认分支。克隆时显式选择 `dev-next`，并初始化 contract / IDL 子模块：

```bash
git clone --branch dev-next --recurse-submodules \
  https://github.com/syswonder/robonix.git
cd robonix

git branch --show-current
git submodule status --recursive
make install
```

`make install` 将 `rbnx`、codegen 和 Atlas / Executor / Soma / Vitals / Pilot / Liaison 等系统二进制安装到 `~/.cargo/bin`，并把当前 clone 登记为 Robonix 源码根。确认安装结果：

```bash
export PATH="$HOME/.cargo/bin:$PATH"
rbnx --version
rbnx path root
```

<div class="expected-result">
<code>git branch --show-current</code> 输出 <code>dev-next</code>；<code>rbnx path root</code> 输出刚克隆的 Robonix 仓库绝对路径。
</div>

## 3. 配置 VLM

Pilot 需要 OpenAI-compatible endpoint。以下变量必须出现在执行 `rbnx boot` 的同一个 shell 环境中：

```bash
export VLM_API_KEY='sk-...'
export VLM_BASE_URL='https://api.example.com/v1'
export VLM_MODEL='your-model-name'
```

不要把真实 key 写进 Git。Robonix 当前不会自动加载 deployment 目录中的 `.env`；如果团队用 `.env` 管理本机变量，应将其加入 `.gitignore`，并在启动前显式加载：

```bash
set -a
source .env
set +a
```

检查变量是否存在时不要打印 key：

```bash
test -n "${VLM_API_KEY:-}" && echo 'VLM_API_KEY is set'
printf 'VLM_BASE_URL=%s\nVLM_MODEL=%s\n' "$VLM_BASE_URL" "$VLM_MODEL"
```

## 4. 构建 Webots deployment

从 Robonix 仓库根目录进入示例：

```bash
cd examples/webots
rbnx build
```

构建读取 `examples/webots/robonix_manifest.yaml`，准备本地 Package，并把远程 Mapping、Navigation 和 Explore 仓库放入 `rbnx-boot/cache/`。第一次构建时间主要取决于容器镜像、模型依赖、网络和 CPU；不要把冷启动时间与复用缓存后的启动时间混为一谈。

<div class="expected-result">
<code>rbnx build</code> 以状态码 0 退出。若某个 Package 失败，终端会显示 Package 名称；完整输出和后续启动日志位于当前 deployment 的 <code>rbnx-boot/</code> 下。
</div>

## 5. 启动仿真与 Robonix

使用两个终端。两个终端都从同一个 Robonix clone 工作。

### 终端 1：Webots、ROS 2 与 RViz2

```bash
cd /path/to/robonix
bash examples/webots/sim/start.sh --world office.wbt
```

脚本会启动仿真容器，等待 ROS 2 topics 就绪，并在容器内启动 RViz2。它默认使用 `rmw_zenoh_cpp`；同一 deployment 的 ROS 2 进程必须使用相同的 `RMW_IMPLEMENTATION`。Webots 容器会为该示例启动 `rmw_zenohd`，普通 quickstart 不需要另起 router，也不需要设置第二个 Robonix 专用 RMW 变量。

<div class="expected-result">
终端出现 <code>[sim/start] ros up (... topics)</code> 和 RViz2 日志路径；Webots 与 RViz2 窗口可见。
</div>

### 终端 2：Robonix stack

```bash
export PATH="$HOME/.cargo/bin:$PATH"
export VLM_API_KEY='sk-...'
export VLM_BASE_URL='https://api.example.com/v1'
export VLM_MODEL='your-model-name'

cd /path/to/robonix/examples/webots
rbnx boot
```

当前 `dev-next` Webots manifest 配置 7 个 System、5 个 Primitive、5 个 Service 和 1 个 Skill：

- System：Atlas、Soma、Vitals、Scene、Executor、Pilot、Liaison
- Primitive：Tiago chassis、RGB-D camera、2D lidar、local audio、client audio bridge
- Service：Memory、Speech、Voiceprint、Mapping、Navigation
- Skill：Explore；启动后保持 `INACTIVE`，第一次被调用时由 Executor 激活

<div class="expected-result">
启动摘要中没有 <code>failures</code>，System 显示监听地址，Primitive 与 Service 为 <code>ACTIVE</code>，Explore 为 <code>INACTIVE</code>。终端最后显示组件已启动以及 <code>rbnx-boot/logs</code> 路径。
</div>

Scene 调试页默认位于 <http://127.0.0.1:50107/>。页面同时显示 2D occupancy、语义对象、机器人位姿、3D 点云和相机流；这些数据只有在相应 provider 已经启动并发布后才会出现。

## 6. 提交第一条任务

保持前两个终端运行，在第三个终端检查注册状态：

```bash
export PATH="$HOME/.cargo/bin:$PATH"
cd /path/to/robonix/examples/webots

rbnx caps -v
rbnx tools
rbnx chat
```

`rbnx chat` 先通过 Atlas 发现 Liaison，再由 Liaison 把用户输入交给 Pilot；它不是绕过交互层直连 Pilot。可以依次尝试：

```text
What can you see in front of the robot?
Explore the current room and report what you find.
What tasks are currently running?
```

`Esc` 中断当前交互回合，`Ctrl+C` 退出 TUI。

<div class="expected-result">
终端 TUI 显示用户输入、规划状态、RTDL capability call 与最终回复；Explore 被调用时会从 <code>INACTIVE</code> 转为 <code>ACTIVE</code>，其 provider 日志是 <code>rbnx-boot/logs/explore.log</code>。
</div>

## 7. 选择其他 Webots world

内置 world：

```bash
bash examples/webots/sim/start.sh --world office.wbt
bash examples/webots/sim/start.sh --world apartment.wbt
bash examples/webots/sim/start.sh --world complete_apartment.wbt
bash examples/webots/sim/start.sh --world break_room.wbt
bash examples/webots/sim/start.sh --world kitchen.wbt
```

|  |  |
|---|---|
| `office.wbt`<br>![office](https://raw.githubusercontent.com/syswonder/robonix/dev-next/examples/webots/sim/thumbnails/office.jpg) | `apartment.wbt`<br>![apartment](https://raw.githubusercontent.com/syswonder/robonix/dev-next/examples/webots/sim/thumbnails/apartment.jpg) |
| `complete_apartment.wbt`<br>![complete apartment](https://raw.githubusercontent.com/syswonder/robonix/dev-next/examples/webots/sim/thumbnails/complete_apartment.jpg) | `break_room.wbt`<br>![break room](https://raw.githubusercontent.com/syswonder/robonix/dev-next/examples/webots/sim/thumbnails/break_room.jpg) |
| `kitchen.wbt`<br>![kitchen](https://raw.githubusercontent.com/syswonder/robonix/dev-next/examples/webots/sim/thumbnails/kitchen.jpg) |  |

`office.wbt` 使用镜像内的 asset seed。第一次运行其他 world 时，如日志报告缺少 Webots assets，先下载完整离线包：

```bash
ROBONIX_WEBOTS_DOWNLOAD_ALL_ASSETS=1 \
  bash examples/webots/sim/start.sh --world apartment.wbt
```

## 8. 停止并清理运行进程

先在运行 `rbnx boot` 的终端按 `Ctrl+C`，等待 Robonix 按依赖顺序关闭。然后停止仿真：

```bash
cd /path/to/robonix
bash examples/webots/sim/stop.sh
```

该脚本停止进程与容器，但保留可复用的 Docker volume 和构建缓存。

## 排错

### Webots 或 RViz2 窗口未出现

```bash
printf 'DISPLAY=%s\n' "${DISPLAY:-<unset>}"
docker ps --filter name=robonix_tiago_sim
```

本地图形桌面通常使用 `DISPLAY=:0`。若日志包含 X11 permission 错误，按 `start.sh` 打印的 `xhost` 命令授权本地 Docker 用户；SSH 主机应使用可信 X11 forwarding 或 `ROBONIX_SIM_STREAM=1` 的浏览器模式。

### Package 启动失败

先读启动摘要中点名的 provider 日志，不要只看 bootstrap 尾部：

```bash
ls -1 rbnx-boot/logs
tail -n 120 rbnx-boot/logs/<provider_id>.log
```

### 远程 Package 不是最新版本

`rbnx-boot/cache/` 会复用已克隆的上游仓库。显式更新：

```bash
rbnx update
```

更新会改变实际运行 revision；团队复现问题时同时记录 deployment commit 和每个远程 Package commit。

## 下一步

- [系统部署与启动](../architecture/deployment-and-startup.md)：理解真实启动所有权、生命周期和日志位置。
- [本体接入指南](../integration-guide/vendor-onboarding.md)：把 Webots provider 替换为真实机器人硬件。
- [开发者指南](../developer-guide.md)：从 template-rbnx 开发自己的 Primitive、Service 或 Skill。
- [接口目录](../interface-catalog/index.md)：查询标准 contract 与 ROS IDL。
