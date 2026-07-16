# Webots 快速上手


本教程在一台 Linux x86_64 主机上启动 Tiago Webots 仿真、Robonix 系统组件、原语（Primitive）、服务（Service）和技能（Skill），然后通过 Liaison 提交一条自然语言任务。第一次执行会编译 Rust 工作区、构建容器并下载依赖；后续复用缓存时才是快速启动流程。

<div class="procedure-meta">
  <div><strong>目标平台</strong>Ubuntu 22.04 或 Debian 13，x86_64</div>
  <div><strong>运行方式</strong>Docker Compose + Webots GUI</div>
  <div><strong>源码仓库</strong>syswonder/robonix</div>
</div>

## 1. 检查主机

需要本地图形桌面、Git、Make、Python 3.10+、Rust stable、uv、Docker Engine 和 Compose v2。只有在验证机器人主机的本地音频时，才需要可由 ALSA 枚举的录音或播放设备。本页使用模拟语音后端验证文本任务链路，因此不要求 NVIDIA GPU；没有 GPU 时 Webots 使用 CPU 渲染，速度会明显降低。

在 Ubuntu / Debian 上安装基础工具：

```bash
sudo apt update
sudo apt install -y \
  build-essential git curl ca-certificates \
  python3 python3-pip python3-grpc-tools alsa-utils
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
python3 -c 'import grpc_tools.protoc; print("grpc_tools: ok")'
arecord -l
aplay -l
```

**预期结果：** 版本命令均以状态码 0 退出；Python 版本不低于 3.10，`grpc_tools: ok` 可见，Docker 命令不需要 `sudo`，`docker compose version` 显示 Compose v2。若要使用机器人主机的本地音频，`arecord -l` 或 `aplay -l` 还应列出目标设备。

## 2. 安装 Robonix

克隆开发分支并初始化能力约定与接口定义子模块：

```bash
git clone --branch dev-next --recurse-submodules \
  https://github.com/syswonder/robonix.git
cd robonix

git branch --show-current
git submodule status --recursive
make install
```

`make install` 将 `rbnx`、代码生成器和 Atlas、Executor、Soma、Vitals、Pilot、Liaison 等系统可执行文件安装到 `~/.cargo/bin`，并把当前克隆目录登记为 Robonix 源码根目录。确认安装结果：

```bash
export PATH="$HOME/.cargo/bin:$PATH"
rbnx --version
rbnx path root
```

**预期结果：** `git branch --show-current` 输出 `dev-next`；`rbnx path root` 输出刚克隆的 Robonix 仓库绝对路径。

## 3. 配置视觉语言模型

Pilot 需要兼容 OpenAI 接口的视觉语言模型（VLM）访问地址。以下变量必须出现在执行 `rbnx boot` 的同一个命令行环境中：

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

## 4. 构建 Webots 部署

从 Robonix 仓库根目录进入示例：

```bash
cd examples/webots
export SPEECH_BACKEND=mock
rbnx build
```

构建读取 `examples/webots/robonix_manifest.yaml`，准备本地软件包，并把清单中通过 `url:` 引用的音频、建图、导航和自主探索仓库放入 `rbnx-boot/cache/`。`SPEECH_BACKEND=mock` 让本页只验证系统、规划与文本交互，不下载本地语音识别模型。第一次构建时间主要取决于容器镜像、网络和 CPU；不要把冷启动时间与复用缓存后的启动时间混为一谈。

**预期结果：** `rbnx build` 以状态码 0 退出。构建脚本的输出直接显示在当前终端，各软件包的构建产物位于各自的 `rbnx-build/`；随后执行 `rbnx boot` 时，运行日志才会写入当前部署目录的 `rbnx-boot/logs/`。

## 5. 启动仿真与 Robonix

使用两个终端。两个终端都从同一个 Robonix clone 工作。

### 终端 1：Webots、ROS 2 与 RViz2

```bash
cd /path/to/robonix
bash examples/webots/sim/start.sh --world office.wbt
```

脚本会启动仿真容器，等待 ROS 2 topics 就绪，并在容器内启动 RViz2。它默认使用 `rmw_zenoh_cpp`；同一 deployment 的 ROS 2 进程必须使用相同的 `RMW_IMPLEMENTATION`。Webots 容器会为该示例启动 `rmw_zenohd`，普通 quickstart 不需要另起 router，也不需要设置第二个 Robonix 专用 RMW 变量。

**预期结果：** 终端出现 `[sim/start] ros up (... topics)` 和 RViz2 日志路径；Webots 与 RViz2 窗口可见。

### 终端 2：Robonix stack

```bash
export PATH="$HOME/.cargo/bin:$PATH"
export VLM_API_KEY='sk-...'
export VLM_BASE_URL='https://api.example.com/v1'
export VLM_MODEL='your-model-name'
export SPEECH_BACKEND=mock
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

cd /path/to/robonix/examples/webots
rbnx boot
```

Webots 部署清单配置以下系统组件和软件包：

- System：Atlas、Soma、Vitals、Scene、Executor、Pilot、Liaison
- Primitive：Tiago chassis、RGB-D camera、2D lidar，以及通过独立仓库取得的 ALSA audio 和 client audio bridge
- Service：Memory、Speech、Voiceprint、Mapping、Navigation
- Skill：Explore；启动后保持 `INACTIVE`，第一次被调用时由 Executor 激活

**预期结果：** 启动摘要中没有 `failures`，系统组件显示监听地址，原语与服务为 `ACTIVE`，Explore 为 `INACTIVE`。终端最后显示组件已启动以及 `rbnx-boot/logs` 路径。

Scene 调试页默认位于 [http://127.0.0.1:50107/](http://127.0.0.1:50107/)。页面同时显示 2D occupancy、语义对象、机器人位姿、3D 点云和相机流；这些数据只有在相应 provider 已经启动并发布后才会出现。

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

**预期结果：** 终端界面显示用户输入、规划状态、RTDL 能力调用与最终回复；Explore 被调用时会从 `INACTIVE` 转为 `ACTIVE`，其 provider 日志是 `rbnx-boot/logs/explore.log`。

## 7. 选择其他 Webots world

回到 Robonix 仓库根目录选择内置 world：

```bash
cd /path/to/robonix
bash examples/webots/sim/start.sh --world office.wbt
bash examples/webots/sim/start.sh --world apartment.wbt
bash examples/webots/sim/start.sh --world complete_apartment.wbt
bash examples/webots/sim/start.sh --world break_room.wbt
bash examples/webots/sim/start.sh --world kitchen.wbt
```

|  |  |
|---|---|
| `office.wbt`<br />![office](https://raw.githubusercontent.com/syswonder/robonix/dev/examples/webots/sim/thumbnails/office.jpg) | `apartment.wbt`<br />![apartment](https://raw.githubusercontent.com/syswonder/robonix/dev/examples/webots/sim/thumbnails/apartment.jpg) |
| `complete_apartment.wbt`<br />![complete apartment](https://raw.githubusercontent.com/syswonder/robonix/dev/examples/webots/sim/thumbnails/complete_apartment.jpg) | `break_room.wbt`<br />![break room](https://raw.githubusercontent.com/syswonder/robonix/dev/examples/webots/sim/thumbnails/break_room.jpg) |
| `kitchen.wbt`<br />![kitchen](https://raw.githubusercontent.com/syswonder/robonix/dev/examples/webots/sim/thumbnails/kitchen.jpg) |  |

`office.wbt` 使用镜像内的 asset seed。第一次运行其他内置 world 前，先下载一次 Cyberbotics 官方离线 asset 包；后续启动会复用持久化缓存：

```bash
ROBONIX_WEBOTS_DOWNLOAD_ALL_ASSETS=1 \
  bash examples/webots/sim/start.sh --world apartment.wbt
```

## 8. 停止并清理运行进程

先在运行 `rbnx boot` 的终端按 `Ctrl+C`，等待 Robonix 关闭。也可以从另一个终端在部署目录执行 `rbnx shutdown`。然后运行仿真的配套停止脚本，它会停止 Webots Compose 项目、RViz2，以及该示例启动的残留 Robonix 进程和软件包容器：

```bash
cd /path/to/robonix/examples/webots
# 仅在没有通过 Ctrl+C 停止 rbnx boot 时执行：
rbnx shutdown
bash sim/stop.sh
```

`sim/stop.sh` 会执行 `docker compose down`，但保留可复用的镜像、Webots asset volume 和软件包构建缓存。该脚本还会按进程名强制清理该示例可能遗留的 Robonix、Python service 和 RViz2 进程；在共享主机上运行前应确认没有其他用户共用这些进程名。

## 排错

### Webots 或 RViz2 窗口未出现

```bash
printf 'DISPLAY=%s\n' "${DISPLAY:-<unset>}"
docker ps --filter name=robonix_tiago_sim
```

本地图形桌面通常使用 `DISPLAY=:0`。若日志包含 X11 permission 错误，按 `start.sh` 打印的 `xhost` 命令授权本地 Docker 用户；SSH 主机应使用可信 X11 forwarding 或 `ROBONIX_SIM_STREAM=1` 的浏览器模式。

### 软件包启动失败

先读启动摘要中点名的 provider 日志，不要只看 bootstrap 尾部：

```bash
ls -1 rbnx-boot/logs
tail -n 120 rbnx-boot/logs/<provider_id>.log
```

### 远程软件包不是最新版本

`rbnx-boot/cache/` 会复用已克隆的上游仓库。显式更新：

```bash
rbnx update
```

更新会改变实际运行的源码修订号；团队复现问题时，应同时记录部署仓库和每个远程软件包的提交号。

## 下一步

- [系统部署与启动](../architecture/deployment-and-startup.md)：理解真实启动所有权、生命周期和日志位置。
- [本体接入指南](../integration-guide/vendor-onboarding.md)：把 Webots 能力提供者替换为真实机器人硬件。
- [开发者指南](../developer-guide.md)：从 template-rbnx 开发自己的原语、服务或技能。
- [接口目录](../interface-catalog/index.md)：查询标准契约与 ROS 接口定义。
