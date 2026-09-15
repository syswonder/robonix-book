# Webots 快速上手


本教程在一台 Linux x86_64 主机上启动 Tiago Webots 仿真，以及 Robonix 的系统组件、原语（Primitive）、服务（Service）和技能（Skill），然后通过 Liaison 提交一条自然语言任务。第一次执行会编译 Rust 工作区、构建容器并下载依赖；后续复用缓存时才是快速启动流程。

<div class="procedure-meta">
  <div><strong>目标平台</strong>Ubuntu 22.04 或 Debian 13，x86_64</div>
  <div><strong>运行方式</strong>Docker Compose + Webots GUI</div>
  <div><strong>源码仓库</strong>syswonder/robonix</div>
</div>

## 1. 检查主机

默认图形界面路径需要可用的 X Server 和图形栈。命令行工具需要 Git、Make、Python 3.10+、Rust stable、uv、Docker Engine 和 Compose v2。

:::note[本教程使用的图形环境]
当前完整 Webots 测试使用 NVIDIA GPU、NVIDIA 驱动和 `nvidia-container-toolkit`；下面的主流程以这条已验证路径为准。仓库的基础 Compose 也映射了 `/dev/dri`，镜像内还包含 Xvfb。但 Intel/AMD 图形和 CPU 软件渲染尚未纳入完整端到端验收，只作为兼容与排错路径。
:::

在 Ubuntu / Debian 上安装基础工具：

```bash
sudo apt update
sudo apt install -y \
  build-essential git curl ca-certificates \
  python3 python3-pip python3-grpc-tools alsa-utils ffmpeg
```

`alsa-utils` 和 `ffmpeg` 供语音链路使用。第 6 节会用麦克风和扬声器完成一次语音任务，`arecord` 和 `aplay` 也来自 `alsa-utils`。

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

scene 镜像的构建需要 BuildKit（`RUN --mount=type=cache`），因此还需 buildx 插件；用发行版自带的 `docker.io` 时它不会一并装上，Ubuntu 上执行 `sudo apt install docker-buildx`。

```bash
git --version
make --version | head -n 1
python3 --version
rustc --version
cargo --version
uv --version
docker version --format '{{.Server.Version}}'
docker compose version
docker buildx version
python3 -c 'import grpc_tools.protoc; print("grpc_tools: ok")'
```

**预期结果：** 版本命令均以状态码 0 退出；Python 版本不低于 3.10，`grpc_tools: ok` 可见，Docker 命令不需要 `sudo`，`docker compose version` 显示 Compose v2。

## 2. 安装 Robonix

克隆源码，检出本手册校验过的提交，并初始化能力约定与接口定义子模块：

```bash
git clone --recurse-submodules https://github.com/syswonder/robonix.git
cd robonix
git checkout --detach cec06ee874eace27dd622e6ce4685c971f04a9e4
git submodule update --init --recursive

git rev-parse HEAD
git submodule status --recursive
make install
```

`make install` 把 `rbnx`、代码生成器和 Atlas、Executor、Soma、Vitals、Pilot、Liaison 等系统可执行文件安装到 `~/.cargo/bin`。它同时把当前克隆目录登记为 Robonix 源码根目录。确认安装结果：

```bash
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
rbnx --version
rbnx path root
```

**预期结果：** `git rev-parse HEAD` 输出 `cec06ee874eace27dd622e6ce4685c971f04a9e4`；`rbnx path root` 输出刚克隆的 Robonix 仓库绝对路径。

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
rbnx build
```

构建读取 `examples/webots/robonix_manifest.yaml`，准备本地软件包，并把清单中通过 `url:` 引用的音频、建图、导航和自主探索仓库放入 `rbnx-boot/cache/`。

第一次构建时间主要取决于容器镜像、模型下载、网络和 CPU。不要把冷启动时间与复用缓存后的启动时间混为一谈。

Webots 部署包含一条完整的语音链路：音频原语、语音识别与合成、声纹。第 6 节会用到它。默认识别后端是本地 FunASR，第一次构建会安装语音依赖并下载 `paraformer-zh-streaming` 模型。要改用腾讯云，或想了解为什么默认不启用 Whisper，见[语音后端配置](../appendix/speech-backends.md)。

**预期结果：** `rbnx build` 以状态码 0 退出。构建脚本的输出直接显示在当前终端，各软件包的构建产物位于各自的 `rbnx-build/`；随后执行 `rbnx boot` 时，运行日志才会写入当前部署目录的 `rbnx-boot/logs/`。

## 5. 启动仿真与 Robonix

使用两个终端。两个终端都从同一个 Robonix clone 工作。

### 终端 1：Webots、ROS 2 与 RViz2

```bash
cd /path/to/robonix
bash examples/webots/sim/start.sh --world office.wbt
```

脚本会启动仿真容器，等待 ROS 2 话题就绪，并在容器内启动 RViz2。它默认使用 ROS 中间件实现（ROS Middleware Implementation，RMW）`rmw_zenoh_cpp`；同一部署中的 ROS 2 进程必须使用相同的 `RMW_IMPLEMENTATION`。Webots 容器会为该示例启动 `rmw_zenohd`，本快速上手流程不需要另起路由器，也不需要设置第二个 Robonix 专用 RMW 变量。

**预期结果：** 终端出现 `[sim/start] ros up (... topics)` 和 RViz2 日志路径；Webots 与 RViz2 窗口可见。

#### RViz2 窗口里在看什么

RViz2 是验收和排障工具，机器人无头运行时不需要它。示例自带的配置是 `examples/webots/sim/rviz2_default.rviz`，Fixed Frame 设为 `map`，默认打开这些显示项：

| Displays 面板中的名字 | 话题 | 用途 |
|---|---|---|
| `SlamMap` | `/map` | 建图服务输出的二维占据栅格 |
| `GlobalCostmap` | `/global_costmap/costmap` | 导航的全局代价地图 |
| `LocalCostmap` | `/local_costmap/costmap` | 导航的局部代价地图 |
| `LaserScan` | `/scanner_normalized` | 雷达点，用来判断是否与墙面重合 |
| `GlobalPlan` | `/plan` | 全局路径 |
| `LocalPlan` | `/local_plan` | 局部路径 |
| `Odometry` | `/odom` | 底盘里程计 |
| `GoalPose` | `/rviz_goal_pose` | 从 RViz 工具栏下发的目标点 |
| `TF` | — | 坐标树 |
| `Grid` | — | 参考网格，不来自机器人 |

左侧 **Displays** 面板控制每一项的开关。刚启动时 `/map` 还是空的，建图服务收到足够数据后才会出现栅格。

三件事最值得先看。**TF 是否连通**：展开 TF 显示，确认 `map → odom → base_link` 这条链存在，断在哪一级就说明哪一级的发布者没起来。**雷达是否贴合**：机器人静止时 LaserScan 的点应当落在墙上，明显偏移说明定位不对。**代价地图是否合理**：机器人周围不应出现大片致命代价，否则规划会失败。

![RViz2 在 Webots 示例跑起来之后的样子。左侧 Displays 面板列出本页表格里的各项，中间是 SlamMap 的占据栅格叠加两层代价地图和雷达点，左下 Navigation 2 面板显示导航状态。](/img/ui/rviz2-live.webp)

这些显示项只是读取话题，RViz2 自己不驱动机器人。完整的本体验收清单见[本体接入指南 §7.4](../integration-guide/vendor-onboarding.md#74-使用-rviz2-验证地图定位与导航)，那里还说明了从 RViz 直接下发导航目标时 `SetGoal` 与 `GoalTool` 的区别。

### 终端 2：Robonix 系统

```bash
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
export VLM_API_KEY='sk-...'
export VLM_BASE_URL='https://api.example.com/v1'
export VLM_MODEL='your-model-name'
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

cd /path/to/robonix/examples/webots
rbnx boot
```

:::tip[没有音频设备的机器]
`audio_driver` 默认自动探测系统的麦克风和扬声器。设备选择在第 6 节的 `Ctrl+A` 页面里做，这里不用配。

机器上确实没有任何声卡时（无声卡服务器或 CI），`audio_driver` 会启动失败。在执行 `rbnx boot` 的同一终端退回空设备：

```bash
export AUDIO_MIC_DEVICE='null'
export AUDIO_SPEAKER_DEVICE='null'
```

字符串 `null` 选择 ALSA 内置的空 PCM，不需要创建 `.asoundrc`。此时第 6 节的语音步骤无法验证，其余步骤不受影响。
:::

Webots 部署清单配置以下系统组件和软件包：

- 系统：Atlas、Soma、Vitals、Scene、Executor、Pilot、Liaison
- 原语：Tiago 底盘、RGB-D 相机、二维激光雷达、`tiago_health`（模拟本体遥测，供 Soma/Vitals 消费），以及通过独立仓库取得的 ALSA 音频和客户端音频桥
- 服务：记忆（memsearch 与 `memgraph` 结构化记忆并行）、语音、声纹、建图、导航
- 技能：探索；启动后保持 `INACTIVE`，第一次被调用时由 Executor 激活

**预期结果：** 启动摘要中没有 `failures`，系统组件显示监听地址，原语与服务为 `ACTIVE`，Explore 为 `INACTIVE`。终端最后显示组件已启动以及 `rbnx-boot/logs` 路径。

Scene 调试页默认位于 [http://127.0.0.1:50107/](http://127.0.0.1:50107/)。页面同时显示二维占据栅格、语义对象、机器人位姿、三维点云和相机流；这些数据只有在相应提供方已经启动并发布后才会出现。

## 6. 提交第一条任务

保持前两个终端运行，在第三个终端检查注册状态：

```bash
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
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

界面顶部会打印本次可用的按键：

```text
Enter = send · F2 = voice (auto end on silence) · Ctrl+A = audio settings · Esc = abort turn · Ctrl+C = quit.
```

![rbnx chat 的界面。左侧是对话区，右上是当前任务与 RTDL 森林，底部输入框的标题栏重复了可用按键。](/img/ui/rbnx-chat-main.webp)

`Esc` 中断当前交互回合，`Ctrl+C` 退出文本用户界面（Text User Interface，TUI）。

**预期结果：** 终端界面显示用户输入、规划状态、机器人任务描述语言（Robot Task Description Language，RTDL）能力调用与最终回复；Explore 被调用时会从 `INACTIVE` 转为 `ACTIVE`，其提供方日志是 `rbnx-boot/logs/explore.log`。

### 选择麦克风与扬声器

在 `rbnx chat` 里按 **`Ctrl+A`** 打开音频设置页。它一屏显示四项：麦克风提供方、麦克风设备、扬声器提供方、扬声器设备。

| 按键 | 作用 |
|---|---|
| `Tab` / `Shift+Tab` | 在四个区块之间切换 |
| `↑` `↓` 或 `k` `j` | 在当前区块内移动 |
| `Enter` 或 `Space` | 选中当前项 |
| `r` | 重新从 Atlas 拉取提供方与设备列表 |
| `Esc` 或 `Ctrl+A` | 关闭并保存 |

本机运行仿真时，两个提供方都选 `audio_driver`，它使用这台机器的 ALSA 设备。设备项留空表示用系统默认；默认设备不对时在这里显式选一个。关闭后聊天界面会打印一行 `audio settings updated: mic=… · speaker=…` 确认。

Atlas 里没有麦克风或扬声器提供方时，页面会提示 `no mic provider in atlas — voice input disabled`。这不影响文本任务。

### 用语音提交一条任务

按 **`F2`** 开始说话，**停止说话后录音自动结束**，不需要再按一次。对着麦克风说一句和上面同样的话，例如“你前面有什么”。

Liaison 依次调用麦克风采集、语音识别、声纹、Pilot 规划和语音合成，最后由扬声器播报回复。这条链路用到第 4 节下载的 FunASR 模型和刚才选定的设备。

**预期结果：** 界面依次显示识别文本、规划状态和回复，扬声器播出回复语音。

识别文本为空或明显不对时，先确认录音设备本身可用：

```bash
arecord -d 3 -f S16_LE -r 16000 -c 1 /tmp/mic-test.wav
aplay /tmp/mic-test.wav
```

这段录放音直接使用 ALSA，不经过 Robonix。听不到声音说明问题在设备或权限，不在语音服务。

## 7. 选择其他 Webots 场景

示例内置五个场景，每次启动选其中一个。`office.wbt` 是默认场景，第 5 节已经用过。

其余四个在第一次运行前，需要先下载一次 Cyberbotics 官方离线资源包。下载只做一次，之后复用持久化缓存：

```bash
cd /path/to/robonix
ROBONIX_WEBOTS_DOWNLOAD_ALL_ASSETS=1 \
  bash examples/webots/sim/start.sh --world apartment.wbt
```

资源就绪后，换场景只需改 `--world`。仿真正在运行时直接执行也可以，Compose 会按新的场景重建容器：

```bash
bash examples/webots/sim/start.sh --world complete_apartment.wbt
bash examples/webots/sim/start.sh --world break_room.wbt
bash examples/webots/sim/start.sh --world kitchen.wbt
```

|  |  |
|---|---|
| `office.wbt`<br />![Webots 办公室场景预览](/img/webots/office.jpg) | `apartment.wbt`<br />![Webots 公寓场景预览](/img/webots/apartment.jpg) |
| `complete_apartment.wbt`<br />![Webots 完整公寓场景预览](/img/webots/complete_apartment.jpg) | `break_room.wbt`<br />![Webots 休息室场景预览](/img/webots/break_room.jpg) |
| `kitchen.wbt`<br />![Webots 厨房场景预览](/img/webots/kitchen.jpg) |  |

默认的 `office.wbt` 第一次启动时，会通过 `https://ghfast.top/` 下载一次带校验和的 [`webots-office-seed-v3`](https://github.com/syswonder/robonix-assets/releases/tag/webots-office-seed-v3)，随后从持久化 Webots 缓存卷复用。要绕过镜像站直连 GitHub，可把 `ROBONIX_WEBOTS_SEED_MIRROR` 设为空；`ROBONIX_WEBOTS_SEED_URL` 可以覆盖完整下载地址。

## 8. 停止并清理运行进程

首先停止 Robonix：在运行 `rbnx boot` 的终端按 `Ctrl+C`，并等待关闭完成；也可以从另一个终端在部署目录执行 `rbnx shutdown`。随后再停止 Webots 仿真和由示例记录的 RViz2 进程：

```bash
cd /path/to/robonix/examples/webots
# 仅在没有通过 Ctrl+C 停止 rbnx boot 时执行：
rbnx shutdown
bash sim/stop.sh
```

`sim/stop.sh` 会对该示例的 Compose 项目执行 `docker compose down`，并停止启动脚本记录的 RViz2 进程。它保留可复用的镜像、Webots 资源卷和软件包构建缓存，不会按进程名扫描或终止 Robonix 与软件包进程，因此不能代替 `rbnx shutdown`。

## 排错

### Webots 或 RViz2 窗口未出现

```bash
printf 'DISPLAY=%s\n' "${DISPLAY:-<unset>}"
docker ps --filter name=robonix_tiago_sim
```

本地图形桌面通常使用 `DISPLAY=:0`。若日志包含 X11 权限错误，按 `start.sh` 打印的 `xhost` 命令授权本地 Docker 用户。

:::info[无本地桌面时使用浏览器流]
`ROBONIX_SIM_STREAM=1` 会启动浏览器查看器：主机存在 `/dev/nvidia0` 时自动选择 NVIDIA Xorg，否则回退到 Xvfb 软件渲染。Xvfb 不需要 NVIDIA 设备，但速度明显较低。

```bash
ROBONIX_SIM_STREAM=1 bash examples/webots/sim/start.sh
```

本机打开 `http://127.0.0.1:8080/`。查看器连接优化后的 WebSocket 端口 `1235`，不要连接 Webots 原始端口 `1234`。端口可分别通过 `ROBONIX_SIM_VIEWER_PORT` 和 `ROBONIX_SIM_STREAM_PORT` 覆盖。

远程机器运行时，把查看器和 WebSocket 一起转发：

```bash
ssh -N \
  -L 18080:127.0.0.1:8080 \
  -L 11235:127.0.0.1:1235 \
  user@server
```

然后打开 `http://127.0.0.1:18080/?wsPort=11235`。
:::

### `audio_driver` 启动失败

先检查 ALSA 是否识别到硬件，再对照 `audio_driver` 日志中的设备名：

```bash
arecord -l
aplay -l
rbnx logs -t audio_driver -l warn
```

`-l` 只列硬件设备，不列 `null` 之类的 ALSA 插件。有硬件但日志报打不开设备时，按第 5 节“选择音频设备”显式指定 `hw:N,M`，或改用 `plughw:N,M` 让 ALSA 重采样。两条命令都列不出任何设备，才按同一张提示卡退回空设备；此时第 6 节的语音步骤无法验证。

### 软件包启动失败

先读启动摘要中点名的提供方日志，不要只看启动器日志尾部：

```bash
ls -1 rbnx-boot/logs
provider_id=tiago_lidar
tail -n 120 "rbnx-boot/logs/${provider_id}.log"
```

### 远程软件包不是最新版本

`rbnx-boot/cache/` 会复用已克隆的上游仓库。显式更新：

```bash
rbnx update
```

更新会改变实际运行的源码修订号；团队复现问题时，应同时记录部署仓库和每个远程软件包的提交号。

## 下一步

- [图形客户端](./client.md)：把 `rbnx chat` 换成网页界面，可以看 RTDL 树、用客户端电脑的麦克风和扬声器。
- [系统部署与启动](../architecture/deployment-and-startup.md)：理解真实启动所有权、生命周期和日志位置。
- [本体接入指南](../integration-guide/vendor-onboarding.md)：把 Webots 能力提供方替换为真实机器人硬件。
- [开发者指南](../developer-guide.md)：从 template-rbnx 开发自己的原语、服务或技能。
- [接口目录](../interface-catalog/index.md)：查询标准契约与 ROS 接口定义。
