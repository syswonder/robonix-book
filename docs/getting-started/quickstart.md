# Webots 快速上手


本教程在一台 Linux x86_64 主机上启动 Tiago Webots 仿真、Robonix 系统组件、原语（Primitive）、服务（Service）和技能（Skill），然后通过 Liaison 提交一条自然语言任务。第一次执行会编译 Rust 工作区、构建容器并下载依赖；后续复用缓存时才是快速启动流程。

<div class="procedure-meta">
  <div><strong>目标平台</strong>Ubuntu 22.04 或 Debian 13，x86_64</div>
  <div><strong>运行方式</strong>Docker Compose + Webots GUI</div>
  <div><strong>源码仓库</strong>syswonder/robonix</div>
</div>

## 1. 检查主机

默认图形界面路径需要可用的 X Server 和图形栈，以及 Git、Make、Python 3.10+、Rust stable、uv、Docker Engine 和 Compose v2。

:::note[本教程使用的图形环境]
当前完整 Webots 测试使用 NVIDIA GPU、NVIDIA 驱动和 `nvidia-container-toolkit`；下面的主流程以这条已验证路径为准。仓库的基础 Compose 也映射了 `/dev/dri`，镜像内还包含 Xvfb，但 Intel/AMD 图形和 CPU 软件渲染尚未纳入完整端到端验收，只作为兼容与排错路径。
:::

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
```

**预期结果：** 版本命令均以状态码 0 退出；Python 版本不低于 3.10，`grpc_tools: ok` 可见，Docker 命令不需要 `sudo`，`docker compose version` 显示 Compose v2。

## 2. 安装 Robonix

克隆源码，检出本手册校验过的提交，并初始化能力约定与接口定义子模块：

```bash
git clone --recurse-submodules https://github.com/syswonder/robonix.git
cd robonix
git checkout --detach 181d3eb974fd495a795ed120a0a4c6e6f342d179
git submodule update --init --recursive

git rev-parse HEAD
git submodule status --recursive
make install
```

`make install` 将 `rbnx`、代码生成器和 Atlas、Executor、Soma、Vitals、Pilot、Liaison 等系统可执行文件安装到 `~/.cargo/bin`，并把当前克隆目录登记为 Robonix 源码根目录。确认安装结果：

```bash
export PATH="$HOME/.cargo/bin:$PATH"
rbnx --version
rbnx path root
```

**预期结果：** `git rev-parse HEAD` 输出 `181d3eb974fd495a795ed120a0a4c6e6f342d179`；`rbnx path root` 输出刚克隆的 Robonix 仓库绝对路径。

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

构建读取 `examples/webots/robonix_manifest.yaml`，准备本地软件包，并把清单中通过 `url:` 引用的音频、建图、导航和自主探索仓库放入 `rbnx-boot/cache/`。语音服务默认使用本地 FunASR 流式识别；第一次构建会安装本地语音依赖并下载 `paraformer-zh-streaming` 模型。腾讯云是可选后端，账号开通、密钥和部署清单配置见[语音后端配置](../appendix/speech-backends.md)。Whisper 当前支持不完善且模型体积过大，本快速上手不启用它。第一次构建时间主要取决于容器镜像、模型下载、网络和 CPU；不要把冷启动时间与复用缓存后的启动时间混为一谈。

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

### 终端 2：Robonix 系统

```bash
export PATH="$HOME/.cargo/bin:$PATH"
export VLM_API_KEY='sk-...'
export VLM_BASE_URL='https://api.example.com/v1'
export VLM_MODEL='your-model-name'
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

cd /path/to/robonix/examples/webots
rbnx boot
```

:::tip[测试机没有音频设备]
先按上面的正常流程启动。只有启动摘要明确显示 `audio_driver` 因找不到输入或输出设备而失败、且本次不需要验证真实录放音时，才停止本次启动，在同一终端设置空设备后重新执行 `rbnx boot`：

```bash
export AUDIO_MIC_DEVICE='null'
export AUDIO_SPEAKER_DEVICE='null'
rbnx boot
```

字符串 `null` 选择 ALSA 内置的空 PCM，不需要创建 `.asoundrc`。
:::

Webots 部署清单配置以下系统组件和软件包：

- 系统：Atlas、Soma、Vitals、Scene、Executor、Pilot、Liaison
- 原语：Tiago 底盘、RGB-D 相机、二维激光雷达，以及通过独立仓库取得的 ALSA 音频和客户端音频桥
- 服务：记忆、语音、声纹、建图、导航
- 技能：探索；启动后保持 `INACTIVE`，第一次被调用时由 Executor 激活

**预期结果：** 启动摘要中没有 `failures`，系统组件显示监听地址，原语与服务为 `ACTIVE`，Explore 为 `INACTIVE`。终端最后显示组件已启动以及 `rbnx-boot/logs` 路径。

Scene 调试页默认位于 [http://127.0.0.1:50107/](http://127.0.0.1:50107/)。页面同时显示二维占据栅格、语义对象、机器人位姿、三维点云和相机流；这些数据只有在相应提供方已经启动并发布后才会出现。

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

`Esc` 中断当前交互回合，`Ctrl+C` 退出文本用户界面（Text User Interface，TUI）。

**预期结果：** 终端界面显示用户输入、规划状态、机器人任务描述语言（Robot Task Description Language，RTDL）能力调用与最终回复；Explore 被调用时会从 `INACTIVE` 转为 `ACTIVE`，其提供方日志是 `rbnx-boot/logs/explore.log`。

## 7. 选择其他 Webots 场景

回到 Robonix 仓库根目录选择内置场景：

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
| `office.wbt`<br />![Webots 办公室场景预览](/img/webots/office.jpg) | `apartment.wbt`<br />![Webots 公寓场景预览](/img/webots/apartment.jpg) |
| `complete_apartment.wbt`<br />![Webots 完整公寓场景预览](/img/webots/complete_apartment.jpg) | `break_room.wbt`<br />![Webots 休息室场景预览](/img/webots/break_room.jpg) |
| `kitchen.wbt`<br />![Webots 厨房场景预览](/img/webots/kitchen.jpg) |  |

`office.wbt` 使用镜像内的资源种子。第一次运行其他内置场景前，先下载一次 Cyberbotics 官方离线资源包；后续启动会复用持久化缓存：

```bash
ROBONIX_WEBOTS_DOWNLOAD_ALL_ASSETS=1 \
  bash examples/webots/sim/start.sh --world apartment.wbt
```

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

:::warning[替代渲染路径尚未完成完整验收]
没有可用 X Server 时，可以用 Xvfb 做低速排错：

```bash
WEBOTS_HEADLESS_MODE=xvfb ROBONIX_FORCE_CPU=1 \
  bash examples/webots/sim/start.sh

ROBONIX_SIM_STREAM=1 bash examples/webots/sim/start.sh
```

`xvfb` 使用 CPU 软件渲染，不需要 NVIDIA 设备，但速度明显较低。当前浏览器流式路径会由 Compose 申请 NVIDIA 设备；使用 `ROBONIX_SIM_STREAM=1` 前，主机必须已安装 NVIDIA 驱动和 `nvidia-container-toolkit`。基础 Compose 虽然把 `/dev/dri` 映射进容器，但 Intel/AMD 与 Xvfb 路径都需要在目标主机单独验收，不能用 NVIDIA CI 的结果代替。
:::

### `audio_driver` 在无声卡主机上启动失败

如果本机本应有麦克风或扬声器，先检查 ALSA 是否识别到硬件，再对照 `audio_driver` 日志中的设备名：

```bash
arecord -l
aplay -l
rbnx logs -t audio_driver -l warn
```

`-l` 只列硬件设备。确认机器确实没有音频设备、且本次只验证非音频链路时，按第 5 节的“测试机没有音频设备”提示卡选择 ALSA 空设备；不要把该配置用于真实语音测试。

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

- [系统部署与启动](../architecture/deployment-and-startup.md)：理解真实启动所有权、生命周期和日志位置。
- [本体接入指南](../integration-guide/vendor-onboarding.md)：把 Webots 能力提供方替换为真实机器人硬件。
- [开发者指南](../developer-guide.md)：从 template-rbnx 开发自己的原语、服务或技能。
- [接口目录](../interface-catalog/index.md)：查询标准契约与 ROS 接口定义。
