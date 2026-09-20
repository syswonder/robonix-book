---
title: x86 Ubuntu 仿真 + ARM64 Robonix 完整部署流程
slug: /getting-started/jetson-x86-webots
---
# x86 Ubuntu 仿真 + ARM64 Robonix 完整部署流程

<div class="procedure-meta">
  <div><strong>目标平台</strong>x86_64（Ubuntu 22.04）+ aarch64（Ubuntu 24.04，JetPack 7.2.1，NVIDIA Jetson Orin Nano 8GB）</div>
  <div><strong>运行方式</strong>两台机器经局域网互联：x86 跑 Webots 仿真与设备驱动，Jetson 跑 Robonix 大脑、Scene、mapping/nav2 与 AI 推理</div>
  <div><strong>源码仓库</strong>syswonder/robonix</div>
</div>

示例 IP：

```text
x86    = 192.168.28.175   （主机 PC，跑仿真）
Jetson = 192.168.28.61    （跑 Robonix 大脑）
```

按实际 IP 替换。下文把 x86 简称"主机"，Jetson 简称"板子"。

:::tip[与单机部署的本质区别]
driver 按官方方式跑在主机的 sim 容器里，但由板子上的 Soma 通过 **ssh 包裹脚本**远程拉起并监督——控制面跨机，数据面走局域网。
:::

因此板子必须能**非交互**地 ssh 到主机，这是第 2 节最先做的事。整个跨机部署只新增一个部署目录 `webots-jetson/`，不改框架代码。

## 1. 总体结构与数据面

```text
Jetson Orin (192.168.28.61)
└── rbnx boot（来自 examples/webots-jetson/robonix_manifest.yaml）
    ├── Atlas            0.0.0.0:50051   ← 主机 driver 经 ssh 注册到这里
    ├── Soma             （本地 URDF 语义监督，runtime reader 走本机 Jazzy rclpy）
    ├── Executor         0.0.0.0:50061
    ├── Pilot            0.0.0.0:50071   → 调用外部 VLM（OpenAI 格式）
    ├── Liaison          0.0.0.0:50081
    ├── Vitals           127.0.0.1:50093
    ├── Scene (native)   web :50107      ← 本机 torch，无 docker
    ├── 4 × tiago driver （ssh 包裹 → 主机 sim 容器内）
    ├── mapping          docker, WebUI :8091
    ├── nav2             docker
    ├── explore          docker
    └── （可选）memory / memgraph / voiceprint / speech / audio_*

x86 主机 (192.168.28.175)
├── Webots 容器  robonix_tiago_sim
└── tiago_camera / tiago_lidar / tiago_chassis / tiago_health（容器内）
```

数据面使用 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`。需要做的是把环境变量设对，全文共三处——主机起 sim 时（第 3 节）、板子 `boot.env` 里（第 4 节）、ssh 包裹脚本的 start 里（第 4 节，容器内的 driver 也要继承）。

:::warning[不要改回 zenoh]
实测 rmw_zenoh 0.1.9（sim 容器，humble）与 0.2.10（Jetson，jazzy）发现图互通、数据面断裂：`ros2 topic list` 什么都看得见，`echo /odom` 永远收不到。修它需要 humble 侧升级到 0.2.x，官方 apt 源里没有这个包。FastDDS 的 humble↔jazzy 互操作实测通过，且零安装零运维，环境已经自带此中间件。
:::

## 2. 两台机器的准备

### 2.1 互联与免密

板子拉起 driver 走 ssh，这条链路在 `rbnx boot` 里无人值守执行，所以必须非交互成功：

```bash
# 在板子上生成密钥并拷到主机
ssh-keygen -t ed25519 -N '' -f ~/.ssh/id_ed25519
ssh-copy-id <user>@192.168.28.175

# 验证：不能弹密码
ssh -o BatchMode=yes <user>@192.168.28.175 'docker ps'
```

**预期结果：** 上面这条 ssh 直接打印主机上的容器列表，状态码 0。

主机 ssh 到板子做部署管理，同样 `ssh-copy-id <user>@192.168.28.61` 一次。

### 2.2 板子系统组件

板子实测环境：JetPack 7.2.1 / L4T R39 / Ubuntu 24.04.4 / CUDA 13.2 / 8GB RAM。

```bash
sudo apt update
sudo apt install -y build-essential cmake git curl pkg-config

# docker（重新登录后生效）
sudo usermod -aG docker $USER

# Rust（rbnx 是 Rust 工具）
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"

# uv（mapping/explore 的 codegen 需要）
curl -LsSf https://astral.sh/uv/install.sh | sh

# ROS 2 Jazzy（Soma runtime reader 与数据面工具）
sudo apt install -y ros-jazzy-ros-base ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs

# 8GB 内存保险：16G swap
sudo fallocate -l 16G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### 2.3 国内网络镜像（板上）

板载 WiFi 下官方源的实测速度只有约 21 kB/s，三件套都提前配好：

```bash
# Docker Hub 加速
sudo tee /etc/docker/daemon.json <<'EOF'
{ "registry-mirrors": ["https://docker.m.daocloud.io", "https://dockerproxy.net"] }
EOF
sudo systemctl restart docker

# GitHub 直连不通时走加速代理
git config --global url."https://ghfast.top/https://github.com/".insteadOf "https://github.com/"

# pip 走清华源
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```

### 2.4 获取代码并安装 rbnx（两台机器各自执行）

```bash
git clone --recursive <robonix-仓库地址> ~/robonix
cd ~/robonix
make install          # rbnx、代码生成器和系统可执行文件 → ~/.cargo/bin
rbnx setup ~/robonix  # 注册源码树
export PATH="$HOME/.cargo/bin:$PATH"
rbnx --version
```

**预期结果：** `rbnx --version` 正常输出；`make install` 状态码 0。

两台机器的 checkout 应为**同一 commit**。目的是保证主机 checkout 就是 driver 实际执行的代码（sim 容器 bind-mount 它），版本偏移会在最难排查的地方暴露。

### 2.5 板子安装 torch（Scene 的 GPU 推理依赖）

:::warning[不要用 sbsa/cu130 wheel]
它只带 Thor（sm_110/sm_121）内核，在 Orin（sm_87）上 `is_available()=True` 但任何真实算子都报 `no kernel image`。必须用 jp6 stage，构建目标就是 Orin sm_87。
:::

```bash
pip install --user --break-system-packages \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu129 \
  torch==2.8.0 torchvision==0.23.0 torchaudio==2.8.0

# jp6 wheel 不带 CUDA 依赖，补装 CUDA 12.9 用户态库（对 JP7 驱动向前兼容）
sudo apt install -y libcublas-12-9 libcufft-12-9 libcurand-12-9 \
  libcusolver-12-9 libcusparse-12-9 libnvjitlink-12-9 cuda-nvrtc-12-9 \
  libcudnn9-cuda-12 cuda-cudart-12-9 cuda-cupti-12-9 libnvfatbin-12-9 \
  cuda-nvtx-12-9 libopenblas0-pthread
sudo ldconfig
```

为什么这样组合是成立的：

- CUDA 13.2 的驱动/nvcc 本身完全支持 Orin，问题纯粹出在预编译 wheel 的打包选择——`sbsa/cu130`（JP7/CUDA 13）只为新芯片（Thor/Blackwell）打包，Orin 用户留给 jp6 线。想要 cu13+sm_87 只能在板上用 CUDA 13.2 从源码编 torch，数小时且无必要。
- CUDA 保证**新驱动兼容旧 CUDA 用户态**，所以「jp6/cu129 torch（内含 sm_87 内核）+ cu12.9 运行库 → CUDA 13.2 驱动」不需要板上真的安装 CUDA 12.9 toolkit——驱动才是唯一和硬件对话的东西。

验证必须跑真实内核，不要信 `is_available()`：

```bash
python3 - <<'PY'
import torch
print("arch:", torch.cuda.get_arch_list())          # 必须含 sm_87
x = torch.randn(2048, 2048, device="cuda")
(x @ x).sum().item(); torch.cuda.synchronize()
print("CUDA kernel OK")
PY
```

**预期结果：** arch 列表包含 `sm_87`，最后一行打印 `CUDA kernel OK`。

## 3. 主机上：启动 Webots 仿真

```bash
cd ~/robonix
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
bash examples/webots/sim/start.sh --tiago-variant lite
```

环境变量必须在启动 sim 的同一个 shell 里设置：容器内的 Webots ROS 2 进程从它继承数据面实现，和板子保持一致。

确认容器：

```bash
docker ps | grep robonix_tiago_sim
```

**预期结果：** `robonix_tiago_sim` 处于 Up 状态。sim 容器把主机的 `examples/webots` 挂载到 `/robonix_pkgs`，后面第 4 节的 ssh 包裹脚本就是到这里面拉起 driver 的。

## 4. 板子上：创建部署目录

部署目录 `~/robonix/examples/webots-jetson/` 全部是部署侧文件。结构：

```text
webots-jetson/
├── robonix_manifest.yaml   # 全栈清单（§4.5）
├── boot.env                # 板侧环境变量，唯一事实来源（§4.4）
├── remote.env              # 跨机事实：主机 IP / 容器名 / atlas 地址（§4.2）
├── soma.yaml               # 机器人语义描述（拷贝并改 urdf 路径）
├── urdf/tiago_webots.urdf  # 拷贝自 examples/webots/urdf/
├── config/{rtabmap,nav2}_params.yaml   # 拷贝自 examples/webots/config/
└── primitives/tiago_{chassis,camera,lidar,health}/
    ├── package_manifest.yaml   # 拷贝原包，替换 build/start/stop 三行
    └── scripts/{remote_build,start,stop}.sh   # ssh 包裹脚本（§4.3）
```

### 4.1 拷贝现成文件

```bash
cd ~/robonix/examples
mkdir -p webots-jetson
cp -r webots/config webots/urdf webots-jetson/
cp webots/soma.yaml webots-jetson/soma.yaml
# soma.yaml 里 urdf.path 改为 ./urdf/tiago_webots.urdf（相对 yaml 目录解析）
sed -i 's|path: .*tiago_webots.urdf|path: ./urdf/tiago_webots.urdf|' \
  webots-jetson/soma.yaml
```

### 4.2 跨机事实：remote.env

四个 ssh 包裹脚本都 source 这个文件，主机 IP、容器名、atlas 地址只在这里写一遍：

```bash
mkdir -p webots-jetson/primitives/tiago_{chassis,camera,lidar,health}/scripts
cat > webots-jetson/remote.env <<'EOF'
HOST_SSH=<user>@192.168.28.175
HOST_WEBOTS_ROOT=/home/<user>/robonix/examples/webots
JETSON_ATLAS=192.168.28.61:50051
SIM_CT=robonix_tiago_sim
EOF
```

### 4.3 ssh 包裹脚本

三个脚本各管一段：`build` 在主机跑（sim 容器 bind-mount 主机 checkout，生成的 stub 必须落在那边）；`start` 是前台 ssh，stdout/stderr 回流给 Soma，断连即停；`stop` 用 pkill 模式精确杀容器内 driver。以 tiago_chassis 为例，其余三个软件包同构：

```bash
PKG=tiago_chassis   # 对 camera/lidar/health 重复本节，替换名字与 pkill 模式

# ---- scripts/remote_build.sh ----
cat > webots-jetson/primitives/$PKG/scripts/remote_build.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)/remote.env"
exec ssh -o BatchMode=yes "$HOST_SSH" \
  "cd ${HOST_WEBOTS_ROOT}/primitives/tiago_chassis \
   && export PATH=$HOME/.local/bin:$HOME/.cargo/bin:$PATH && bash scripts/build.sh"
EOF

# ---- scripts/start.sh ----
cat > webots-jetson/primitives/$PKG/scripts/start.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)/remote.env"
exec ssh -o BatchMode=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=4 "$HOST_SSH" \
  "export ROBONIX_ATLAS=${JETSON_ATLAS} \
          ROBONIX_ADVERTISE_HOST=${HOST_SSH##*@} \
          ROBONIX_SIM_CONTAINER=${SIM_CT} \
          RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   cd ${HOST_WEBOTS_ROOT}/primitives/tiago_chassis && bash scripts/start.sh"
EOF

# ---- scripts/stop.sh ----
cat > webots-jetson/primitives/$PKG/scripts/stop.sh <<'EOF'
#!/usr/bin/env bash
set -uo pipefail
source "$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)/remote.env"
exec ssh -o BatchMode=yes "$HOST_SSH" "
  docker exec ${SIM_CT} pkill -TERM -f '[p]ython3 -m chassis_driver.driver' 2>/dev/null
  sleep 1
  docker exec ${SIM_CT} pkill -KILL -f '[p]ython3 -m chassis_driver.driver' 2>/dev/null
  true"
EOF

chmod +x webots-jetson/primitives/$PKG/scripts/*.sh
```

三处细节需要注意：`source` 行用 `readlink -f` 向上三层定位 `remote.env`——用 `dirname "$0"/..` 在被 soma 从别的工作目录调用时会解析到错误的层级。start 里 `ROBONIX_ADVERTISE_HOST=${HOST_SSH##*@}` 让 driver 对 Atlas 广播主机的局域网 IP，写成 127.0.0.1 的话 Executor 反拨 MCP 会失败。stop 的 pkill 模式加 `[b]racket` 转义首字母，否则模式匹配到 ssh 自己的 `bash -c` 命令行会把会话杀掉。

pkill 的 driver 模块名按软件包替换，看主机上对应软件包 `scripts/stop.sh` 的原文即可。

再把包清单的 `build/start/stop` 三行换成包裹脚本（capabilities 等其余内容不动）：

```bash
cd ~/robonix/examples/webots-jetson/primitives/$PKG
cp ~/robonix/examples/webots/primitives/$PKG/package_manifest.yaml .
cat >> package_manifest.yaml <<'EOF'
# Jetson 混合部署：清单留在板上供 Soma 监督，build/start/stop 经 ssh 包裹
# 到主机执行（driver 代码 = 主机 checkout，sim 容器 bind-mount）。
build: bash scripts/remote_build.sh
start: bash scripts/start.sh
stop: bash scripts/stop.sh
EOF
```

### 4.4 板侧环境变量：boot.env

```bash
cat > ~/robonix/examples/webots-jetson/boot.env <<'EOF'
# Source this before rbnx build / rbnx boot on the Jetson:
#   cd ~/robonix/examples/webots-jetson && source boot.env

# ── ROS 2 / FastDDS 数据面（不要用 zenoh，见 §1）──
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# ── scene jetson-native ──
export ROBONIX_SCENE_ROS_DISTRO=jazzy

# ── 部署接线 ──
export ROBONIX_SOMA_ROBOT_YAML="$HOME/robonix/examples/webots-jetson/soma.yaml"
export ROBONIX_TIAGO_VARIANT=lite
export MAPPING_WEBUI_HOST=0.0.0.0     # WebUI 默认绑 127.0.0.1
export MAPPING_ENABLE_VIZ=true        # mapping WebUI :8091

# ── VLM（pilot）— 启动前填入自己的 key ──
export VLM_BASE_URL="https://api.openai.com/v1"
export VLM_API_KEY="sk-REPLACE-ME"
export VLM_MODEL="your-model-name"

# JP7 = Ubuntu 24.04：PEP 668 拦 --user pip，scene build.sh 自己不带这个 flag
export PIP_BREAK_SYSTEM_PACKAGES=1
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"

# speech 留空 = 本地 FunASR 后端（重）；设 tencent = 云端轻量路径
export SPEECH_BACKEND=
EOF
```
### 4.5 全栈清单：robonix_manifest.yaml

从官方示例复制后打四个补丁，差异都集中在 `system:` 块：

```bash
cp ~/robonix/examples/webots/robonix_manifest.yaml \
   ~/robonix/examples/webots-jetson/robonix_manifest.yaml
```

```yaml
system:
  atlas:
    listen: 0.0.0.0:50051      # 主机 driver 要从局域网注册，必须 0.0.0.0
  soma:
    robot_yaml: ${ROBONIX_SOMA_ROBOT_YAML}
    runtime_reader_command:    # Soma runtime reader 跑在板子上（本机 Jazzy rclpy）
      - bash
      - -lc
      - source /opt/ros/jazzy/setup.bash && exec python3 -u {script} {config}
  scene:
    manifest: package_manifest.jetson-native.yaml   # 板上原生跑，不走 docker
    config: {}
  executor:
    listen: 0.0.0.0:50061
  pilot:
    listen: 0.0.0.0:50071
  vitals:
    listen: 127.0.0.1:50093
  liaison:
    listen: 0.0.0.0:50081
```

另外两处：`primitive:` 块里 4 个 tiago 软件包的 `path` 改为 `./primitives/<名字>`（`sentinel_timeout_s: 150` 对 WiFi 链路保留官方放宽值）；`mapping`/`nav2` 的 `params_file: config/rtabmap_params.yaml` 等相对路径保持不变，`rbnx boot` 相对 manifest 目录解析。

:::tip[首次启动可以只跑核心栈]
8GB 板上全栈（含本地 FunASR 语音）约 6.9GB。想先跑核心再逐个加，把 `service:`/`skill:` 里 url 拉取的软件包和 speech/memory 相关节注释掉即可，第 8 节给出逐组件的内存实测账。
:::

## 5. 构建并启动全栈

```bash
cd ~/robonix/examples/webots-jetson
source boot.env
rbnx build --no-update-check
rbnx boot --no-update-check
```

`rbnx build` 构建 8 个软件包：4 个 driver（build 经 ssh 在主机执行）+ mapping + nav2 + explore + scene。第一次构建耗时主要来自容器镜像、模型下载和网络。

`rbnx boot` 的时序分两段：Soma stage 1 逐个经 ssh 拉起 4 个 driver 并等 Driver INIT——WiFi 下首帧可能慢，sentinel 已放宽到 150s；stage 2 拉起 scene/mapping/nav2/explore。`rbnx boot` 保持前台运行，Ctrl-C 即关闭部署。

**预期结果：** 启动摘要没有 failures，系统组件显示监听地址，driver 与服务为 `ACTIVE`，explore 为 `INACTIVE`（首次调用时激活），终端最后显示 `9 component(s) up`（全栈为 13）以及 `rbnx-boot/logs` 路径。

## 6. 验证

### 6.1 数据面（板上）

```bash
source /opt/ros/jazzy/setup.bash
timeout 8 ros2 topic hz /odom
timeout 8 ros2 topic hz /head_front_camera/rgb/image_raw
```

**预期结果：** `/odom` 约 10Hz；RGB 在 WiFi 下约 5Hz，插网线后明显更高。

### 6.2 提供方注册（板上）

```bash
export PATH="$HOME/.cargo/bin:$PATH"
rbnx caps --server 127.0.0.1:50051
```

**预期结果：** 4 个 tiago driver 为 `ACTIVE`，且 advertise 地址是**主机的局域网 IP**——出现 127.0.0.1 说明 `ROBONIX_ADVERTISE_HOST` 没生效。

### 6.3 Web UI（主机浏览器）

```text
Scene   http://192.168.28.61:50107/
Mapping http://192.168.28.61:8091/
```

### 6.4 闭环对话（板上）

```bash
rbnx chat
```

测试：

```text
看看前面有什么
向前移动0.5米
```

**预期结果：** Pilot 调用场景服务回答第一句，Executor 经 nav2 驱动底盘执行第二句。对话链路排查方法与单机部署相同，见[快速上手 §6](../quickstart.md#6-提交第一条任务)。

### 6.5 资源占用（内存与"显存"）

Orin 是统一内存架构，CPU 和 GPU 共享同一份 RAM，**没有独立的显存可查**——`nvidia-smi` 的 memory 一栏会显示 `Not Supported`，这是正常的，不是驱动问题。看真实占用用 tegrastats：

```bash
tegrastats --interval 2000
```

输出里重点关心 `RAM 5347/7698MB` 这一段：已用/总量（单位 MB），就是 CPU + GPU 加起来的统一内存占用；旁边的 `lfb` 是最大连续空闲块，数值变小说明内存碎片化在加剧。GPU/EMC 的利用率百分比也在同一行。`Ctrl+C` 退出。

更友好的交互界面是 jtop（来自 `jetson-stats` 包，`sudo pip install jetson-stats` 安装），按 GPU/CPU/RAM 分栏实时刷新。快速粗查用 `free -h` 即可——它看到的"已用"已经包含 GPU 的分配。

## 7. 日常启停

```bash
# ── 启动（按顺序）──
# 主机：起仿真
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && bash ~/robonix/examples/webots/sim/start.sh
# 板子：起全栈
cd ~/robonix/examples/webots-jetson && source boot.env && rbnx boot --no-update-check

# ── 停止（按顺序）──
# 板子：停全栈
rbnx shutdown
# 主机：停仿真
bash ~/robonix/examples/webots/sim/stop.sh
```

正常关闭应先 `rbnx shutdown` 让各组件走完生命周期，再停仿真。`shutdown` 报 stale process groups 的处理见排错一节。

## 8. 可选：全栈软件包与内存预算

8GB 板上以下软件包已实测可全部拉起。在 manifest 的 `service:` 块里加：

```yaml
  - name: voiceprint
    path: ${ROBONIX_SOURCE_PATH}/services/voiceprint
    config: {}
  - name: memgraph
    path: ../../services/memory
    config:
      backend: memgraph
  - name: memory
    path: ${ROBONIX_SOURCE_PATH}/services/memsearch
    config: {}
  - name: speech
    path: ${ROBONIX_SOURCE_PATH}/services/speech
    config:
      speech_backend: ${SPEECH_BACKEND}
      disable_whisper: true
```

`primitive:` 块里加（url 拉取型）：

```yaml
  - name: audio_driver
    url: https://github.com/syswonder/primitive-audio-driver-rbnx
    branch: main
    config: {}
  - name: audio_client_bridge
    url: https://github.com/syswonder/primitive-audio-client-bridge-rbnx
    branch: main
    config: {}
```

逐个添加、逐步实测的内存账（Orin Nano 8GB，统一内存口径，累积 `used`）：

| 增量 | 累积 used | 该组件 RSS |
|---|---|---|
| 基线 9 组件 | 3.3GB | — |
| + memgraph + memory | 3.8GB | memsearch 1303MB、memgraph 97MB |
| + audio_driver + audio_client_bridge | ≈不变 | ≈0 |
| + voiceprint | 3.8GB | 735MB |
| + speech（FunASR 本地后端） | 6.9GB | 3620MB |

13 个软件包全部能到 `ACTIVE`，但本地 FunASR 后端的 speech 把余量压到约 480MB + swap。长期运行建议在 `boot.env` 里 `export SPEECH_BACKEND=tencent`（云后端，构建和运行都不装模型），或不部署 speech。语音后端的完整对比见[语音后端配置](../../appendix/speech-backends.md)。

## 排错

### `ros2 topic list` 看得见话题但收不到数据

数据面实现不一致。确认三处 `RMW_IMPLEMENTATION` 都是 `rmw_fastrtps_cpp`：主机起 sim 的 shell、板子 `boot.env`、ssh 包裹脚本 start。查实际生效值：

```bash
source /opt/ros/jazzy/setup.bash
printenv RMW_IMPLEMENTATION
```

不要用 zenoh 跨这些版本，原因见 §1 的 warning。

### torch 报 `no kernel image is available for execution on the device`

wheel 的构建目标不含本机架构。检查 arch 列表：

```bash
python3 -c "import torch; print(torch.cuda.get_arch_list())"
```

输出里没有 `sm_87` 就是装错了线（sbsa/cu130 只带 sm_110/sm_121）。按 §2.5 重装 jp6/cu129 一套。注意版本配对：torchvision 0.24 配 torch 2.8 会报 `operator torchvision::nms does not exist`。

### 软件包启动失败：pb2 描述符报错

报 `Descriptors cannot be created directly` 是板上系统 `grpcio-tools 1.14.1` 生成了旧式 pb2，被 protobuf ≥4.21 拒绝。codegen 必须走版本受控的 venv（仓库自带 `scripts/run_python_codegen.sh`）；tiago_health 的 `scripts/build.sh` 若裸调 `rbnx codegen`，改成走该脚本后在主机重新 build。

### pip 安装被拒（PEP 668）或 supervision 装不上

Ubuntu 24.04 的 PEP 668 拦 `pip install --user`，`boot.env` 里的 `PIP_BREAK_SYSTEM_PACKAGES=1` 已全局解决；手动装包时记得带 `--break-system-packages`。`supervision==0.14.0` 不支持 py3.12，scene 依赖树里需 pin 到 ≥0.17.1。

### driver 注册了但 Executor 调不到

advertise 地址写成了 127.0.0.1。检查 ssh 包裹脚本 start 里 `ROBONIX_ADVERTISE_HOST=${HOST_SSH##*@}` 是否原样保留，`rbnx caps` 里 driver 的 endpoint 必须是主机局域网 IP。

### `rbnx shutdown` 报 refused stale/mismatched process groups

多发生在全栈刚扩过容后，服务实际都已停止。确认无业务进程后，kill 残留的 `rbnx boot` 与 `rbnx __watch-boot` 进程，删除 `rbnx-boot/state.json`，即可再次 boot。

### mapping WebUI 打不开

两个前提都满足才会监听 :8091：`MAPPING_ENABLE_VIZ=true`（默认 off）且 `MAPPING_WEBUI_HOST=0.0.0.0`（默认绑 127.0.0.1）。mapping 的 gRPC 端口是 50120，不要混用。

### 非交互 ssh 找不到 rbnx/uv

非交互 ssh 不加载 `.bashrc` 的 PATH。ssh 包裹脚本里显式 `export PATH=$HOME/.local/bin:$HOME/.cargo/bin:$PATH`。往 env 文件追加 PATH 时用单引号写法，避免 `$PATH` 被当前 shell 提前展开成字面值。

### 远程长任务挂死 ssh 会话

后台进程握住 stdout 会挂住管道。远程构建/下载统一：

```text
nohup setsid <命令> </dev/null >log 2>&1 &
```

然后轮询日志文件，不挂 ssh 管道。

## 下一步

- [Webots 快速上手](../quickstart.md)：单机版的完整教程，对话、语音、场景切换的细节都在那边。
- [多架构与多运行环境部署](../../architecture/multiplatform-deployment.md)：`manifest:` 选择器与本机/容器边界的权威说明——本教程的 `package_manifest.jetson-native.yaml` 就是它的应用。
- [图形客户端](../client.md)：把 `rbnx chat` 换成网页界面。
- [系统部署与启动](../../architecture/deployment-and-startup.md)：理解 boot 的启动所有权、生命周期和日志位置。
- [接口目录](../../interface-catalog/index.md)：查询标准契约与 ROS 接口定义。
