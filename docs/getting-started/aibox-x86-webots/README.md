---
title: x86 Ubuntu 仿真 + AiBOX Robonix 完整部署流程
slug: /getting-started/aibox-x86-webots
---
# x86 Ubuntu 仿真 + AiBOX Robonix 完整部署流程

<div class="procedure-meta">
  <div><strong>目标平台</strong>x86_64（Ubuntu 22.04）+ aarch64（Ubuntu 22.04，Firefly AiBOX，RK1828）</div>
  <div><strong>运行方式</strong>两台机器经 IP 互联（局域网或虚拟组网均可）：x86 跑 Webots 仿真与设备驱动，AiBOX 跑 Robonix 大脑、mapping/nav2（原生进程）与 explore（docker）</div>
  <div><strong>源码仓库</strong>syswonder/robonix</div>
</div>

示例 IP（[RFC 5737](https://datatracker.ietf.org/doc/html/rfc5737) 文档专用地址，与 `examples/webots-aibox/*.example` 里的占位符一致，按实际替换）：

```text
x86   = 192.0.2.10   （主机 PC，跑仿真与 driver）
AiBOX = 192.0.2.20   （跑 Robonix 大脑）
```

下文把 x86 简称"主机"，AiBOX 简称"板子"。部署目录 `examples/webots-aibox/` **随仓库自带**，不需要手工拼装——要填的只有两台机器的事实（IP、ssh 别名、VLM 凭据）。

:::tip[与 x86 仿真 + Jetson 部署的区别]
Jetson 方案（[x86 仿真 + Jetson 部署](/getting-started/jetson-x86-webots)）的数据面是 FastDDS；本方案两台机器都是 ROS 2 Humble + **rmw_zenoh_cpp**，由 sim 容器里的 Zenoh router 中转。Jetson 方案的部署目录需要手工拷贝拼装，本方案在仓库里直接给出。
:::

## 1. 总体结构与数据面

```text
AiBOX 板 (192.0.2.20)
└── rbnx boot（来自 examples/webots-aibox/robonix_manifest.yaml）
    ├── Atlas            0.0.0.0:50051   ← 主机 driver 经 ssh 注册到这里
    ├── Soma             0.0.0.0:50091   （robot_yaml 用 examples/webots/soma.full.yaml）
    ├── Executor         0.0.0.0:50061
    ├── Pilot            0.0.0.0:50071   → 调用外部 VLM（OpenAI 格式）
    ├── Liaison          0.0.0.0:50081
    ├── Vitals           0.0.0.0:50093   （经 docker 网桥 172.17.0.1 反查 Soma）
    ├── 4 × tiago driver （ssh 包裹 → 主机 sim 容器内）
    ├── mapping          原生进程，板上 ROS 2 Humble，slam_toolbox
    ├── nav2             原生进程，板上 ROS 2 Humble
    └── explore          docker（skill 镜像，注册后保持 INACTIVE 待任务激活）

x86 主机 (192.0.2.10)
├── Webots 容器  robonix_tiago_sim（内含 Zenoh router，listen :7447）
└── tiago_camera / tiago_chassis / tiago_lidar / tiago_health（docker-exec 进容器）
```

两点与直觉不同的地方：

**mapping 与 nav2 在板上跑裸进程，不是容器。** 这由清单里 `manifest: package_manifest.jetson-native.yaml` 选择——该名字的含义就是"宿主机原生、无容器"（软件包的同一套脚本还有默认形态，即 x86 单机部署用的 docker 形态，见 [x86 仿真 + Jetson 部署](/getting-started/jetson-x86-webots) 一节对映射/导航容器的说明）。原生形态直接用板子自己安装的 ROS 2 Humble，省去在板上构建几 GB 的 ROS 镜像。

**数据面是星型的 Zenoh 拓扑。** 多播发现跨不过两台机器之间的链路，所以每个参与者都是同一台 router 的 client：

```text
sim 容器内 rmw_zenohd（router, listen :7447）
   ▲                 ▲
   │ 容器内直连        │ tcp/192.0.2.10:7447
驱动（sim 容器内）    板子：mapping / nav2 / explore
```

- 板上原生进程读环境变量 `ZENOH_SESSION_CONFIG_URI` 指向的会话配置（`run/zenoh-aibox.json5`，见 4.3 节），`mode: client` 指向 router；
- explore 容器不读该文件——它的 entrypoint 用 `ROBONIX_ZENOH_ROUTER` / `ROBONIX_ZENOH_MODE` 环境变量把镜像内自带的默认模板改写成等效 client 配置；
- sim 容器内的 driver 与 router 同网络命名空间，容器内直连。

控制面与数据面分离：driver 的生命周期注册走 Atlas（`BOARD_ATLAS` 指向板子 ：50051），ROS 话题走 Zenoh。Pilot 调用 VLM 走 HTTPS，主机有代理而板子没有直连外网时用 ssh 反向隧道（4.4 节）。

## 2. 两台机器的准备

### 2.1 互联与免密

板子拉起 driver 走 ssh，这条链路在 `rbnx boot` 里无人值守执行，必须非交互成功。约定 ssh 别名 `aibox-ubuntu`（部署目录的 `remote.env:HOST_SSH` 引用它）：

```bash
# 在板子上生成密钥
ssh-keygen -t ed25519 -N '' -f ~/.ssh/id_ed25519

# 公钥追加到主机的 authorized_keys（scp 或 ssh-copy-id 均可）
ssh-copy-id <user>@192.0.2.10

# 板子 ~/.ssh/config 里写别名
cat >> ~/.ssh/config <<'EOF'
Host aibox-ubuntu
    HostName 192.0.2.10
    User <user>
    IdentityFile ~/.ssh/id_ed25519
EOF

# 验证：不能弹密码
ssh -o BatchMode=yes aibox-ubuntu 'docker ps'
```

**预期结果：** 最后一条 ssh 直接打印主机上的容器列表，状态码 0。

### 2.2 主机（x86）

- docker 与 docker compose plugin（Webots 仿真容器）；GPU headless 模式还需 NVIDIA 驱动与 `nvidia-container-toolkit`，纯 CPU 可用 `ROBONIX_FORCE_CPU=1` 降级。
- git（checkout 会被 sim 容器 bind-mount，driver 代码从这里执行）。
- 主机不需要装 ROS 2，仿真容器自带。

### 2.3 板子（AiBOX）

实测环境：Ubuntu 22.04.5 LTS / aarch64 / ROS 2 Humble。

```bash
sudo apt install -y build-essential git curl docker.io docker-compose-v2

# ROS 2 Humble 与 rmw_zenoh_cpp（mapping/nav2 原生运行依赖）
sudo apt install -y ros-humble-ros-base ros-humble-rmw-zenoh-cpp

# Rust（rbnx 与系统组件都是 Rust）
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"
```

板侧 docker 的拉镜像出口、存储驱动等运维配置与单机部署相同，这里不展开；`explore` 的镜像在首次 `rbnx build` 时构建。

### 2.4 获取代码并构建（两台机器各自执行）

```bash
git clone <robonix-仓库地址> ~/robonix     # 板子上通常是 /root/robonix
cd ~/robonix
cargo build --workspace                    # boot 会从 target/debug 拉起全部系统组件
```

**预期结果：** 构建状态码 0；`target/debug/rbnx` 存在。

两台机器的 checkout 应为**同一 commit**：主机 checkout 是 driver 实际执行的代码（sim 容器 bind-mount 它，`remote_build.sh` 也在那边跑 codegen），版本偏移会在最难排查的地方暴露。

## 3. 主机上：启动 Webots 仿真

```bash
cd ~/robonix
bash examples/webots/sim/start.sh --world office.wbt
```

脚本直到打印 `[sim/start] live odom, RGB, and lidar data received` 才算就绪（它在等真实的传感器话题，不是容器 Up）。之后脚本转入 `docker compose logs -f` 常驻——Ctrl-C 只退出日志跟踪，仿真继续跑。重复执行是安全的（compose 复用运行中的容器），但**永远不要起第二个仿真**。

无显示器的服务器用 GPU headless 模式：

```bash
WEBOTS_HEADLESS_MODE=nvidia bash examples/webots/sim/start.sh --world office.wbt
```

:::note[RViz]
`start.sh` 会自动在容器内拉起 rviz2（与栈共用同一数据面），`/map`、`/scanner`、`/goal_pose` 都能看到。headless 服务器上如需在桌面查看，用 `ROBONIX_SIM_STREAM=1` 的浏览器串流模式，见 `examples/webots/sim/README.md`。
:::

## 4. 板子上：部署目录与配置

部署目录就是仓库里的 `examples/webots-aibox/`。要填的机器事实只有两处：跨机事实进 `remote.env`（板子部署目录内，本地文件不入库），板侧环境进 `/root/robonix/run/`（`boot.env` 按**绝对路径** source 它们）。

### 4.1 跨机事实：remote.env

```bash
cd /root/robonix/examples/webots-aibox
cp remote.env.example remote.env
```

```bash
HOST_SSH=aibox-ubuntu            # 2.1 节配置的板→主机 ssh 别名（BatchMode 可用）
HOST_WEBOTS_ROOT=/home/<user>/robonix   # 主机 checkout 根目录
SIM_HOST_IP=192.0.2.10           # 主机一个板子可达的 IP；driver 对 Atlas 广播它
BOARD_ATLAS=192.0.2.20:50051     # 板子地址；主机 driver 据此注册到板上 Atlas
```

`SIM_CT` 可不填（默认 `robonix_tiago_sim`，与 compose 的容器名一致）；自定义过 `ROBONIX_SIM_CONTAINER` 时才需要。

### 4.2 板侧环境：run/ 三个文件

```bash
mkdir -p /root/robonix/run
cp examples/webots-aibox/webots-ubuntu.env.example /root/robonix/run/webots-ubuntu.env
cp examples/webots-aibox/zenoh-aibox.json5.example /root/robonix/run/zenoh-aibox.json5
cp examples/webots-aibox/vlm.env.example           /root/robonix/run/vlm.env
```

三个文件各自服务一类消费者，把 `192.0.2.10` / `192.0.2.20` 换成真实地址即可：

| 文件 | 谁读它 | 要改什么 |
|---|---|---|
| `run/webots-ubuntu.env` | `boot.env` source，全栈继承 | 两个 IP；走 VLM 隧道时取消代理三行的注释 |
| `run/zenoh-aibox.json5` | mapping/nav2 初始化 rmw 时读（`ZENOH_SESSION_CONFIG_URI` 指向） | `connect.endpoints` 里的主机 IP |
| `run/vlm.env` | Pilot（清单里 `${VLM_*}` 引用，缺一 Pilot 拒绝启动） | 真实凭据；此文件不出板 |

`run/` 是部署机位目录，不在 git 里；`webots-ubuntu.env` 里 `ROBONIX_ADVERTISE_HOST=172.17.0.1` 是 docker 网桥地址，Vitals 等板上容器经它反查板上的组件，一般不用动。

### 4.3 VLM 反向隧道（可选）

板子不能直连 VLM 上游、而主机有 HTTP 代理（如监听 `127.0.0.1:7897`）时，在主机开一条反向隧道，独占一个终端：

```bash
ssh -N -o ServerAliveInterval=30 -R 127.0.0.1:18080:127.0.0.1:7897 aibox
```

并按 4.2 节打开 `run/webots-ubuntu.env` 里的代理三行（`HTTPS_PROXY`/`HTTP_PROXY` 指向 `127.0.0.1:18080`）。先验证全链路再启动，否则故障会拖到最后一个对话任务才暴露：

```bash
# 主机：本地代理有出口（任何 HTTP 状态都行，超时不行）
curl -sx http://127.0.0.1:7897 -o /dev/null -w '%{http_code}\n' https://www.baidu.com

# 板子：隧道 → 主机代理 → VLM 上游
set -a; source /root/robonix/run/vlm.env; set +a
curl -sx http://127.0.0.1:18080 -o /dev/null -w '%{http_code}\n' "$VLM_BASE_URL"
```

**预期结果：** 两条 curl 都返回 HTTP 状态码（如 `200`）。

## 5. 构建并启动全栈

```bash
ssh aibox    # 之后所有命令在板子的交互终端里
cd /root/robonix/examples/webots-aibox
. ./boot.env
rbnx build -f robonix_manifest.yaml
rbnx boot -f robonix_manifest.yaml --no-update-check -v
```

`rbnx build` 会 clone 三个上游包（mapping / nav2 / explore）进 `examples/webots/rbnx-boot/cache/`，并经 ssh 在主机侧对四个原语包跑 codegen；`rbnx boot` 逐层等待注册后再起下一层。

**预期结果（热启动实测）**，按顺序出现：

```text
[ OK ]  atlas / executor / soma            （秒级）
[ OK ]  tiago_chassis / tiago_camera / tiago_lidar / tiago_health   ACTIVE（各 3-10 s）
[ OK ]  vitals / pilot / liaison
[ OK ]  mapping                ACTIVE（约 30 s）
[ OK ]  nav2                   ACTIVE（约 40 s）
[ OK ]  explore                INACTIVE（INACTIVE 是它的设计态，任务到来才激活）
✓ 8 component(s) up
```

boot 前台常驻；拆除用另一终端执行 `rbnx shutdown`（第 7 节）。

## 6. 验证

```bash
rbnx caps
```

**预期结果：** 12 个 provider——`tiago_chassis` / `tiago_camera` / `tiago_lidar` / `tiago_health`、`mapping`、`nav2`、`explore`（INACTIVE）、`pilot`、`liaison`、`vitals`、`soma` 全部 ACTIVE，`executor` REGISTERED（首个任务到来才激活，属正常）。

`rbnx ask` 是 chat 的非交互形式（同一 gRPC 通路，单条 prompt、事件流打到 stdout），按顺序发三条：

1. **相机链路**：`rbnx ask "What can you see in front of the robot?"` — 事件流里应出现 `snapshot` 调用与 640×480 图像结果，随后 VLM 给出画面描述。
2. **探索闭环**：`rbnx ask "Explore the current room and report what you find."` — 轮询返回的任务直到终态。成功终态应对应 office 世界约 90 m² 的覆盖（RViz 里地图应明显扩张）。工具调用成功不等于任务成功，要看任务级终态。
3. **任务快照**：`rbnx ask "What tasks are currently running?"` — Pilot 返回当前 Executor 任务快照，探索结束后应为空。

:::warning[跨 rmw_zenoh 的导航目标确认超时]
Nav2 行为树的 `default_server_timeout`（BT 节点等待 action server 确认目标的上限，单位毫秒）在同机回环假设下默认 20 ms。本部署的确认要跨板↔主机的 Zenoh 链路，实测 RTT 均值 5–9 ms、毛刺 15–37 ms，20 ms 贴着均值，会间歇性掐死导航目标（`Timed out while waiting for action server to acknowledge`），探索退化成原地打转。部署目录自带的 `config/nav2_params.yaml` 已设为 `200`（一个数量级余量，只放宽最坏情况的等待上限，不影响成功路径耗时），本页的验证数字以此为前提；换用自己的参数文件时请保留该值。
:::

## 7. 日常启停

```bash
# 板子，部署目录内：
rbnx shutdown
# 主机，最后停仿真：
bash ~/robonix/examples/webots/sim/stop.sh
```

`rbnx shutdown` 先经 Atlas 逐个通知远端 driver，再执行各原语包的 stop 钩子（先杀主机上残留的 ssh 包裹进程、再扫容器内 driver），最后反向停板上的系统组件并删除 `rbnx-boot/state.json`。重启 = shutdown + boot 全流程；某个 driver 崩溃后 boot 无法单独重启它，走完整循环。

下次启动前如提示 provider 名字被占用，先查残留包裹进程（两台机器都查）并 SIGINT 之，不要干等 Atlas 的 GC：

```bash
pgrep -fa "rbnx start"
```

## 排错

- **`ssh -o BatchMode=yes aibox-ubuntu true` 失败** — 2.1 节的密钥或别名没配对；boot 会在拉起第一个原语时失败，先修 ssh。
- **包裹脚本报 `sim container ... is not running`** — start 钩子启动时先探容器，仿真没起（或起在了别的容器名下）。先回第 3 节把仿真跑起来，再 boot。
- **explore 冷启动超时** — 首次 boot 时 explore 的容器构建发生在 Soma 的注册等待窗口内，冷环境可能超过注册超时。`rbnx shutdown` 后再 boot 一次，热启动是绿的。
- **Pilot 在 `rbnx caps` 里缺失 / `rbnx ask` 找不到它** — Pilot 启动早期要探测 VLM 的上下文窗口，该请求无超时，隧道卡住会把启动挂死。重启 boot；复现则先查 4.3 节的隧道。
- **重启 provider 报 already registered** — 上一实例要等 Atlas GC（90 s 后标 TERMINATED，约 10 min 后移除）。先按第 7 节杀掉残留的 `rbnx start` 包裹进程，再整栈重启。
- **重新部署后话题或 /map 消失** — 机器地址变了。刷新 `run/webots-ubuntu.env`、`run/zenoh-aibox.json5` 的 IP 与 `remote.env` 的 `SIM_HOST_IP` / `BOARD_ATLAS`，然后 shutdown + boot。
- **导航目标频繁 FAILED/CANCELED、探索覆盖远小于房间** — 见第 6 节的 `default_server_timeout` 提醒；RViz 里机器人原地打转、frontier 全部 unreachable 是典型症状。
