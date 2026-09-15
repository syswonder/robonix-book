import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# 图形客户端

[Robonix Client](https://github.com/syswonder/robonix-client) 跑在**操作者自己的设备上**，不装在机器人上。机器人只负责跑 Robonix，人在自己的电脑上开一个网页去指挥它。

这是它和 `rbnx chat` 的根本区别：`rbnx chat` 是机器人本机的终端界面，用机器人的音频设备；Client 用的是你这台设备的麦克风和扬声器，人不必守在机器人跟前。

目前支持 Linux、macOS 和 Windows，后续会扩展到手机和平板。

它只需要机器人 Atlas 的地址，Liaison、Executor 和音频能力都由它自己从 Atlas 发现，不用逐个填。

本页依据 [`739f8499`](https://github.com/syswonder/robonix-client/tree/739f8499) 编写。

![Chat 页，一条探索任务正在执行。左侧是四个标签和会话列表，中间是对话，右侧自上而下是 Current Goal、RTDL Forest、Execution history、Node detail 和 Event Log。](/img/ui/cl-chat-live.webp)

右栏是它与 `rbnx chat` 最大的差别，也是排障时真正用得上的部分：

| 面板 | 内容 |
|---|---|
| Current Goal | 当前目标，以及它被解析到哪个提供方、能力约定和操作。`EXECUTOR VERIFIED` 表示 Executor 已校验过这次调用 |
| RTDL Forest | 正在执行的 RTDL 树 |
| Active RTDL | 展开当前树的完整结构 |
| Execution history | 已结束的执行记录，可以回看上一条任务怎么走的 |
| Node detail | 点选某个节点后显示它的提供方、开始时间、耗时、参数与结果 |
| Event Log | 本次会话的状态事件，用来判断任务卡在规划、执行还是等待 |

它和 `rbnx chat` 提交任务的路径相同，都经 Liaison 到 Pilot，区别在三点：客户端是网页而非终端；它能可视化 RTDL 树和执行历史；语音可以用<strong>客户端这台电脑</strong>的麦克风和扬声器，而 `rbnx chat` 用的是机器人本机的音频设备。

客户端跑在操作者的电脑上，不装在机器人上。跟着 [Webots 快速上手](./quickstart.md)做完仿真的话，机器人端就是那台跑 `rbnx boot` 的机器；两者同机时 `--robot-host` 用 `127.0.0.1`。

## 1. 准备机器人端

客户端所在主机必须能访问机器人 Atlas 的监听地址。需要从外部主机连接时，机器人部署清单至少要让 Atlas、Liaison 和 Executor 监听可信局域网或 Tailscale 接口：

```yaml
system:
  atlas:
    listen: 0.0.0.0:50051
  executor:
    listen: 0.0.0.0:50061
  liaison:
    listen: 0.0.0.0:50081
```

Pilot 可以继续只监听机器人本机；Liaison 会发现并调用它。不要把这些端口直接暴露到公网。

若要使用客户端主机的麦克风和扬声器，机器人还需包含反向音频桥：

```yaml
primitive:
  - name: audio_client_bridge
    url: https://github.com/syswonder/primitive-audio-client-bridge-rbnx
    branch: main
    config:
      transport: reverse
      listen_host: 0.0.0.0
      listen_port: 60002
```

客户端会自己从 Atlas 找到这个桥并连上去，不用在 Client 里填端口。`listen_port` 和 Atlas 一样，只应在可信局域网或 Tailscale 上可达，不要暴露到公网。

## 2. 安装

需要 Python 3.11 或更高版本。

<Tabs groupId="client-platform">
<TabItem value="linux" label="Linux">

安装 PortAudio 和 Python 虚拟环境支持：

```bash
sudo apt update
sudo apt install -y libportaudio2 portaudio19-dev python3-venv
```

</TabItem>
<TabItem value="macos" label="macOS">

使用 Homebrew 安装 PortAudio：

```bash
brew install portaudio
```

</TabItem>
<TabItem value="windows" label="Windows">

`sounddevice` 自带 PortAudio DLL，通常不需要额外安装。

音频服务仍然找不到设备时，装一个自带 PortAudio 的 PyAudio 预编译轮子，[elibroftw/pyaudio_portaudio](https://github.com/elibroftw/pyaudio_portaudio/releases) 提供 Python 3.10–3.14 的 `win_amd64` 版本。

</TabItem>
</Tabs>

随后安装 Client：

```bash
git clone https://github.com/syswonder/robonix-client.git
cd robonix-client
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -e '.[audio]'
```

## 3. 启动并连接

假设机器人 Atlas 位于 `192.168.1.50:50051`：

```bash
source .venv/bin/activate
robonix-client --robot-host 192.168.1.50
```

浏览器打开 [http://127.0.0.1:7860/](http://127.0.0.1:7860/)。确认顶部的 **Robot Host** 是机器人地址、**Atlas Port** 是 `50051`，然后点击 **Connect**。状态变为在线后即可提交文本任务。

`--robot-host` 指定机器人；`--host` 指定 Client Web 页面监听在哪个本机地址。只在当前电脑访问页面时保留默认的 `--host 127.0.0.1`。

## 4. 使用页面

左侧四个标签对应四件事：下达任务、看机器人健康、配音频、改连接参数。

### 聊天（Chat）

顶栏是连接信息：**Robot Host** 和 **Atlas Port** 指向机器人，**User** 是提交任务时带的身份，右侧三个指示分别是运行状态、**Hands-free** 开关和连接状态。改了地址要重新 **Connect**。

**会话管理**在左栏。**New session** 开一条新会话，历史会话列在下面，鼠标悬停出现 **Rename** 和 **Delete**。会话之间互不影响，换一条会话等于换一条任务上下文。**Clear** 清空当前会话的消息，不删除会话本身。

中间是对话区。一轮任务会依次出现这些内容：

```text
STATUS  SUBMITTED TASK; WAITING FOR PILOT STREAM.
STATUS  PLANNING THE NEXT STEP
STATUS  IN_PROGRESS
STATUS  DONE
ROBONIX 回复文本
```

任务执行期间再输入一句，不会新开任务，而是作为任务调整指令（steer）加进去，消息前面标 `ADDED TO RUNNING TASK`：

![同一条会话里的三轮对话。第二、三轮标着 ADDED TO RUNNING TASK，说明它们加进了正在执行的任务而不是另起一条。](/img/ui/cl-chat-multi.webp)

底部输入框旁边，空闲时是 **Send**，任务执行时变成红色的 **ABORT ALL TASKS**。**Start recording** 按钮和 `F2` 等价。

右栏是这个客户端比 `rbnx chat` 多出来的部分，排障时真正有用：

| 面板 | 内容 | 什么时候看 |
|---|---|---|
| **Current Goal** | 当前目标，以及它被解析到哪个提供方、能力约定和操作。`EXECUTOR VERIFIED` 表示 Executor 已校验这次调用 | 判断 Pilot 有没有选对能力 |
| **RTDL Forest** | 正在执行的 RTDL 树。标题栏给出 `1 active · plan 10 · round 1 · 1 call(s)` | 看任务被拆成了什么结构 |
| **Active RTDL** | 展开当前树，执行中的节点是橙色 | 看卡在哪个节点 |
| **Execution history** | 已结束的执行记录，数字是条数 | 回看上一条任务怎么走的 |
| **Node detail** | 点选某个节点后显示它的提供方、开始时间、耗时，以及展开的调用参数与结果 | 看某次调用传了什么、返回了什么 |
| **Event Log** | 本次会话的状态事件，带时间戳 | 判断卡在规划、执行还是等待 |

Forest 里显示的是**当前正在执行的那一个 plan**，不是整条任务。一个任务通常由多个 plan 组成，Pilot 每次 `PLANNING THE NEXT STEP` 就产生下一个，界面上的 `Plan 10 · round 1` 就是它的编号。**一个 plan 执行完，它的树随即消失**，没有下一个时 Forest 显示 `No RTDL tree is currently executing`。要回看已经执行完的 plan，用 **Execution history**。

### 健康状态（Vitals）

Vitals 回答“机器人本身怎么样”，与任务是否成功无关。任务失败时先看这一页，能分清是硬件、提供方还是规划的问题。

![Vitals 页。顶栏是整机摘要，左栏是部件树，中间是 URDF 模型，右栏是选中部件的详情，底部是模块与提供方表。](/img/ui/cl-vitals.webp)

顶栏是整机摘要：**HARDWARE** / **BATTERY** / **SOFTWARE** 各自的健康计数、上次更新时间，以及 Soma、Hardware、Modules、Atlas 四个数据源的指示灯。右侧 **Alerts** 带数字，点开是告警中心。机器人有异常时名字后面出现 `WARN` 之类的徽标。

左栏 **Body** 是部件树，来自本体模型（Soma）声明的结构，例如底盘、左右轮、电池、头部相机、雷达、音频。每项后面一个状态点。点某个部件，中间的 URDF 模型会高亮它，右栏切换到它的详情。

右栏分三段：**IDENTITY** 是类型、父部件、由哪些提供方支撑、对应的 URDF link 和 joint；**STATUS** 是聚合健康、直接健康、就绪状态和数据来源；**SIGNALS** 是该部件上报的原始健康信号，没有就显示 `No direct health signals`。

底部两个表切换看：**Modules** 是 Robonix 各模块（executor、pilot、vitals 等）的健康、状态、来源和 TTL；**Providers** 是各能力提供方。`SELF_REPORTED` 表示该状态由模块自己上报，TTL 是这条状态的有效期。

### 音频（Audio）

![Audio 页。上半是音频服务与路由，下半选择输入输出的提供方和具体设备，并提供测试按钮。](/img/ui/cl-audio.webp)

Client 启动时会自动起本机音频服务。**Audio Device Server** 仍显示 offline 时，点 **Start Audio**。

**Robonix Audio Route** 决定语音从哪来、到哪去。点 **Refresh Route** 拉取当前可用的提供方，然后：

1. **Input Primitive** 和 **Output Primitive** 都选 `audio_client_bridge`，表示用这台电脑的设备；选 `audio_driver` 则用机器人本机的设备。
2. 在 **Input Device** 和 **Output Device** 里选具体的麦克风和扬声器，列表不全时点 **Refresh Devices**。
3. 点 **Apply Route** 生效。
4. 分别点 **Test Microphone** 和 **Test Speaker** 验证，前者应当看到输入电平，后者应当听到声音。

路由生效后按 `F2` 开始一次语音输入，页面上的录音按钮作用相同。免按键语音要另外打开顶栏的 **Hands-free**。

**Audio Log** 记录音频服务自己的日志，设备打不开时先看它。**Client Bridge Diagnostics** 检查这台电脑到机器人音频桥的连通性。

**Voiceprint** 区域用于声纹：**Enroll Voice** 录一段音注册当前用户。部署开启访问控制时，语音任务要先通过声纹才会进入 Pilot。

### 设置（Settings）

![Settings 页。机器人地址、Atlas 端口、用户标识、录音时限和可选的 Liaison Endpoint。](/img/ui/cl-settings.webp)

Settings 保存机器人地址、Atlas 端口、用户标识（**User ID** 与 **User Name**）、**Record Seconds** 录音时限，以及可选的 **Liaison Endpoint**。

点 **Save Settings** 写入 `~/.config/robonix-client/settings.yaml`，页面同时在浏览器本地存储保留一份。

**Liaison Endpoint 正常留空**，让 Client 从 Atlas 发现它。只有在 Liaison 不通过 Atlas 暴露、或要强制指向某个实例时才填。

## 5. 最小验收

按顺序完成以下检查：

1. Connect 后状态为在线。
2. 发送“目前有哪些能力？”并收到文本回复；部署已接入相机和图像理解能力时，再发送“你能看到什么？”验证视觉链路。
3. 发送一个持续几秒的任务，在执行期间再提交一句修改要求，确认它作为任务调整指令生效。
4. 打开 Active RTDL，确认显示的运行节点与 Executor 一致。
5. 点击 Stop，确认按钮短暂显示 **Stopping**，随后任务结束且界面回到空闲。
6. 使用音频桥时，确认麦克风测试有输入电平、扬声器测试能播放声音，再测试一次 F2 语音任务。

## 排错

### 一直离线

确认输入的是 Atlas 所在机器，而不是客户端自己的地址：

```bash
nc -vz 192.168.1.50 50051
```

同时检查机器人清单中的 Atlas 不是只监听 `127.0.0.1`。

### 音频桥不可用

确认 Atlas 中存在 `audio_client_bridge`，并且它注册了 `robonix/primitive/audio/bridge_info`。桥接端口由该能力公布，Client 会把其中的环回主机名改写为当前 Robot Host 后主动连接；不要用猜测的固定端口排错。若 Client 已连接但设备列表为空，先检查操作系统是否允许当前终端访问麦克风，以及 PortAudio 是否能枚举设备。

### 文本可用但语音失败

Client 音频链路只负责采集和播放。语音识别、语音合成和唤醒词还要求机器人上的 Speech 服务正常启动，并且 Liaison 能发现所选的输入、输出与语音能力。分别查看 `audio_client_bridge`、`speech` 和 `liaison` 的提供方日志，不要只看 Client 页面上的最后一条错误。
