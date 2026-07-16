# 图形客户端

Robonix Client 是运行在 Linux 或 macOS 上的浏览器客户端。它先连接机器人上的 Atlas，再发现 Liaison、Executor 和音频能力；用户不需要分别填写这些组件的地址。

## 1. 准备机器人端

客户端所在主机必须能访问机器人 Atlas 的监听地址。需要从外部主机连接时，机器人部署清单至少应让 Atlas、Liaison 和 Executor 监听可信局域网或 Tailscale 接口：

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
    config: {}
```

该软件包提供反向音频桥：客户端主动连接机器人，机器人清单不保存客户端 IP，也不需要在 Client 中猜测固定桥接端口。客户端通过 Atlas 的 `bridge_info` 能力查询 `audio_client_bridge` 公布的 WebSocket 端点，再用同一条连接传输麦克风和扬声器音频。

## 2. 安装

需要 Python 3.11 或更高版本。

Linux 先安装 PortAudio：

```bash
sudo apt update
sudo apt install -y libportaudio2 portaudio19-dev python3-venv
```

macOS 使用 Homebrew 安装 PortAudio：

```bash
brew install portaudio
```

随后在任一平台安装 Client：

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

### Chat

- **Conversation** 显示用户输入、系统状态、能力调用和回复。
- 输入框在空闲时提交新任务；任务执行中提交的内容作为 steer，交给 Pilot 调整当前任务。
- **Stop** 请求停止当前会话的任务和正在执行的动作。
- **Current Goal** 显示当前任务摘要，以及正在调用的提供者和接口。
- **RTDL Forest** 以 Executor 的状态为准显示正在运行的 RTDL；点击 **Active RTDL** 查看完整树和节点详情，点击 **Execution history** 查看已结束的执行记录。
- **Event Log** 显示当前会话收到的状态事件，便于判断任务处于规划、执行、等待还是结束阶段。

### Audio

Client 启动时会自动启动本机音频服务；如果 **Audio Device Server** 仍显示 offline，再点击 **Start Audio**。点击 **Refresh Route** 后，若语音输入和输出都使用客户端电脑的设备：

1. Input Primitive 选择 `audio_client_bridge`。
2. Output Primitive 选择 `audio_client_bridge`。
3. 选择本机麦克风和扬声器。
4. 点击 **Apply Route**。
5. 分别运行 **Test Microphone** 和 **Test Speaker**。

路由生效后，按 **F2** 开始一次语音输入；页面上的 **Voice** 按钮作用相同。免按键语音需要再显式开启 **Hands-free**。

### Settings

Settings 保存机器人地址、Atlas 端口、用户标识、录音时限和可选的 Liaison Endpoint。点击 **Save Settings** 后，设置会写入 `~/.config/robonix-client/settings.yaml`，页面也会在浏览器本地存储中保留当前值。正常部署应留空 **Liaison Endpoint**，让 Client 从 Atlas 发现它。

## 5. 最小验收

按顺序完成以下检查：

1. Connect 后状态为在线。
2. 发送“目前有哪些能力？”并收到文本回复；部署已接入相机和图像理解能力时，再发送“你能看到什么？”验证视觉链路。
3. 发送一个持续几秒的任务，在执行期间再提交一句修改要求，确认它作为 steer 生效。
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

确认 Atlas 中存在 `audio_client_bridge`，并且它注册了 `robonix/primitive/audio/bridge_info`。桥接端口由该能力公布，Client 会把其中的 loopback 主机名改写为当前 Robot Host 后主动连接；不要用猜测的固定端口排错。若 Client 已连接但设备列表为空，先检查操作系统是否允许当前终端访问麦克风，以及 PortAudio 是否能枚举设备。

### 文本可用但语音失败

Client 音频链路只负责采集和播放。语音识别、语音合成和唤醒词还要求机器人上的 Speech 服务正常启动，并且 Liaison 能发现所选的输入、输出与语音能力。分别查看 `audio_client_bridge`、`speech` 和 `liaison` 的 provider 日志，不要只看 Client 页面上的最后一条错误。
