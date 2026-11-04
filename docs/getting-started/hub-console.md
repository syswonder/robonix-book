---
title: Robonix Hub 与 Robonix Console 使用指南
description: 通过 Robonix Console 安装并运行 TIAGo Webots 仿真，连接 Robonix Client，并管理配置、历史与备份。
toc_max_heading_level: 3
---

import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './hub-console.module.css';

<div className={styles.guide}>

# Robonix Hub 与 Robonix Console 使用指南

本页面向首次使用 Robonix Hub、Robonix Console 和 Robonix Client 的使用者。准备一台运行 Ubuntu 22.04 的 Linux 电脑和可用的模型服务，即可从安装开始，完成“选择整机配置 → 启动仿真与 Robonix → 连接 → 执行任务”。

以下以 **PAL Robotics TIAGo 的 Webots 仿真**为例，全程通过 Robonix Console 管理部署与启停。完成后，Robonix Client 应显示已连接，并能提交任务、查看执行记录和相机画面。

| 产品 | 用途 |
|---|---|
| [Robonix Hub](https://hub.robonix.ai/) | 账户、软件包目录、配置备份与分享 |
| Robonix Console | 安装在仿真电脑上，管理配置、启动、停止、更新与日志 |
| [Robonix Client](https://github.com/syswonder/robonix-client) | 安装在操作者电脑或 Android 手机，与机器人交互并查看执行状态 |

首次使用按[仿真流程](#没有实体机器人运行仿真)操作；后续修改配置、恢复历史和备份，查阅[日常管理](#日常管理部署历史与云端备份)。

## 没有实体机器人：运行仿真

### 准备运行仿真的电脑

本示例采用 [PAL Robotics TIAGo 整机配置](https://hub.robonix.ai/package/robonix.robot.pal_robotics.tiago_webots)，准备以下条件：

- Ubuntu 22.04，架构为 x86_64 或 aarch64。
- 已安装 Docker；运行安装器的普通用户无需 `sudo` 即可运行 Docker，因为仿真由该用户启动。
- 可访问 Robonix Hub、GitHub 和容器镜像源的网络；首次构建期间保持网络畅通。
- Robonix Hub 账户，以及 OpenAI 兼容的模型服务地址、访问密钥和模型名。
- 操作者电脑或手机能够通过局域网或 VPN 访问仿真电脑。

在仿真电脑终端执行：

```bash
curl --version
docker ps
docker compose version
```

各命令应正常退出。`docker ps` 报权限错误时，先处理当前用户的 Docker 权限，再继续。Robonix Console 与 Robonix Client 可以安装在同一台电脑上。

### 安装 Robonix Console

#### 准备 Robonix Hub 账户

[准备 Robonix Hub 账户](https://hub.robonix.ai/auth/login)。打开账户页面后，没有账户时点底部“注册”，按页面提示完成注册与协议确认；已有账户可直接登录。收到邀请时也可使用邀请链接完成激活。开启两步验证时准备验证码。

#### 执行安装命令

在仿真电脑终端，以平时使用电脑的普通用户执行：

```bash
curl -fsSL https://hub.robonix.ai/install | bash
```

不要使用 root，也不要在命令前加 `sudo`。安装器会检查主机、准备依赖、校验发布文件并显示服务条款和隐私政策；阅读后按提示接受。补齐系统依赖时，可能单独询问 sudo 密码。

#### 等待安装完成

等待 Robonix Console 安装。已有 `rbnx` 时安装器使用现有安装；没有时会下载并构建 Robonix。首次构建可能较久。若安装器报告 Robonix 构建失败，先处理报错；Robonix Console 页面能打开不代表 Robonix 已安装成功。

<figure>
  <img src={useBaseUrl('/img/hub/console-install-progress.png')} alt="Robonix Console 安装器显示检查、下载和服务配置进度" width="1251" height="1026" loading="lazy" />
  <figcaption>等待终端给出安装结果和管理页面地址；失败时保留报错和构建日志。</figcaption>
</figure>

#### 登录 Robonix Console

安装结束后，在同一台电脑浏览器打开 [http://127.0.0.1:4880](http://127.0.0.1:4880)。填写 Robonix Hub 邮箱和密码，点“登录”；有两步验证时继续输入验证码。若提示尚未同意协议，先在 Robonix Hub 完成确认，再返回登录。

从另一台电脑访问时，在操作者电脑终端执行以下转发命令；`ROBOT_USER` 是仿真电脑上的 Linux 用户名，`ROBOT_IP` 是该电脑可达的局域网或 VPN 地址：

```bash
ssh -N -L 4880:127.0.0.1:4880 ROBOT_USER@ROBOT_IP
```

保持终端打开，在操作者电脑浏览器访问同一个本地地址。此隧道只用于访问 Robonix Console；Robonix Client 还需能访问机器人组件。

<figure className={styles.compactFigure}>
  <img src={useBaseUrl('/img/hub/hub-account-login-register.png')} alt="Robonix Hub 账户页面中的邮箱密码登录与注册入口" width="453" height="484" loading="lazy" />
  <figcaption>图为 Robonix Hub 账户登录与注册入口；Robonix Console 使用同一 Robonix Hub 账户。完成 Robonix Console 登录后，应能进入概览和配置页面。</figcaption>
</figure>

#### 设置模型服务

登录 Robonix Console 后，打开“设置 → 机器人环境变量”，填写模型服务的三个字段。密钥输入时不会直接显示；不要把密钥写进公开仓库或截图。

| 字段 | 变量 | 填写内容 |
|---|---|---|
| VLM 终端 URL | `VLM_BASE_URL` | 模型服务的 Chat Completions 兼容接口地址 |
| VLM KEY | `VLM_API_KEY` | 该服务的访问密钥 |
| VLM 模型 | `VLM_MODEL` | 用于 Pilot 任务规划的模型名 |

这三项用于任务规划组件 Pilot。填写或核对后保存。安装时已经填写过的值也应在此检查，确认地址、密钥和模型名属于同一服务。

<figure>
  <img src={useBaseUrl('/img/hub/console-model-settings.png')} alt="Robonix Console 设置页中的模型地址、密钥和模型名" width="1736" height="739" loading="lazy" />
  <figcaption>在“设置 → 机器人环境变量”填写①中的三项，模型地址与模型名以所用服务为准。</figcaption>
</figure>

### 安装 TIAGo 配置并设置启动钩子

#### 选择整机配置

在 Robonix Console 的“概览”点“选择机器人配置”（**Choose a robot configuration**），选择 **PAL Robotics TIAGo**，核对包名为 `robonix.robot.pal_robotics.tiago_webots`，再点“创建部署”（**Create deployment**）。

安装完成后，核对概览顶部的当前部署。后续启动钩子、配置修改与运行均作用于该部署。

#### 设置仿真启动与停止命令

在同一概览页打开“启动钩子”（**Boot hooks**），填写以下两项后点“保存”（**Save**）：

| 字段 | 命令 |
|---|---|
| 启动前（Before boot） | `bash sim/start.sh --detach` |
| 退出后（After exit） | `bash sim/stop.sh` |

命令中的路径相对于当前部署目录解析。Robonix Console 在启动 Robonix 前运行第一项，在运行退出后运行第二项；无需另开终端启动或停止 Webots。

#### 允许 Robonix Client 连接

在概览页点“允许其他机器连接”（**Allow other machines**）并确认，使操作者电脑或手机可以连接。此操作会修改相应组件的监听地址并保存为新版本，在下次启动时生效；仅在可信网络中使用。

### 启动并检查运行

在概览点“启动”（**Start**）。首次启动会构建仿真环境，需要等待数分钟，实际耗时取决于网络、硬件与缓存。

点“查看运行日志”（**View run logs**）观察进度；启动后再查看 `rbnx 输出` 和具体组件日志，确认启动过程与组件注册情况。出现失败或长时间等待注册时，先检查本次日志中的具体报错，再重试。

概览中的“连接 robonix-client”区域列出 **Robot Host** 和 **Atlas Port**。记录可从操作者设备访问的地址与端口，供下一步连接使用；不能只凭“运行中”判断所有功能均已就绪。

### 安装 Robonix Client 并执行首个任务

#### 在操作者电脑安装

先在操作者电脑安装 [uv](https://docs.astral.sh/uv/getting-started/installation/)，用于安装 Robonix Client 和所需 Python。Linux 或 macOS 在终端执行：

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Windows 在 PowerShell 执行：

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

打开新终端，执行：

```bash
uv tool install --python 3.11 git+https://github.com/syswonder/robonix-client.git
```

`ROBOT_IP` 表示概览页列出的、操作者电脑可达的仿真电脑地址；两者同机时可用 `127.0.0.1`。替换后执行：

```bash
robonix-client --robot-host ROBOT_IP
```

在操作者电脑浏览器打开 [http://127.0.0.1:7860](http://127.0.0.1:7860)。核对 **Robot Host** 与概览显示的 **Atlas Port**，**Liaison Endpoint** 留空，点 **Connect**，等待显示 **Connected**。

Android 8.0 或更高版本也可从 [Robonix Client Android 发布页](https://github.com/syswonder/robonix-client-android/releases/latest)安装 APK，在 **Settings** 填写 **Robot Host**，点 **Save** 并连接。电脑和手机均须能直接访问仿真电脑的相关组件；仅建立 Robonix Console 的 SSH 转发不足以连接 Robonix Client。

#### 提交任务并核对结果

连接成功后，在 **Chat** 输入并发送：

```text
向左转 90 度，拍一张照片，并描述看到的内容。
```

检查 TIAGo 的转向、相机画面与回复是否一致，并在 RTDL 执行树和执行记录中查看导航、拍照调用。机器人任务描述语言（Robot Task Description Language，RTDL）用于描述任务执行过程；执行状态的读取方式见[图形客户端指南](client.md)。只收到规划回复不代表动作已经完成。

相机与执行记录的位置如下图；完整交互、音频与状态面板操作见[图形客户端指南](client.md)。

<figure>
  <img src={useBaseUrl('/img/ui/cl-chat-live.webp')} alt="Robonix Client 的对话与任务执行面板" width="1400" height="862" loading="lazy" />
  <figcaption>对照相机画面、当前目标与执行记录，检查任务结果。</figcaption>
</figure>

### 停止运行

在 Robonix Console 概览点“停止”。运行退出后，配置的 **After exit** 钩子执行 `bash sim/stop.sh` 停止 Webots；检查停止日志，确认本次运行已结束，再切换或启动其他部署。

## 运行之后：查阅功能指南

仿真连接与首个任务验证完成后，按需求阅读相应章节。本页不重复服务操作步骤，参数与功能说明以对应章节为准。

| 需求 | 对应指南 |
|---|---|
| 创建地图、保存地图与定位 | [建图与定位](../components/mapping.md) |
| 设置导航输入、目标与速度 | [导航服务](../components/navigation.md) |
| 查看相机、标注房间与检查场景 | [场景服务](../components/scene.md) |
| 查看任务执行树、相机与状态 | [图形客户端](client.md) |

## 日常管理：部署、历史与云端备份

在 Robonix Console 中管理当前部署的配置。每次操作先核对**当前部署**，再区分“配置清单”“软件包代码”和“地图等本机资源”；它们的保存与恢复范围不同。

### 修改配置并保存

打开“配置”，展开需要修改的系统组件、原语（Primitive）、服务（Service）或技能（Skill），按参数说明编辑；需要指定提供方时，选择当前部署中的实际条目名称。首次运行保留整机已有的组合，不随意关闭其他组件依赖的条目。

<figure>
  <img src={useBaseUrl('/img/hub/console-parameters.png')} alt="Robonix Console 中展开软件包并修改参数" width="1240" height="691" loading="lazy" />
  <figcaption>展开条目后编辑参数；长说明可点“更多”查看。</figcaption>
</figure>

模型地址和密钥等本机共用值放在“设置 → 机器人环境变量”；“配置”中的环境变量只对当前部署生效，同名时以部署为准。密钥不写进清单。

需要添加功能时，在对应分组点“添加软件包”，先核对 [Robonix Hub 目录](https://hub.robonix.ai/discover)中的能力与依赖，再添加、设置参数。已有条目不能重复添加；本机包需在部署目录中有 `package_manifest.yaml`。

修改后点“保存为新版本”，填写修改说明，核对差异后保存。新增或更换软件包需重新构建；启动时读取的参数需重启后生效。

<figure>
  <img src={useBaseUrl('/img/hub/console-save-version.png')} alt="保存配置新版本时的修改说明和差异" width="2048" height="832" loading="lazy" />
  <figcaption>先核对差异再保存，随后在“历史”中确认新记录。</figcaption>
</figure>

### 管理多套部署

“设置”的部署列表显示名称和路径，“概览”上方可切换当前部署。一套部署对应一个目录及其独立历史，可分别保存不同实验配置。

| 操作 | 结果与检查 |
|---|---|
| 新建空白部署 | 只填写名称，Robonix Console 创建空目录；随后在“配置”添加软件包并保存第一个版本 |
| 登记已有目录 | 新建时填写已有目录的路径，Robonix Console 使用该目录；先检查清单和附带文件 |
| 安装整机配置 | 当前部署为空则放入其中；已有配置时新建并切换，保留原部署 |
| 切换部署 | 只改变当前操作对象，不自动停止原来的运行；先停止原运行再启动另一套 |
| 重命名或删除 | 在“设置”操作；Robonix Console 创建的目录删除时连同文件删除，登记的外部目录只移除记录 |

最后一套部署不能删除。同一主机不要同时启动多套 Robonix；先在概览停止当前运行，再操作其他部署。

默认部署通常在 `~/.local/share/robonix-console/deployments/default`，其他部署以“设置”中的实际路径为准。清单历史不包含目录中的全部文件，删除前另外备份本机资源。

### 查看与恢复配置历史

打开“历史”，查看最新记录和“当前”标记。选择旧版本，点“与当前对比”，确认差异。

<figure>
  <img src={useBaseUrl('/img/hub/console-history.png')} alt="配置历史中的版本与对比入口" width="1400" height="680" loading="lazy" />
  <figcaption>先对比，再决定是否回滚；核对当前部署名称。</figcaption>
</figure>

需要恢复时，选择目标版本，点“回滚到此版本”并确认，回到“配置”检查结果，再按原启动方式重启。回滚本身不新增版本，也不会还原软件包代码、地图或标定文件。

“导出历史”保存当前部署的全部配置版本；“导入历史”会替换当前部署已有历史，导入前先导出留底。直接在本机修改过清单时，Robonix Console 会在下一次保存或回滚前先记下该改动。

### 备份、恢复与分享配置

配置备份包含**部署清单与配置历史**，最多保留最新 200 个版本；不包含本机软件包、`.env`、地图、模型与标定资源。恢复一份清单不等于还原了一台可运行的机器人。

#### 上传当前部署

在“概览 → 云端备份”核对部署名称，点“上传此部署”，检查将发送的主机、系统、版本信息和密钥替换提示，确认后上传。在 [Robonix Hub 我的机器人](https://hub.robonix.ai/robots)查看备份。

<figure>
  <img src={useBaseUrl('/img/hub/console-cloud-backup.png')} alt="Robonix Console 中的云端上传与下载入口" width="1568" height="680" loading="lazy" />
  <figcaption>云端备份卡片作用于当前部署；上传前检查名称和内容。</figcaption>
</figure>

各部署分别备份，同一部署再次上传会替换自己的上一份配置备份。换用其他账户后，备份属于该账户。

#### 在另一台机器恢复

在目标机器安装 Robonix Console，登录同一 Robonix Hub 账户，点“从云端下载”并选择“机器人・部署”。**默认创建新部署，当前部署保持不变**；只有显式勾选覆盖，才替换当前部署的清单和历史。

下载后核对硬件、设备地址和路径，补齐软件包、本机凭据、地图与资源，再构建、启动并验证。仿真配置不能直接替代实体机器人的适配配置。

#### 分享配置

在“我的机器人”中点已上传备份旁的“分享”，检查密钥替换方式后确认并复制链接。分享的是密钥替换为环境变量引用的备份，自己的记录不受影响；点“停止分享”后链接立即失效。

接收者无需登录即可查看分享页。安装时在接收方的 Robonix Console“云端备份”卡片点“导入分享链接”，粘贴、打开并安装；空部署直接使用，已有配置时新建部署。分享链接不携带作者的本机文件，安装后仍需自行补齐。

[Robonix Hub 用量](https://hub.robonix.ai/usage)显示当前空间占用与额度。超限时先整理不需要的备份，额度以页面为准。

### 使用 AI 配置助手

Robonix Console 右侧的“AI 机器人配置助手”用于读配置、查软件包、改参数和读日志；给机器人发任务使用 Robonix Client。助手使用“设置”中的模型三项，模型接口需兼容 Chat Completions 并支持工具调用。

可以先输入“检查当前配置有没有问题”。修改配置、回滚、启动停止、上传云端等改变状态的操作会先展示确认卡片；核对对象和差异后点“同意”，不合适则点“拒绝”。

<figure>
  <img src={useBaseUrl('/img/hub/console-assistant.png')} alt="AI 配置助手读取配置并请求确认修改" width="2352" height="1800" loading="lazy" />
  <figcaption>从读取到修改分步检查，不跳过确认卡片中的差异。</figcaption>
</figure>

配置修改会保存为带“(AI)”说明的新版本，可在历史中恢复。设置本机环境变量时，在专用输入框中填写值，不将密钥发到聊天正文。执行机器人任务仍需通过 Robonix Client。

### 更新程序与软件包

| 更新对象 | 操作与验证 |
|---|---|
| Robonix Console | “更新”页检查版本；自动更新保留部署、历史与日志，不更新机器人软件包 |
| Robonix | 在“更新”选择发布版本或提交 ID，按页面执行升级；流程会停止 Robonix、切换源码、编译并重建软件包，完成后重新验证 |
| 软件包 | 在“更新”页核对当前部署、来源及可更新版本；更新远程软件包后检查构建结果，随后启动并验证 |

重新构建不等于更新代码。本机目录包需另外更新源文件；固定版本或提交的包，需先明确要使用的版本，再更新配置和构建。

Robonix Console 在启动时和此后每 6 小时检查更新；协议变化、构建或升级任务进行中、关闭自动更新时，需按页面提示处理。升级前保留可用配置，并先停止当前运行；配置回滚不能撤销代码升级。

“设置”还可调整日志保留数量、下载镜像、构建和运行前加载的环境脚本、本机变量与访问账户。默认保留最近 10 次 Robonix Console 运行日志。需要访问页面时优先使用 SSH 隧道；Robonix Console 页面访问权限与 Robonix Client 组件连接是两回事。

<details>
<summary>重新安装或卸载 Robonix Console</summary>

再次执行安装命令即可重新安装，部署与历史会保留。卸载前先结束任务，在概览停止 Robonix 并检查退出钩子的执行结果。在安装 Robonix Console 的普通用户账户下执行：

```bash
curl -fsSL https://hub.robonix.ai/install | bash -s -- --uninstall
```

卸载移除 Robonix Console 服务并停止其管理的运行和后台任务，保留部署、历史与日志。后续清理目录前，另行备份需要保留的资源。

</details>

## 遇到问题：排查与反馈

先保留报错，从最后一个正常步骤继续排查，避免通过反复重装跳过具体错误。

### 按现象定位

| 现象 | 优先检查 |
|---|---|
| 无法登录 Robonix Hub 或 Robonix Console | 账户激活、协议确认、密码和二次验证；有网络异常时查看 [Robonix Hub 服务状态](https://hub.robonix.ai/status) |
| Robonix Console 页面打不开 | 安装结果、本机 `127.0.0.1:4880`；远程使用时检查 SSH 转发 |
| 首次启动停在仿真准备阶段 | 当前用户运行 `docker ps` 是否正常，网络是否可下载镜像，Before boot 命令是否保存 |
| 停止后仿真仍在运行 | After exit 命令是否为 `bash sim/stop.sh`，退出钩子的日志是否报错 |
| Robonix Client 离线 | Robot Host 和 Atlas Port 是否正确，是否允许其他机器连接，以及运行和组件是否就绪 |
| 模型请求失败 | 模型地址、密钥、模型名与网络，重启后是否使用最新设置 |
| 参数修改未生效 | 当前部署是否正确，修改是否保存，以及是否需要重新构建或启动 |
| 地图、导航或场景异常 | 参阅[对应功能指南](#运行之后查阅功能指南)，检查其依赖和组件日志 |
| 恢复备份后无法运行 | 软件包、本机凭据、地图和资源是否补齐 |

### 找到对应日志

下载和构建任务看 Robonix Console“更新”页的任务日志；运行看“日志”，选择本次记录，再看 `rbnx 输出` 和具体组件标签。用筛选定位错误，必要时显示全部或下载完整日志。启动与退出钩子执行失败时，保留相应输出，不只提供“启动失败”四个字。

### 提交反馈

1. 点 Robonix Console 或 Robonix Hub 侧栏底部的反馈按钮；手机从右上角进入。
2. 选择反馈对象，写清复现步骤、期望结果、实际现象和完整报错。
3. 点“查看”检查随附信息，保留相关日志；按需勾选当前部署清单。
4. 确认内容后发送，检查发送成功提示。

<figure>
  <img src={useBaseUrl('/img/hub/console-feedback.png')} alt="反馈表单中的问题描述、附件预览和发送入口" width="1680" height="1232" loading="lazy" />
  <figcaption>发送前查看附件，核对本次问题所需日志，避免在描述中直接粘贴密钥。</figcaption>
</figure>

Robonix Console 反馈可附系统与硬件、版本、最近日志和管理程序设置；当前部署清单默认不勾选，选中后其中的密钥会替换为占位符。Robonix Hub 网页反馈附浏览器信息。检查附件是否包含本次运行日志，缺失时另外整理提供。

一条反馈描述一个问题，注明整机配置、软件版本和发生时间。Robonix Console 无法连接 Robonix Hub 时，可“下载反馈包”交给维护者。需要恢复配置时，先导出历史留底，再对比、回滚并重启；本机资源与代码另行检查。

## 了解相关概念

部署、能力和组件的定义分别见[部署与启动](../architecture/deployment-and-startup.md)、[命名空间与能力约定](../architecture/namespace-and-contracts.md)和[系统组件](../architecture/components.md)。需要查看 `rbnx` 命令时，参阅[命令行工具](../components/rbnx.md)。

</div>
