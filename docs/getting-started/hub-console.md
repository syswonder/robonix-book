---
title: Robonix Console 使用指南
description: 安装与登录 Robonix Console，配置并运行 TIAGo Webots，连接 Robonix Client，管理部署、配置历史与备份。
toc_max_heading_level: 3
---

import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './hub-console.module.css';

<div className={styles.guide}>

# Robonix Console 使用指南

Robonix Console 安装在运行 Robonix 的机器上，通过浏览器管理这台机器的配置、运行、日志和更新。软件包目录提供整机配置、原语、服务和技能；配置修改可以保存为版本，随后对比、回滚、备份或分享。

本页以 **Webots 中的 PAL Robotics TIAGo** 为例，按安装、登录、配置机器人和运行的顺序介绍操作。完成后，应能从 Robonix Client 连接机器人、发送任务并查看执行结果。

## 组成

| 产品 | 位置 | 用途 |
|---|---|---|
| Robonix Console | 运行 Robonix 的机器，默认端口 4880 | 管理所在机器的配置、运行、日志和更新 |
| [Robonix Hub](https://hub.robonix.ai/) | hub.robonix.ai | 账户、软件包目录、配置备份与分享 |
| [Robonix Client](https://github.com/syswonder/robonix-client) | 操作者电脑或 Android 手机 | 发送任务，查看执行状态和相机画面 |

Robonix Console 使用 Robonix Hub 账户登录。Robonix Hub 的目录和账户页面不能直接修改机器上的配置；配置修改在 Robonix Console 中进行。

## 使用流程

首次使用按以下顺序操作：

1. 激活 Robonix Hub 账户，准备运行环境。
2. 安装 Robonix Console，登录后检查模型服务。
3. 安装 TIAGo 整机配置，按需修改参数并保存。
4. 下载和构建软件包，设置 Webots 启动钩子，再启动 Robonix。
5. 查看日志，连接 Robonix Client 并发送任务。
6. 对比和恢复配置版本，备份或分享部署，按需使用 AI 机器人配置助手。

## 激活账户

准备可登录的 Robonix Hub 账户。已有账户时，可在 [Robonix Hub 登录页面](https://hub.robonix.ai/auth/login)登录。通过邀请开通账户时，按以下步骤激活：

1. 打开邮件中的邀请链接，输入验证码。
2. 阅读个人信息出境告知和许可协议，按页面提示确认。
3. 设置密码；需要二次验证时，在账户设置中绑定身份验证器。

协议更新后，先在 Robonix Hub 完成确认，再登录 Robonix Console。

## 安装

### 准备运行环境

本例在 Linux 电脑上运行 TIAGo Webots 仿真，准备以下条件：

- Ubuntu 22.04，x86_64 或 aarch64。
- 已安装 Docker，执行安装器的普通用户可以不加 <code>sudo</code> 运行 Docker。
- 可访问 Robonix Hub、GitHub 和容器镜像源的网络。
- 已激活的 Robonix Hub 账户，以及模型服务地址、访问密钥和模型名。
- 操作者设备能够通过局域网或 VPN 访问运行 Robonix 的电脑。

在运行仿真的电脑终端检查：

```bash
curl --version
docker ps
docker compose version
```

各命令应正常退出。<code>docker ps</code> 报权限错误时，先处理当前用户的 Docker 权限。Robonix Console 与 Robonix Client 可以安装在同一台电脑上。

### 安装命令

在仿真电脑终端，以平时使用电脑的普通用户执行：

```bash
curl -fsSL https://hub.robonix.ai/install | bash
```

不要使用 root，也不要在命令前加 `sudo`。安装器会检查主机、准备依赖、校验发布文件并显示服务条款和隐私政策；阅读后按提示接受。补齐系统依赖时，可能单独询问 sudo 密码。安装器询问模型服务时，可以填写，也可以选择跳过，登录后再按下文设置。

### 安装过程

等待 Robonix Console 安装。已有 `rbnx` 时安装器使用现有安装；没有时会下载并构建 Robonix。首次构建可能较久。若安装器报告 Robonix 构建失败，先处理报错；Robonix Console 页面能打开不代表 Robonix 已安装成功。

<figure>
  <img src={useBaseUrl('/img/hub/manual/install.webp')} alt="安装进度" width="1251" height="1026" loading="lazy" />
  <figcaption>安装器显示检查、下载和服务配置进度；等待终端给出安装结果与访问地址。</figcaption>
</figure>

## 登录

### 打开管理页面

安装结束后，在同一台电脑浏览器打开 [http://127.0.0.1:4880](http://127.0.0.1:4880)。填写 Robonix Hub 邮箱和密码，点`登录`；有两步验证时继续输入验证码。若提示尚未同意协议，先在 Robonix Hub 完成确认，再返回登录。

从另一台电脑访问时，在操作者电脑终端执行以下转发命令；`ROBOT_USER` 是仿真电脑上的 Linux 用户名，`ROBOT_IP` 是该电脑可达的局域网或 VPN 地址：

```bash
ssh -N -L 4880:127.0.0.1:4880 ROBOT_USER@ROBOT_IP
```

保持终端打开，在操作者电脑浏览器访问同一个本地地址。此隧道只用于访问 Robonix Console；Robonix Client 还需能访问机器人组件。

<figure>
  <img src={useBaseUrl('/img/hub/manual/login.webp')} alt="邮箱、密码和登录按钮" width="1400" height="680" loading="lazy" />
  <figcaption>填写邮箱①和密码②，点“登录”③；已启用二次验证时继续输入验证码。</figcaption>
</figure>

### 页面布局

登录后，侧栏中的概览、配置、历史、日志、更新和设置用于管理所在机器；目录、我的机器人、个人主页和用量显示 Robonix Hub 的内容。侧栏底部提供账户菜单和反馈入口，右侧提供 AI 机器人配置助手。手机通过顶部菜单和底部标签切换页面。

### 设置模型服务

登录 Robonix Console 后，打开`设置 → 机器人环境变量`，填写模型服务的三个字段。密钥输入时不会直接显示；不要把密钥写进公开仓库或截图。

| 字段 | 变量 | 填写内容 |
|---|---|---|
| VLM 终端 URL | `VLM_BASE_URL` | 模型服务的 Chat Completions 兼容接口地址 |
| VLM KEY | `VLM_API_KEY` | 该服务的访问密钥 |
| VLM 模型 | `VLM_MODEL` | 用于 Pilot 任务规划的模型名 |

这三项用于任务规划组件 Pilot。填写或核对后保存。安装时已经填写过的值也应在此检查，确认地址、密钥和模型名属于同一服务。这些值保存在仿真电脑上，不随配置备份上传。运行期间修改后，重新启动 Robonix 才会读取新值。

<figure>
  <img src={useBaseUrl('/img/hub/manual/variables.webp')} alt="设置页中的模型服务与本机环境变量" width="1240" height="995" loading="lazy" />
  <figcaption>模型服务包括地址、密钥和模型名三项。①标出环境变量区域，②用于添加其他变量。</figcaption>
</figure>

## 配置机器人

一套部署的配置写在部署清单 <code>robonix_manifest.yaml</code> 中，包括原语（Primitive）、服务（Service）、技能（Skill）和系统组件的参数。部署清单与配置历史保存在部署目录中。

### 从目录安装整机配置

在 Robonix Console 的 <code>概览</code> 点 <code>选择机器人配置</code>（Choose a robot configuration），选择 **PAL Robotics TIAGo**，核对包名为 <code>robonix.robot.pal_robotics.tiago_webots</code>，再点 <code>创建部署</code>（Create deployment）。也可从 <code>目录</code> 的机器人分类打开同一整机配置，查看说明并安装。

<figure>
  <img src={useBaseUrl('/img/hub/manual/catalog.webp')} alt="检索 TIAGo 整机配置" width="1498" height="680" loading="lazy" />
  <figcaption>选择“机器人”①，搜索 tiago ②，打开 TIAGo 条目③。</figcaption>
</figure>

<figure>
  <img src={useBaseUrl('/img/hub/manual/robot-package.webp')} alt="TIAGo 的整机配置页" width="1400" height="680" loading="lazy" />
  <figcaption>整机页显示说明和仓库；①为安装入口。</figcaption>
</figure>

整机配置包含部署清单以及附带的本机软件包、配置和仿真脚本。当前部署为空时使用当前目录；已有配置时会新建部署并切换，原部署保留。安装后先核对当前部署，再修改配置或启动。

### 修改参数

打开 <code>配置</code>，展开需要修改的条目。提供 <code>config.spec</code> 的软件包显示分组表单；点参数行查看说明并编辑，长说明可点 <code>更多</code> 展开，<code>改回默认</code> 清除该项设置。

<figure>
  <img src={useBaseUrl('/img/hub/manual/parameters.webp')} alt="分组参数表单" width="1240" height="691" loading="lazy" />
  <figcaption>展开参数分组①，选择参数行②；“更多”③显示完整说明。</figcaption>
</figure>

参数需要指定其他条目作为提供方时，从列表中选择当前部署里的实际条目名称。提供方在其他机器上时，按参数说明选择 <code>手动填写</code> 并输入名称。

<figure>
  <img src={useBaseUrl('/img/hub/manual/provider.webp')} alt="能力提供方选择框" width="1240" height="680" loading="lazy" />
  <figcaption>展开提供方参数①，从下拉框②选择条目；示例中为 tiago_lidar。</figcaption>
</figure>

没有严格格式说明的软件包按键值对编辑；<code>查看说明</code> 显示附带说明。首次运行保留整机已有组合，不随意关闭其他组件依赖的条目。

### 本机环境变量

部署引用了尚未设置的必填变量时，概览或配置页会提示缺失项，可点 <code>设置</code> 填写。值保存在本机，所有部署共用；部署自身的同名环境变量优先。密钥使用环境变量引用，不直接写入公开清单。

### 添加软件包

在原语、服务或技能分组点 <code>添加软件包</code>，从目录检索并选择条目；已在部署中的软件包不能重复加入。添加前先检查能力说明、依赖与所需参数。

<figure>
  <img src={useBaseUrl('/img/hub/manual/add-package.webp')} alt="技能检索与添加" width="1680" height="680" loading="lazy" />
  <figcaption>①为搜索框，②为搜索结果；图中 find_object 用于展示添加方式，首次运行无需额外添加。</figcaption>
</figure>

添加本机软件包时，选择 <code>本机目录</code>，从部署目录内包含 <code>package_manifest.yaml</code> 的子目录中选择。需要直接编辑清单时，可使用 <code>编辑 YAML</code>，完成后返回表单检查。

### 保存为新版本

点 <code>保存为新版本</code>，填写修改说明，检查差异后保存。新增或更换软件包后需下载并构建；启动时读取的参数在重启后生效。

<figure>
  <img src={useBaseUrl('/img/hub/manual/save-version.webp')} alt="版本说明和配置差异" width="2048" height="832" loading="lazy" />
  <figcaption>核对版本号①，填写修改说明②，检查配置差异③后保存。</figcaption>
</figure>

目录中没有合适配置时，可在 <code>设置</code> 新建空白部署，再逐项添加软件包并保存第一个版本。首次体验使用本例的 TIAGo 配置。

## 运行

### 下载与构建软件包

打开`更新`页，在软件包列表中检查当前部署引用的原语、服务和技能。随整机配置提供的条目来自本机目录；远程软件包需要下载后构建。

点`更新全部软件包`，等待页面上方的任务结束，期间保持网络畅通。查看任务日志与构建结果，确认没有失败的软件包，再继续启动。耗时取决于硬件、网络和已有缓存，不以等待固定时长作为成功标志。

构建失败时，保留日志中出错的软件包和原因，处理后重试；确认属于缓存损坏时，再使用`清理缓存`。

<figure>
  <img src={useBaseUrl('/img/hub/manual/packages.webp')} alt="更新页中的软件包列表与更新按钮" width="1568" height="1050" loading="lazy" />
  <figcaption>①为当前部署的软件包列表，②为“更新全部软件包”；任务结束后检查构建日志与结果。</figcaption>
</figure>

### Webots 仿真

回到概览页，打开`启动钩子`（**Boot hooks**），填写以下两项后点`保存`（**Save**）：

| 字段 | 命令 |
|---|---|
| 启动前（Before boot） | `bash sim/start.sh --detach` |
| 退出后（After exit） | `bash sim/stop.sh` |

命令中的路径相对于当前部署目录解析。Robonix Console 在启动 Robonix 前运行第一项，在运行退出后运行第二项；无需另开终端启动或停止 Webots。

### 允许其他机器连接

在概览页点`允许其他机器连接`（**Allow other machines**）并确认，使操作者电脑或手机可以连接。此操作会修改相应组件的监听地址并保存为新版本，在下次启动时生效；仅在可信网络中使用。

### 启动与停止

在概览点`启动`（**Start**）。首次启动会构建仿真环境，需要等待数分钟，实际耗时取决于网络、硬件与缓存。软件包构建和仿真环境构建是两个准备环节，均须完成。

点`查看运行日志`（**View run logs**）观察进度；启动后再查看 `rbnx 输出` 和具体组件日志，确认启动过程与组件注册情况。出现失败或长时间等待注册时，先检查本次日志中的具体报错，再重试。

概览中的`连接 robonix-client`区域列出 **Robot Host** 和 **Atlas Port**。记录可从操作者设备访问的地址与端口，供下一步连接使用；不能只凭`运行中`判断所有功能均已就绪。

<figure>
  <img src={useBaseUrl('/img/hub/manual/running.webp')} alt="运行状态与客户端连接信息" width="2048" height="864" loading="lazy" />
  <figcaption>①为运行状态，②为停止按钮，③和④为客户端所需的 Robot Host 与 Atlas Port。</figcaption>
</figure>

在 Robonix Console 概览点`停止`。运行退出后，配置的 **After exit** 钩子执行 `bash sim/stop.sh` 停止 Webots；检查停止日志，确认本次运行已结束，再切换或启动其他部署。

同一主机同一时间只运行一套 Robonix，避免组件端口冲突。切换部署前先停止当前运行；开机自启也只为一套部署启用。

### 查看日志

点 <code>查看运行日志</code> 或打开 <code>日志</code>，选择本次运行，再查看 <code>rbnx 输出</code> 和组件标签。筛选框按关键字过滤，<code>跟随最新</code> 用于持续观察；日志较长时可显示全部或下载完整日志。

<figure>
  <img src={useBaseUrl('/img/hub/manual/logs.webp')} alt="运行记录和组件日志" width="1458" height="680" loading="lazy" />
  <figcaption>选择运行记录①，查看 rbnx 输出②；③筛选关键字，④切换“跟随最新”。</figcaption>
</figure>

### 连接 Robonix Client

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

<figure>
  <img src={useBaseUrl('/img/hub/manual/client.webp')} alt="客户端连接界面" width="2048" height="680" loading="lazy" />
  <figcaption>填写机器人主机①和 Atlas 端口②；连接成功后检查“已连接”③。</figcaption>
</figure>

### 发送任务

连接成功后，输入并发送：

```text
向左转 90 度，拍一张照片，并描述看到的内容。
```

检查 TIAGo 的转向、相机画面与回复是否一致，并查看任务执行记录。机器人任务描述语言（Robot Task Description Language，RTDL）的执行树显示能力调用与执行状态；只收到规划回复不代表动作已完成。

<figure>
  <img src={useBaseUrl('/img/hub/manual/task.webp')} alt="拍照与场景描述任务结果" width="1960" height="1092" loading="lazy" />
  <figcaption>拍照任务示例：①为回复内容，②为已完成的当前目标。</figcaption>
</figure>

## 版本管理

### 版本与对比

打开 `历史`，查看最新记录和`当前`标记。选择旧版本，点`与当前对比`，确认差异。

<figure>
  <img src={useBaseUrl('/img/hub/manual/history.webp')} alt="配置版本列表与对比入口" width="1400" height="680" loading="lazy" />
  <figcaption>①标记当前版本；选择旧版本②，点“与当前对比”③查看差异。</figcaption>
</figure>

### 回滚

需要恢复时，选择目标版本，点`回滚到此版本`并确认，回到`配置`检查结果，再按原启动方式重启。回滚本身不新增版本，也不会还原软件包代码、地图或标定文件。

### 导出与导入

`导出历史`保存当前部署的全部配置版本；`导入历史`会替换当前部署已有历史，导入前先导出留底。直接在本机修改过清单时，Robonix Console 会在下一次保存或回滚前先记下该改动。

## 部署目录

`设置`的部署列表显示名称和路径，`概览`上方可切换当前部署。一套部署对应一个目录及其独立历史，可分别保存不同实验配置。

| 操作 | 结果与检查 |
|---|---|
| 新建空白部署 | 只填写名称，Robonix Console 创建空目录；随后在`配置`添加软件包并保存第一个版本 |
| 登记已有目录 | 新建时填写已有目录的路径，Robonix Console 使用该目录；先检查清单和附带文件 |
| 安装整机配置 | 当前部署为空则放入其中；已有配置时新建并切换，保留原部署 |
| 切换部署 | 只改变当前操作对象，不自动停止原来的运行；先停止原运行再启动另一套 |
| 重命名或删除 | 在`设置`操作；Robonix Console 创建的目录删除时连同文件删除，登记的外部目录只移除记录 |

最后一套部署不能删除。同一主机不要同时启动多套 Robonix；先在概览停止当前运行，再操作其他部署。

默认部署通常在 `~/.local/share/robonix-console/deployments/default`，其他部署以`设置`中的实际路径为准。清单历史不包含目录中的全部文件，删除前另外备份本机资源。

## 备份与分享

配置备份包含**部署清单与配置历史**，最多保留最新 200 个版本；不包含本机软件包、`.env`、地图、模型与标定资源。恢复一份清单不等于还原了一台可运行的机器人。

<figure>
  <img src={useBaseUrl('/img/hub/manual/backup.webp')} alt="部署备份卡片" width="1568" height="680" loading="lazy" />
  <figcaption>先核对当前部署；①上传配置与历史，②选择要下载的备份。</figcaption>
</figure>

### 上传当前部署

在 `概览 → 云端备份`核对部署名称，点`上传此部署`，检查将发送的主机、系统、版本信息和密钥替换提示，确认后上传。在 [Robonix Hub 我的机器人](https://hub.robonix.ai/robots)查看备份。

各部署分别备份，同一部署再次上传会替换自己的上一份配置备份。换用其他账户后，备份属于该账户。

### 在另一台机器恢复

在目标机器安装 Robonix Console，登录同一 Robonix Hub 账户，点`从云端下载`并选择`机器人・部署`。**默认创建新部署，当前部署保持不变**；只有显式勾选覆盖，才替换当前部署的清单和历史。

下载后核对硬件、设备地址和路径，补齐软件包、本机凭据、地图与资源，再构建、启动并验证。仿真配置不能直接替代实体机器人的适配配置。

### 分享配置

在`我的机器人`中点已上传备份旁的`分享`，检查密钥替换方式后确认并复制链接。分享的是密钥替换为环境变量引用的备份，自己的记录不受影响；点`停止分享`后链接立即失效。

接收者无需登录即可查看分享页。安装时在接收方的 Robonix Console`云端备份`卡片点`导入分享链接`，粘贴、打开并安装；空部署直接使用，已有配置时新建部署。分享链接不携带作者的本机文件，安装后仍需自行补齐。

[Robonix Hub 用量](https://hub.robonix.ai/usage)显示当前空间占用与额度。超限时先整理不需要的备份，额度以页面为准。

## AI 机器人配置助手

Robonix Console 右侧的`AI 机器人配置助手`用于读配置、查软件包、改参数和读日志；给机器人发任务使用 Robonix Client。助手使用`设置`中的模型三项，模型接口需兼容 Chat Completions 并支持工具调用。

可以先输入`检查当前配置有没有问题`。修改配置、回滚、启动停止、上传云端等改变状态的操作会先展示确认卡片；核对对象和差异后点`同意`，不合适则点`拒绝`。

配置修改会保存为带`(AI)`说明的新版本，可在历史中恢复。设置本机环境变量时，在专用输入框中填写值，不将密钥发到聊天正文。执行机器人任务仍需通过 Robonix Client。

<figure>
  <img src={useBaseUrl('/img/hub/manual/assistant.webp')} alt="AI 配置助手确认卡片" width="768" height="920" loading="lazy" />
  <figcaption>修改前核对确认卡片中的操作与差异，点“同意”后才会执行。</figcaption>
</figure>

## 目录与收藏

打开 [Robonix Hub 目录](https://hub.robonix.ai/discover)，按类别或名称查找软件包，查看版本、说明、仓库及能力约定。整机配置页列出所用的软件包；能力约定的概念与接口定义见[命名空间与能力约定](../architecture/namespace-and-contracts.md)及 book 的接口目录。

目录内容可直接浏览；收藏和安装需要登录。点 <code>收藏</code> 后，可在收藏列表或个人主页查看，Robonix Hub 与 Robonix Console 使用同一账户的收藏记录。

## 更新与卸载

| 更新对象 | 操作与验证 |
|---|---|
| Robonix Console | `更新`页检查版本；自动更新保留部署、历史与日志，不更新机器人软件包 |
| Robonix | 在`更新`选择发布版本或提交 ID，按页面执行升级；流程会停止 Robonix、切换源码、编译安装并重建软件包，完成后重新验证 |
| 软件包 | 在`更新`页核对当前部署、来源及可更新版本；更新远程软件包后检查构建结果，随后启动并验证 |

升级任务在后台进行，关闭页面不会中断任务。升级失败时查看日志和页面报告的回退结果，再决定后续操作。

重新构建不等于更新代码。本机目录包需另外更新源文件；固定版本或提交的包，需先明确要使用的版本，再更新配置和构建。

Robonix Console 在启动时和此后每 6 小时检查更新；协议变化、构建或升级任务进行中、关闭自动更新时，需按页面提示处理。升级前保留可用配置，并先停止当前运行；配置回滚不能撤销代码升级。

### 重新安装与卸载

再次执行安装命令即可重新安装，部署与历史会保留。卸载前先结束任务，在概览停止 Robonix 并检查退出钩子的执行结果。在安装 Robonix Console 的普通用户账户下执行：

```bash
curl -fsSL https://hub.robonix.ai/install | bash -s -- --uninstall
```

卸载停止并移除 Robonix Console 服务，保留部署、历史与日志。后续清理目录前，另行备份需要保留的资源。

## 设置

在 `设置` 中可以调整日志保留数量、下载镜像、构建和运行前加载的环境脚本、本机变量与访问账户。默认保留最近 10 次 Robonix 运行的日志。需要访问页面时优先使用 SSH 隧道；Robonix Console 页面访问权限与 Robonix Client 组件连接是两回事。

## 反馈问题

### 提交反馈

1. 点 Robonix Console 或 Robonix Hub 侧栏底部的反馈按钮；手机从右上角进入。
2. 选择反馈对象，写清复现步骤、期望结果、实际现象和完整报错。
3. 点`查看`检查随附信息，保留相关日志；按需勾选当前部署清单。
4. 确认内容后发送，检查发送成功提示。

Robonix Console 反馈可附系统与硬件、版本、最近日志和管理程序设置；当前部署清单默认不勾选，选中后其中的密钥会替换为占位符。Robonix Hub 网页反馈附浏览器信息。检查附件是否包含本次运行日志，缺失时另外整理提供。

一条反馈描述一个问题，注明整机配置、软件版本和发生时间。Robonix Console 无法连接 Robonix Hub 时，可`下载反馈包`交给维护者。需要恢复配置时，先导出历史留底，再对比、回滚并重启；本机资源与代码另行检查。

<figure>
  <img src={useBaseUrl('/img/hub/manual/feedback.webp')} alt="反馈表单和随附信息" width="1680" height="1232" loading="lazy" />
  <figcaption>①选择反馈对象，②填写描述，③预览附件，④按需勾选当前部署清单，⑤发送。</figcaption>
</figure>

### 排查与日志

| 现象 | 优先检查 |
|---|---|
| 无法登录 Robonix Hub 或 Robonix Console | 账户激活、协议确认、密码和二次验证；有网络异常时查看 [Robonix Hub 服务状态](https://hub.robonix.ai/status) |
| Robonix Console 页面打不开 | 安装结果、本机 `127.0.0.1:4880`；远程使用时检查 SSH 转发 |
| 首次启动停在仿真准备阶段 | 当前用户运行 `docker ps` 是否正常，网络是否可下载镜像，Before boot 命令是否保存 |
| 停止后仿真仍在运行 | After exit 命令是否为 `bash sim/stop.sh`，退出钩子的日志是否报错 |
| Robonix Client 离线 | Robot Host 和 Atlas Port 是否正确，是否允许其他机器连接，以及运行和组件是否就绪 |
| 模型请求失败 | 模型地址、密钥、模型名与网络，重启后是否使用最新设置 |
| 参数修改未生效 | 当前部署是否正确，修改是否保存，以及是否需要重新构建或启动 |
| 地图、导航或场景异常 | 参阅[相关功能指南](#相关功能与概念)，检查其依赖和组件日志 |
| 恢复备份后无法运行 | 软件包、本机凭据、地图和资源是否补齐 |

下载和构建任务看 Robonix Console`更新`页的任务日志；运行看`日志`，选择本次记录，再看 `rbnx 输出` 和具体组件标签。用筛选定位错误，必要时显示全部或下载完整日志。启动与退出钩子执行失败时，保留相应输出，不只提供`启动失败`四个字。

## 相关功能与概念

仿真连接与首个任务验证完成后，按需求阅读相应章节。参数与功能说明以对应章节为准。

| 需求 | 对应指南 |
|---|---|
| 创建地图、保存地图与定位 | [建图与定位](../components/mapping.md) |
| 设置导航输入、目标与速度 | [导航服务](../components/navigation.md) |
| 查看相机、标注房间与检查场景 | [场景服务](../components/scene.md) |
| 查看任务执行树、相机与状态 | [图形客户端](client.md) |

部署、能力和组件的定义分别见[部署与启动](../architecture/deployment-and-startup.md)、[命名空间与能力约定](../architecture/namespace-and-contracts.md)和[系统组件](../architecture/components.md)。需要查看 `rbnx` 命令时，参阅[命令行工具](../components/rbnx.md)。

</div>
