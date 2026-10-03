# 安装 Robonix Hub

本页说明在机器人上安装 Robonix Hub。前提是机器人运行 x86_64 或 aarch64（包括 Jetson）上的 Debian 或 Ubuntu，能访问 [hub.robonix.ai](https://hub.robonix.ai)，并且有一个运行 Robonix 的普通用户。完成后，可以在浏览器中打开 Robonix Hub。

## 执行安装命令

以运行 Robonix 的普通用户执行，不要使用 root 或 `sudo`：

```bash
curl -fsSL https://hub.robonix.ai/install | bash
```

安装脚本分 8 步执行，每一步显示编号：

| 步骤 | 内容 |
|---|---|
| 检查本机 | 显示用户、处理器架构、系统版本和安装位置 |
| 系统软件包 | 缺少 git、curl、gpgv、xz 或 python3-venv 时安装；只在这一步询问一次 `sudo` 密码 |
| 许可协议 | 校验发布签名，在终端中央的窗口中按系统语言显示许可协议；选择 `同意` 后继续 |
| 下载 | 按本机架构下载 Robonix Hub，校验 sha256 和签名 |
| Robonix | 已安装 `rbnx` 时直接使用；未安装时把源码下载到 `~/.local/share/robonix-hub/robonix` 编译安装，耗时较长 |
| 服务 | 作为当前用户的 systemd 服务运行，并设为开机启动 |
| 基本配置 | 设置 Pilot 使用的模型服务，见下一节 |
| 完成 | 确认页面已响应，给出访问地址 |

程序位于 `~/.local/bin/robonix-hub`，数据位于 `~/.local/share/robonix-hub`。拒绝许可协议时，安装立即结束，不留下文件。

## 设置模型服务

Pilot 和 [AI 机器人配置助手](./assistant.md) 通过兼容 OpenAI 的接口调用模型。模型服务由三项组成，在 `设置` 页显示为 `VLM 终端 URL`、`VLM KEY` 和 `VLM 模型`，对应变量 `VLM_BASE_URL`、`VLM_API_KEY` 和 `VLM_MODEL`。

- 本机尚无这三项时，安装脚本询问 `Set it now? [Y/n]`。直接回车依次填写，密钥输入时不显示；输入 `n` 跳过。
- 本机已有这三项，或安装前已在终端中设置了同名环境变量时，安装脚本列出现有的值（密钥只显示末 4 位），询问 `Keep these? [Y/n]`。直接回车保留；输入 `n` 后逐项修改，某一项直接回车即保留原值。

这三项保存为本机环境变量，只有当前用户可读，不写入部署清单，也不会上传。之后可以在 `设置` 页的 `本机环境变量` 中查看和修改。

## 访问 Robonix Hub

在机器人上用浏览器打开 `http://127.0.0.1:4880`。从另一台电脑访问时，先建立 SSH 隧道，再在该电脑上打开同一地址：

```bash
ssh -L 4880:127.0.0.1:4880 USER@ROBOT_HOST
```

`USER` 为机器人上运行 Robonix Hub 的用户，`ROBOT_HOST` 为机器人的地址。也可以在 `设置` 中开启局域网访问，并同时限定可以登录的账户。

## 自动更新

Robonix Hub 在启动时和此后每 6 小时检查一次新版本。新版本经签名校验后自动安装并重启；Robonix 和正在运行的部署不受影响。新版本无法正常启动时，自动退回原版本，并在 `更新` 页记录原因。

新版本的许可协议有变化时，Robonix Hub 不自动更新，需要在 `更新` 页阅读并接受新协议后再更新。`更新` 页的 `Robonix Hub` 一栏显示当前版本和更新记录，并提供 `检查更新` 和 `自动更新` 开关。

## 卸载

```bash
curl -fsSL https://hub.robonix.ai/install | bash -s -- --uninstall
```

卸载删除程序和服务，保留部署、版本和日志。
