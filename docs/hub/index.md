# Robonix Console 与 Robonix Hub

Robonix Console 用来在机器人上安装、配置和运行 Robonix。Robonix Hub 提供软件包目录，从中获取原语（Primitive）、服务（Service）和技能（Skill）。本章面向在机器人上部署和管理 Robonix 的用户。

## 两者的分工

| | Robonix Console | Robonix Hub |
|---|---|---|
| 运行位置 | 机器人上，默认地址 `http://127.0.0.1:4880` | [hub.robonix.ai](https://hub.robonix.ai) |
| 用途 | 管理所在机器人：部署、运行、日志、更新 | 账户、软件包目录、云端备份、分享、问题反馈 |
| 能否修改机器人 | 能 | 不能 |

两者外观风格一致，使用同一个账户。Robonix Console 侧栏分为两组：`这台机器人` 下的页面管理所在机器人；`Robonix Hub` 下的页面（`目录`、`我的机器人`、`个人主页`、`用量`）显示 Robonix Hub 上的内容，由 Robonix Console 向 Robonix Hub 请求。

## 在线演示

在线演示 [hub.robonix.ai/demo](https://hub.robonix.ai/demo/) 是一个真实运行的 Robonix Console，无需安装和登录，所做的修改只保存在当前浏览器中。

## 章节导航

- [账户](./account.md)：激活账户和登录。
- [安装 Robonix Console](./install.md)：安装、基本配置、自动更新和卸载。
- [配置机器人](./configure.md)：配置页、参数表单、机器人环境变量和版本。
- [运行](./run.md)：构建软件包、启动与停止、日志、连接 robonix-client。
- [AI 机器人配置助手](./assistant.md)：用一句话检查和修改配置，改动先经确认。
- [目录与云端](./cloud.md)：软件包目录、收藏、云端备份、分享和 API。
- [反馈问题](./feedback.md)：在页面上提交问题，以及提交后的处理。
