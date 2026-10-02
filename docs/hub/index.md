# Robonix Hub 概览

Robonix Hub 用来在机器人上安装、配置和运行 Robonix，并通过软件包目录获取原语（Primitive）、服务（Service）和技能（Skill）。本章面向在机器人上部署和管理 Robonix 的用户。

## 两个组成部分

| | 本地 Robonix Hub | Robonix Hub Cloud |
|---|---|---|
| 运行位置 | 机器人上，默认地址 `http://127.0.0.1:4880` | [hub.robonix.ai](https://hub.robonix.ai) |
| 用途 | 管理所在机器人：部署、运行、日志、更新 | 账户、软件包目录、云端副本、问题反馈 |
| 能否修改机器人 | 能 | 不能 |

两者是不同的程序，外观风格一致，使用同一个账户。本地 Robonix Hub 侧栏中 `Robonix Hub Cloud` 下的页面（`目录`、`我的机器人`、`个人主页`）显示云端的内容，由本地 Robonix Hub 向云端请求。

## 先试用

在线演示 [hub.robonix.ai/demo](https://hub.robonix.ai/demo/) 是一个真实运行的本地 Robonix Hub，无需安装和登录，所做的修改只保存在当前浏览器中。

## 本章内容

- [账户](./account.md)：注册、激活和登录。
- [安装本地 Robonix Hub](./install.md)：安装、基本配置、自动更新和卸载。
- [配置机器人](./configure.md)：部署页、参数表单、本机环境变量和版本。
- [运行](./run.md)：构建软件包、启动与停止、日志、连接 robonix-client。
- [目录与云端](./cloud.md)：软件包目录、收藏、云端副本和 API。
- [反馈问题](./feedback.md)：在页面上提交问题，以及提交后的处理。
