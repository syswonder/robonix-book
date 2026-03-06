# robonix-server（当前进程名：robonix-core）

<!-- toc -->

本文档说明系统核心进程（当前二进制名为 `robonix-core`）的运行方式，包括启动、环境变量与基本确认。其内部接口与模块设计请参阅 [第三章 - Robonix Framework](../chapter3-developer-guide/robonix-framework.md)。

## 作用

- 保存原语、服务、技能的注册信息（由 `rbnx deploy register` 写入）。
- 提供任务提交接口，负责规划与执行调度。
- 可选提供 Web 管理界面（Robonix Console）。

运行前需已安装二进制（`make install-core`），并已 source ROS2 与 robonix-sdk 环境。

## 启动

无命令行参数，行为由环境变量控制。在已 source ROS2 与 robonix-sdk 的终端执行：

```bash
robonix-core
```

或于 `robonix/rust` 下执行 `./core.sh`（后台、带 Web UI）。

## 环境变量

| 变量 | 说明 |
|------|------|
| `ROBONIX_WEB_ASSETS_DIR` | Web 界面静态资源目录（如 `$(pwd)/robonix-core/web`）；与 `ROBONIX_WEB_PORT` 同时设置时启用 Web 界面 |
| `ROBONIX_WEB_PORT` | Web 服务端口（如 `8000`）；与上面同时设置时在浏览器访问 `http://localhost:8000` 打开 Robonix Console |
| `RUST_LOG` | 日志级别，如 `robonix_core=info`、`robonix_core=debug` 或 `debug` |

未设置 Web 变量时仅提供 ROS2 服务，不启动 HTTP。

## 确认运行

- `ros2 service list | grep rbnx` 可见 `/rbnx/prm/query`、`/rbnx/task/submit` 等。
- 已设 Web 端口时，浏览器访问 `http://localhost:<ROBONIX_WEB_PORT>` 进入 Robonix Console。

## 与 rbnx 的关系

- 注册/注销由 `rbnx deploy register` / `rbnx deploy unregister` 完成，rbnx 经 daemon 调用本进程 ROS2 服务。
- 能力进程的启动/停止由 `rbnx deploy start` / `rbnx deploy stop` 控制；本进程仅维护能力元数据。

环境准备见 [快速开始](../chapter1-getting-started/quickstart.md)。
