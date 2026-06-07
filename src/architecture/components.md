# 系统组件

[toc]

Robonix 把"操作系统"职责拆成 **12 个系统组件**，按角色划分（不是按实现语言）。每个组件在 `system/<name>/` 下有自己的目录和 README。

本页描述的是 **dev 分支当前真实的实现状态**——哪些已落地、哪些还是 stub——而不是白皮书的目标蓝图。

## 12 个组件

| 组件 | 负责 | 状态 | 实现 |
|------|------|------|------|
| **atlas** | 能力发现 / 目录 | 已实现 | Rust（`system/atlas`） |
| **chronos** | 统一时间 / PTP 对齐 | stub | `system/chronos`（仅 README） |
| **executor** | 方案编排与能力分发 | 已实现 | Rust（`system/executor`） |
| **keystone** | 身份 / 配置 / 策略 | stub | `system/keystone` |
| **liaison** | 人机交互（chat / 语音 / TUI） | 已实现 | Rust（`system/liaison`） |
| **nexus** | gRPC / MCP / ROS 2 通信库的集合 | 库，非独立进程 | 现有 gRPC/MCP/ROS 2；未来加自研 ROS 2 零拷贝 |
| **pilot** | 规划 / 决策 / 记忆 / 世界模型 | 已实现 | Rust（`system/pilot`） |
| **scene** | 场景状态 / 语义地图 / 对象注册表 | 已实现 | Python（`system/scene`） |
| **scribe** | 结构化日志 / 回放 / 审计 | stub | `system/scribe` |
| **sentinel** | 安全监督 | 内嵌 executor（v0.1） | `system/sentinel` |
| **soma** | 本体状态 / 设备与原语抽象 | stub（能力约定已定） | `system/soma` |
| **vitals** | 健康监测 / 心跳 | 部分（经 atlas 心跳） | `system/vitals` |

## 实现 vs stub

v0.1 真正以独立进程跑起来的有 5 个：**atlas / executor / pilot / liaison**（Rust 二进制，`make install` 装到 `~/.cargo/bin`）和 **scene**（Python 服务）。另外几个不是独立进程：

- **nexus**：不是进程，而是一套通信库的集合——gRPC、MCP、ROS 2 的客户端/服务端实现，每个组件直接链接使用（能力约定就投影到这三种 transport 上）。规划中还包括自研的 ROS 2 零拷贝库。它不是"待实现的 stub"，而是已经在用的那套传输代码。
- **sentinel**：安全监督作为 executor 的子模块跑（在能力分发链路上拦截），还没拆成独立组件。
- **vitals**：健康监测的心跳那一半在 atlas 里（provider 周期心跳、超时驱逐），独立的 vitals 组件未起。

其余 4 个——**chronos / keystone / scribe / soma**——是 v0.1 的 stub：目录和 README 占位，规划在后续版本落地（soma 的能力约定已经定义，见 [Soma 接口](../interface-catalog/system/soma.md)，缺的是参考 provider）。

## 与能力约定层的关系

system 组件里对外暴露能力约定的有 pilot / executor / liaison / scene / soma（命名空间 `robonix/system/*`，见 [系统接口](../interface-catalog/system/index.md)）。atlas 用自己的 gRPC 控制面（不是能力约定，见 [Atlas 能力目录](atlas.md)）。其余 stub 暂无能力约定。

启动顺序见 [系统部署与启动流程](deployment-and-startup.md)。
