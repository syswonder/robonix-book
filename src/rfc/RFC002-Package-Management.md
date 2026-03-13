# RFC 002: Robonix Package Management

Robonix 包管理规范

| 版本 | 日期 | 作者 |
|------|------|------|
| 0.1 | 2026-03-11 | 韩喻泷 |

<!-- toc -->

## 1. 目标与术语

### 1.1 目标

Package 即“一个可构建、可启动的应用”的交付单元。本 RFC 规定：

- Package 的身份与元数据（通过 `robonix_manifest.yaml` 描述）。
- Node 与 entry 的语义：一个 package 可声明多个 node，每个 node 对应一个进程，由一个启动 entry 描述如何拉起。
- 构建与启停：仅通过 rbnx 工具完成，无单独 deployment 规范（不引入“部署描述文件”或“部署阶段”）。

Manifest 核心是版本、权限（预留）以及各 node 的启动 entry；不要求声明“实现了哪些 RIDL 接口”（类比 Android：不要求声明实现了多少 AIDL）。

### 1.2 术语

- Package：可构建、可启动的单元；根目录包含 `robonix_manifest.yaml`，并可包含源码、依赖描述等。
- Node：运行起来的一个进程。在通信模型中，node 是 stream/query/command 的目标标识（调用方指定“发给哪个 node”）。
- Node id：在系统内唯一标识一个 node；与 manifest 中 `nodes[].id` 对应，也是通信时解析 channel 所用的目标标识，建议使用 `com.syswonder.xxx` 等反向域名风格。
- Entry：描述如何启动一个 node，格式为 `模块:函数`（如 `python_ping_client.call_ping:main`），由 rbnx start 时用于构造启动命令。

---

## 2. Package 目录与 manifest 文件

### 2.1 约定

- Package 根目录：包含且仅包含一个 robonix_manifest.yaml（文件名固定）。
- 其他内容（Python 包、C++ 包、资源文件等）由具体构建策略决定；rbnx build 会依据 manifest 与当前实现执行校验、colcon 构建等，输出到 package 局部的构建目录（如 `rbnx-build`），不要求统一顶层子目录名。

### 2.2 文件位置

`robonix_manifest.yaml` 必须位于 package 的根目录。rbnx 通过该文件识别目录为一个 package（与 `-p` 解析规则配合）。

---

## 3. robonix_manifest.yaml 结构

### 3.1 顶层结构

```yaml
manifestVersion: 1

package:
  id: <package 稳定标识>
  name: <包名>
  version: <版本号>
  vendor: <厂商>
  description: <简短描述>
  license: <许可证 SPDX 或名称>

# 可选
# permissions: []

nodes:
  - id: <node id，建议 com.xxx.xxx>
    type: python   # 可选，默认 python
    entry: <模块:函数>
  # 可有多项，即多个 node；每次 rbnx start -p -n <node_id> 只启动一个
```

- manifestVersion：当前固定为 `1`，用于将来 schema 演进。
- package：必填，描述 package 身份与版本。
- permissions：可选，预留；将来可声明能力/权限需求。
- nodes：必填且至少一项；每项描述一个 node 的启动 entry（id、type、entry）。每次 `rbnx start -p <package> -n <node_id>` 只启动一个 node。

### 3.2 package 字段说明

| 字段 | 必填 | 说明 |
|------|------|------|
| `package.id` | ✓ | 稳定标识，可用于仓库、依赖引用等；建议反向域名，如 `com.robonix.example.ping_client`。 |
| `package.name` | ✓ | 包名，用于构建产物、日志、rbnx 解析等；通常与目录名或主模块名一致。 |
| `package.version` | ✓ | 版本号，语义化版本或项目约定格式。 |
| `package.vendor` | ✓ | 厂商或组织名。 |
| `package.description` | ✓ | 简短文字描述。 |
| `package.license` | ✓ | 许可证标识（如 MulanPSL-2.0）或名称。 |

### 3.3 nodes（node 列表）字段说明

| 字段 | 必填 | 说明 |
|------|------|------|
| `nodes` | ✓ | Node 列表；至少一项。每项对应一个进程（node）。`rbnx start -p <package> -n <node_id>` 每次只启动一个 node。 |
| `nodes[].id` | ✓ | Node id。运行时的进程标识，也是 stream/query/command 通信时的目标节点标识。必须在系统内唯一，建议 `com.syswonder.xxx` 风格。 |
| `nodes[].type` | - | 节点类型，当前支持 `python`；可省略，默认 `python`。 |
| `nodes[].entry` | ✓ | 该 node 的启动入口，格式 `模块:函数`（如 `mypkg.main:main`）。rbnx start 会据此构造启动命令。 |

### 3.4 Node id 与通信的关系

Manifest 中的 node id（`nodes[].id`）与运行时通信一致：

- 当某 node 作为 server 提供某 RIDL 接口时，会使用该 node id 向 robonix-server 注册 channel。
- 当调用方要调用某 node 的某接口时，需指定 (node_id, interface_id)；meta API 据此解析出 channel，再完成 ROS 调用。

因此 node id 的命名应全局唯一、可读、稳定，推荐反向域名，如 `com.syswonder.hal_arm`、`com.syswonder.map_semantic`。

### 3.5 完整示例

```yaml
manifestVersion: 1

package:
  id: com.robonix.example.ping_client
  name: python_ping_client
  version: 0.1.0
  vendor: robonix
  description: Python 客户端，调用 robonix/system/debug/ping
  license: MulanPSL-2.0

# 节点列表：每个 node 一个启动 entry；node id 即通信时的目标标识
nodes:
  - id: com.syswonder.example.ping_client
    type: python
    entry: python_ping_client.call_ping:main
```

---

## 4. rbnx 命令

### 4.1 概览

构建与启停均由 rbnx 完成，无单独“deploy”步骤或部署描述文件。

```bash
rbnx build -p <package>           # 构建
rbnx start -p <package> -n <node_id>   # 启动指定 node，阻塞直到进程退出
```

### 4.2 `-p` 参数：package 解析

`-p` 可为以下之一：

- 路径：指向包含 `robonix_manifest.yaml` 的目录（绝对或相对路径）。
- 名字：rbnx 在以下位置按名称查找带 `robonix_manifest.yaml` 的子目录（名字即该子目录名或 package.name）：
  - 当前工作目录下的 `examples/<name>`
  - 当前工作目录下的 `<name>`
  - 当前工作目录下的 `rust/examples/<name>`

未找到则报错并提示已尝试的路径。

### 4.3 build

- 校验 manifest 语法与必填字段。
- 根据当前实现执行构建（如 colcon 构建、安装依赖、生成代码等）；构建产物落在 package 局部的构建目录（如 `rbnx-build`）及 install 空间，供 start 使用。
- 具体步骤（如是否调用 ridlc、colcon 包列表）以实现为准；本 RFC 仅约定“由 rbnx 统一入口、含 manifest 校验”。

### 4.4 start

- 解析 `-p` 得到 package 根目录，`-n` 指定要启动的 node id；加载 manifest。
- 根据该 node 的 `type` 与 `entry` 构造启动命令（如 Python：source 环境 + `python3 -m <module>` 等），在 package 或约定的工作目录下启动进程；进程在运行时以该 node 的 id 作为 node id（例如向 meta API 注册时使用）。阻塞直到该进程退出。
- 可选：通过 `--endpoint` 或环境变量 `ROBONIX_META_GRPC_ENDPOINT` 指定 robonix-server 的 meta gRPC 地址，默认如 `127.0.0.1:50051`。

---

## 5. 构建与运行产物（约定）

- 构建目录：可在 package 根下或同级目录使用固定子目录（如 `rbnx-build`）存放 colcon workspace、build/install 等；具体以实现为准。
- 日志与进程信息：rbnx start 可将日志、PID 等写入约定位置（如 `rbnx-deploy/logs` 或等价路径），便于排查与 stop 时匹配进程。
- 环境：start 时需注入 ROS/colcon 环境（如 source setup）、以及 meta gRPC 地址等，以便 node 能注册与解析 channel。

---

## 6. 设计原则

- Manifest 聚焦：版本、权限（预留）、各 node 的启动 entry；不要求声明“实现了哪些 RIDL 接口”。
- 无 deployment 规范：不引入单独的部署描述文件或部署阶段；类似“安装后直接启动”的模型，由 rbnx 按 package 启停即可。
- Node 与通信一致：manifest 中的 node id 即通信时的目标标识，保证“谁启动、谁被调用”可追溯。
- 扩展预留：permissions、launchProfiles、config 等可在不破坏现有 schema 的前提下后续迭代。

---

## 7. 与 RFC001 的边界

- RFC001：接口契约（RIDL、channel 概念）；不涉及 package、manifest、rbnx。
- RFC002：Package、manifest、node、entry、rbnx build/start/stop；不涉及 RIDL 语法或 channel 分配算法。

两者在运行时的衔接：node 进程使用 manifest 中的 node id 向 robonix-server 注册或解析 channel，调用方通过 (node_id, interface_id) 解析 channel 并通信。
