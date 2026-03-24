# RFC 002: Robonix Package Management

Robonix 包管理规范

| 版本 | 日期 |
|------|------|
| 0.1 | 2026-03-11 |

## 1. 目标与术语

### 1.1 目标

在 Robonix 中，**Package** 表示“一个可以独立构建、独立启动”的应用交付单元。本 RFC 约定：package 的身份与版本信息如何描述（`robonix_manifest.yaml`），**Node** 与 **entry** 如何表达“启动哪个进程、从哪个入口函数进入”，以及为何**构建与启停统一走 rbnx**，而不引入单独的 deployment 描述文件或部署阶段。

Manifest 侧重版本、权限（预留）与启动信息；**不要求**在 manifest 中枚举“实现了哪些 RIDL 接口”。标准服务、**硬件原语**实现方、skill 与用户应用在通信层面都通过同一套 RIDL 契约对接，只需在运行时约定接口即可。

### 1.2 术语

| 术语 | 含义 |
|------|------|
| Package | 根目录含 `robonix_manifest.yaml` 的可构建单元 |
| Node | 一个进程；通信注册与解析的目标标识 |
| Node id | `nodes[].id`；通信解析用；建议 `com.syswonder.xxx` |
| Entry | `模块:函数`，`rbnx start` 用来构造启动命令 |


## 2. 目录约定

- Package 根：**恰好一个** `robonix_manifest.yaml`。
- 其余目录由语言/构建策略决定；产物多在 `rbnx-build` 等（实现细节不限定顶层子目录名）。


## 3. robonix_manifest.yaml

### 3.1 骨架

```yaml
manifestVersion: 1

package:
  id: <稳定标识>
  name: <包名>
  version: <版本>
  vendor: <厂商>
  description: <描述>
  license: <SPDX 或名称>

# permissions: []   # 可选，预留

nodes:
  - id: <node id>
    type: python    # 可选，默认 python
    entry: <模块:函数>
```

- `manifestVersion`：当前 `1`。
- `nodes`：至少一项；每次 `rbnx start -p <pkg> -n <node_id>` 只启一个 node。

### 3.2 `package` 字段

| 字段 | 必填 | 说明 |
|------|------|------|
| `id` | ✓ | 稳定 id；建议反向域名 |
| `name` | ✓ | 构建/rbnx 解析用 |
| `version` / `vendor` / `description` / `license` | ✓ | 元数据 |

### 3.3 `nodes` 字段

| 字段 | 必填 | 说明 |
|------|------|------|
| `nodes[].id` | ✓ | 进程与通信目标标识；系统内唯一 |
| `nodes[].type` | | 默认 `python` |
| `nodes[].entry` | ✓ | `模块:函数` |

**Node id 与通信**：server 注册、client 解析均用 `(node_id, interface_id)` → channel。

### 3.4 示例

```yaml
manifestVersion: 1

package:
  id: com.robonix.example.ping_client
  name: python_ping_client
  version: 0.1.0
  vendor: robonix
  description: Call robonix/system/debug/ping
  license: MulanPSL-2.0

nodes:
  - id: com.syswonder.example.ping_client
    type: python
    entry: python_ping_client.call_ping:main
```


## 4. rbnx

```bash
rbnx build -p <package>
rbnx start -p <package> -n <node_id>
```

### 4.1 `-p` 解析

- 路径：含 manifest 的目录。
- 名字：在当前目录下试 `examples/<name>`、`<name>`、`rust/examples/<name>`；失败则报错列路径。

### 4.2 `build`

校验 manifest；执行实现定义的构建（colcon、代码生成等）；产物供 `start`。

### 4.3 `start`

读 manifest；按 `type`+`entry` 启进程；阻塞至退出。Meta：`--endpoint` 或 `ROBONIX_META_GRPC_ENDPOINT`（默认如 `127.0.0.1:50051`）。


## 5. 产物（约定级）

- 构建目录：如 `rbnx-build`（以实现为准）。
- 日志/PID：如 `rbnx-deploy/logs`（以实现为准）。
- `start` 注入 ROS/colcon 环境与 meta 地址。


## 6. 设计原则

- Manifest 聚焦版本、权限（预留）、node entry；不强制列 RIDL。
- 无独立 deployment 阶段。
- **Node id = 通信目标**，可追溯。
- 预留：`permissions`、`launchProfiles`、`config` 等演进。


## 7. 与 RFC001

- **RFC001**：RIDL、channel 概念。
- **RFC002**：package、manifest、node、rbnx。

衔接：进程用 manifest 的 node id 向 meta 注册/解析；(node_id, interface_id) 解析 channel。
