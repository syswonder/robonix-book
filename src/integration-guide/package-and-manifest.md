# Manifest 与包管理

每个 Robonix 包的根目录下须有一个 `robonix_manifest.yaml`，`rbnx` CLI 通过它执行校验、构建和启动。

## Manifest 结构

```yaml
manifestVersion: 1

build:
  script: scripts/build.sh

package:
  id: com.robonix.example.vlm_service   # reverse-DNS 格式
  name: vlm_service
  version: 0.1.0
  vendor: robonix
  description: VLM service registered under robonix/srv/model/vlm
  license: MulanPSL-2.0

nodes:
  - id: com.robonix.services.vlm
    type: python
    start: exec python3 -m vlm_service.service
```

Docker compose 类型的节点示例（`tiago_sim_stack`）：

```yaml
nodes:
  - id: com.robonix.prm.tiago
    type: compose
    start: |
      CF=(-f compose.yaml)
      if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
        CF+=(-f compose.gpu.yaml)
        echo "[tiago_sim_stack] NVIDIA GPU detected — merging compose.gpu.yaml"
      fi
      exec docker compose "${CF[@]}" up --build ros2-bridge
```

## 字段说明

| 字段 | 说明 |
|------|------|
| `manifestVersion` | 当前固定为 `1` |
| `build.script` | `rbnx build` 时在包目录下执行的 shell 脚本路径 |
| `package.id` | 包的唯一标识，reverse-DNS 格式 |
| `package.name` | 人类可读的短名 |
| `nodes[].id` | 节点的 `node_id`，即注册到控制平面时使用的标识 |
| `nodes[].type` | 启动方式：`python`、`compose`、`binary` 等 |
| `nodes[].start` | 启动命令或内联 shell 脚本 |

## rbnx 命令

```bash
# 校验 manifest 格式
rbnx validate examples/packages/vlm_service

# 执行构建脚本
rbnx build -p examples/packages/vlm_service

# 启动节点（阻塞直到进程退出）
rbnx start \
  -p examples/packages/vlm_service \
  -n com.robonix.services.vlm \
  --endpoint 127.0.0.1:50051
```

`rbnx start` 在启动节点前会检查构建产物（`rbnx-build/.rbnx-built`），若不存在则自动执行构建脚本。控制平面地址优先取 `--endpoint` 参数，其次取 `ROBONIX_META_GRPC_ENDPOINT` 环境变量，默认为 `127.0.0.1:50051`。

## 构建脚本

`build.script` 以包目录为工作目录执行。典型用途：

- **Python 包**：设置 `PYTHONPATH`，运行 `robonix-codegen` 生成 `robonix_mcp_types/` 和 `proto_gen/`
- **Docker compose 包**：执行 `docker compose build`，在镜像构建阶段完成 codegen

以 `vlm_service` 为例（仅设置 PYTHONPATH）：

```bash
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${SCRIPT_DIR}${PYTHONPATH:+:$PYTHONPATH}"
```

声明 MCP 接口的包须在构建阶段完成 `robonix-codegen --lang mcp`，确保 `robonix_mcp_types/` 已生成再启动进程（详见 [Provider 注册](provider-registration.md)）。

## `DESCRIPTION.md`（包说明文件）

每个包的根目录**应**提供一份 `DESCRIPTION.md`，用于描述这个包本身——接口清单、源码组织、关键函数、使用方式、依赖约束等。它面向的是**想用这个包的开发者 / Agent**，不是 LLM 行为规范：

```
my_package/
├── robonix_manifest.yaml
├── DESCRIPTION.md         ← 包的说明（本节讨论的东西）
├── scripts/
├── src/
└── ...
```

`DESCRIPTION.md` 没有强制格式，但建议包含：

- **概述**：这个包做什么，典型使用场景
- **提供的接口**：列出 `robonix_manifest.yaml` 里 `interfaces.provides` 中的每个契约，说明语义 / 数据速率 / 典型消费者
- **消费的接口**：`interfaces.consumes` 的依赖说明，以及这些依赖通常由谁提供
- **源码组织**：关键模块、入口点、实现要点
- **运行时参数 / 环境变量**：启动行为可以被哪些 env / YAML 字段调整
- **已知限制 / TODO**

### 与 Skill 的区别

`DESCRIPTION.md` 描述的是**包**（部署单元），与 Robonix 的 [Skill](../skill-library.md) 概念是两回事：

| | `DESCRIPTION.md` | Skill |
|---|---|---|
| 位置 | 每个包的根目录 | 独立注册到 Atlas（技能 主动提交 / `~/.robonix/skills/` / `ROBONIX_SKILLS_EXTRA_DIRS`） |
| 面向 | 开发者、集成方、读源码的 Agent | 运行时 VLM（system prompt 注入） |
| 内容 | 包的说明书：接口 / 源码 / 函数 / 用法 | Agent 行为单元：基本技能（技能 进程）或 RTDL 结构化技能图 |
| 是否自动注册到 Atlas | 否 | 由专门的 skill 注册路径管理 |

> **历史变化**：早期 `rbnx start` 会自动扫描包内 `skills/<name>/` 目录并向 Atlas 注册。这一机制已取消——它让"包"与"Skill"这两个独立概念产生耦合（包是部署单元，Skill 是 Agent 能力库）。现在两者完全解耦：包根目录的描述统一放 `DESCRIPTION.md`，Skill 通过独立路径注册（详见技能库文档）。
