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
