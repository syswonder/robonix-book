# Manifest 与包管理

每个 Robonix 包的根目录下必须有一个 `robonix_manifest.yaml`，`rbnx` CLI 通过它来执行校验、构建和启动。

## Manifest 结构

以 `vlm_service`（Python 服务）为例：

```yaml
# rust/examples/packages/vlm_service/robonix_manifest.yaml
manifestVersion: 1

build:
  script: scripts/build.sh

package:
  id: com.robonix.example.vlm_service
  name: vlm_service
  version: 0.1.0
  vendor: robonix
  description: VLM service registered under robonix/sys/model/vlm
  license: MulanPSL-2.0

nodes:
  - id: com.robonix.services.vlm
    type: python
    start: exec python3 -m vlm_service.service
```

以 `tiago_sim_stack`（Docker compose）为例：

```yaml
# rust/examples/packages/tiago_sim_stack/robonix_manifest.yaml
manifestVersion: 1

build:
  script: scripts/build.sh

package:
  id: com.robonix.example.tiago_sim_stack
  name: tiago_sim_stack
  version: 0.1.0
  vendor: robonix
  description: Tiago sim stack — Docker image runs Webots, Nav2, and tiago_bridge
  license: MulanPSL-2.0

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

`manifestVersion` 目前固定为 `1`。

`build.script` 指向一个 shell 脚本，`rbnx build` 时在包目录下执行。典型用途是设置 `PYTHONPATH`、编译 proto 桩、构建 Docker 镜像等。

`package` 段描述包的元数据。`id` 使用 reverse-DNS 格式，`name` 是人类可读的短名。

`nodes` 是一个数组，每个元素描述一个可启动的节点。`id` 是注册到控制平面时使用的 `node_id`。`type` 表示启动方式（`python`、`compose`、`binary` 等，影响 `rbnx start` 的进程管理策略）。`start` 是启动命令或内联脚本。

## rbnx 命令

校验 manifest 格式：

```bash
rbnx validate examples/packages/vlm_service
```

执行构建脚本：

```bash
rbnx build -p examples/packages/vlm_service
```

启动一个节点（`-n` 指定 node_id，`--endpoint` 指定控制平面地址）：

```bash
rbnx start \
  -p examples/packages/vlm_service \
  -n com.robonix.services.vlm \
  --endpoint 127.0.0.1:50051
```

`rbnx start` 启动节点进程后会阻塞直到进程退出。在 `run.sh` 中通常用 `&` 放入后台。

## 构建脚本

`scripts/build.sh` 在 `rbnx build` 时以包目录为 cwd 执行。以 vlm_service 为例，它只需设置 PYTHONPATH：

```bash
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${SCRIPT_DIR}${PYTHONPATH:+:$PYTHONPATH}"
```

Docker compose 类型的包通常在构建脚本中执行 `docker compose build`。
