# Robonix 包与部署配置规范

Robonix 部署分**两层** manifest：

| 文件 | 范围 | 谁读 |
|---|---|---|
| 部署根目录 `robonix_manifest.yaml` | 部署 | `rbnx deploy` |
| 每个包内 `package_manifest.yaml` | 包 | `rbnx start` |

## 命令

```bash
# 整个栈一键起（读 deploy manifest，依次启动 system + 所有包）
rbnx deploy -f robonix_manifest.yaml

# 只起单个包（开发调试）
rbnx start -p ./service/slam_fastlio2
```

`rbnx deploy` 的流程：
1. 展开 `${VAR}` 环境变量
2. 起 `system:` 服务（atlas / executor / pilot / liaison / memory / vlm 等 cargo 二进制）
3. 对每个 `primitive` / `service` / `skill` 条目：把它的 `config` 块写到 `rbnx-deploy/instances/<name>.json`，然后 `rbnx start -p <path>`，env 里带两个变量：`RBNX_CONFIG_FILE=<json-path>` 和 `RBNX_INSTANCE_NAME=<name>`
4. 日志落到 `rbnx-deploy/logs/<component>.log`
5. Ctrl-C 统一 kill

> config 用文件传递（不是 env 里直接塞 JSON）——避开 bash 对引号/换行的 escape、`ARG_MAX` 限制、以及 `printenv | jq` 这种不直观的 debug 路径。包里一行 `jq` 就能读配置，同一个包的多个 instance（name 不同）各有自己的 json 文件，互不干扰。

## Deploy manifest 示例

```yaml
name: my-robot

env:
  ROS_DISTRO: humble

# 每个 system 服务的 config 直接写在它自己下面，不重复。
# pilot 会从 vlm.listen 自己推断该 dial 哪里，不需要再在 pilot 里写一遍。
system:
  atlas:    { listen: 127.0.0.1:50051 }
  executor: { listen: 127.0.0.1:50061 }
  pilot:    { listen: 127.0.0.1:50071 }
  liaison:  { listen: 127.0.0.1:50081 }
  memory:   { backend: sqlite, path: ${HOME}/.robonix/memory.db }
  vlm:
    listen: 127.0.0.1:50091
    upstream: ${VLM_BASE_URL}
    api_key:  ${VLM_API_KEY}
    model:    ${VLM_MODEL}
    api_format: openai

# 硬件。每个条目是一个硬件实例。
primitive:
  - package: com.robonix.primitive.sensor.lidar3d.mid360
    path: ./primitive/sensor_lidar3d_mid360
    name: lidar3d
    config:
      ip: 192.168.1.161
      mounted_frame: livox_frame

# 场景服务。path 是本地路径；url 是 git 地址（首次 clone 到 rbnx-deploy/cache/）。
service:
  - package: com.robonix.service.slam.fastlio2
    path: ./service/slam_fastlio2
    name: slam
    config:
      mode: mapping
      cube_len: 100
      map_dir: ${HOME}/.robonix/maps/current

  - package: com.robonix.service.nav.nav2
    url: https://github.com/syswonder/robonix-nav.git
    branch: v0.3
    name: nav
    config:
      planner: SmacPlanner2D

skill: []
```

### 条目字段

- **`package`**：包名，要和包 manifest 里的 `package.name` 一致
- **`path`** 或 **`url`**：二选一。`path` 相对 manifest 目录；`url` 是 git，首次 clone 到 `rbnx-deploy/cache/<name>/`，可加 `branch:` 锁分支
- **`name`**：instance 名字 / 日志前缀
- **`config`**：任意 YAML 字典，JSON 化成 `RBNX_CAP_CONFIG_JSON` 给包

## Package manifest 示例

```yaml
manifestVersion: 1

package:
  name: com.robonix.service.slam.fastlio2
  version: 0.1.0
  vendor: syswonder
  description: FASTLIO2 3D LiDAR-Inertial SLAM + PGO
  license: MulanPSL-2.0

build: bash scripts/build.sh     # 可选；首次 rbnx start 会自动 build
start: bash bin/start.sh         # 必填；rbnx start 入口

# 提供的能力。name 是 contract id，可选 path 指向包本地 TOML（当你自定义接口时）。
# 不写 path 就去 rust/contracts/ 找官方 TOML。
capabilities:
  - name: robonix/service/slam/lio_odom
  - name: robonix/service/slam/save_map
  - name: robonix/service/slam/switch_mode

depends:
  - name: com.robonix.primitive.sensor.lidar3d.mid360
```

### 包里读 config

rbnx 传两个 env 变量：
- `RBNX_CONFIG_FILE`：本 instance 的配置 json 绝对路径
- `RBNX_INSTANCE_NAME`：本 instance 名字（同包多实例时用来区分）

```bash
# bin/start.sh
set -eo pipefail
: "${RBNX_CONFIG_FILE:=/dev/null}"      # 单独 rbnx start 的 fallback
MODE=$(jq -r '.mode // "mapping"' < "$RBNX_CONFIG_FILE")
CUBE=$(jq -r '.cube_len // 100'  < "$RBNX_CONFIG_FILE")
echo "[start] instance=$RBNX_INSTANCE_NAME mode=$MODE cube=$CUBE"
exec ros2 launch my_pkg.launch.py mode:="$MODE" cube_len:="$CUBE"
```

或者 Python：
```python
import json, os
cfg = json.load(open(os.environ["RBNX_CONFIG_FILE"]))
mode = cfg.get("mode", "mapping")
```

## 设计说明

### 为什么 config 是透传，不做 schema

每个包最清楚自己的配置。Robonix 核心不定 schema，包自己 parse `RBNX_CAP_CONFIG_JSON`，加字段不用改核心代码。

### 多实例（primitive 一机多件）

一台车两个 MID360，或同一个 camera 驱动挂两个摄像头：deploy manifest 里写两条就行。

```yaml
primitive:
  - package: com.robonix.primitive.sensor.lidar3d.mid360
    path: ./primitive/sensor_lidar3d_mid360
    name: lidar_front
    config: { ip: 192.168.1.161, mounted_frame: livox_front, topic_prefix: /lidar_front }
  - package: com.robonix.primitive.sensor.lidar3d.mid360
    path: ./primitive/sensor_lidar3d_mid360
    name: lidar_rear
    config: { ip: 192.168.1.162, mounted_frame: livox_rear, topic_prefix: /lidar_rear }
```

两条用同一个包、同一个 path，`name` 和 `config` 不同。rbnx 会分别 spawn 两个 `rbnx start`，每个拿到自己的 `RBNX_CAP_CONFIG_JSON`。包里要根据 config（比如 `topic_prefix`）决定发什么 topic、以什么 instance id 注册到 atlas（如 `robonix/primitive/sensor/lidar3d@front`）。

### Primitive 的 driver 生命周期

每个抽象硬件类别对应一个 driver contract（如 `robonix/primitive/sensor/lidar3d/driver`）。driver 是普通 RPC 接口，里面通过 `command` 字段区分 `INIT / RESET / SHUTDOWN / PROBE` 四个操作。Robonix 启动 primitive 包后，会先调它的 driver `INIT(config_json)` 做硬件初始化 / 自检 / 参数下发；失败则不进入数据面。`PROBE` 可随时被调用查询状态。`config_json` 是从 manifest 透传下来的字符串，包自己解析。

Driver 的 IDL（共享的）在 `rust/crates/robonix-interfaces/lib/robonix_msg/srv/Driver.srv`：
```
uint8 CMD_INIT = 0
uint8 CMD_RESET = 1
uint8 CMD_SHUTDOWN = 2
uint8 CMD_PROBE = 3
uint8 command
string config_json
---
bool ok
string state      # uninit | ready | error | shutdown
string error
```

## 开发自己的包

### 接口来源：官方 vs 包内

| 层 | 接口在哪 | 说明 |
|---|---|---|
| **primitive** | `rust/contracts/primitive/` | 官方标准，接入新硬件按已有接口实现；有空缺提 PR 新增 |
| **service** | `rust/contracts/service/`（多数）+ 少量包内 | 场景服务大多复用官方接口（SLAM / nav / perception），只有明确私有的才自定义 |
| **system** | `rust/contracts/system/` | 控制面，全官方（pilot / executor / memory / vlm 等） |
| **skill** | **全部在包内** | skill 是 agent 层，每个包自己定义，不进主仓库 |

### Primitive / Service 包：实现官方接口

`capabilities:` 里**只写 name**，不写 path：

```yaml
capabilities:
  - name: robonix/primitive/sensor/lidar3d
  - name: robonix/primitive/sensor/lidar3d/driver
```

Robonix 去 `rust/contracts/primitive/sensor/lidar3d.v1.toml` 查接口形状，你的代码按 TOML 里指向的 ROS IDL / proto 实现。别人看到你的包立刻知道它"提供什么"，接口互通。

### Skill 包：TOML 写在包里

skill 的接口是 agent 层面的，每个应用都不一样，**没有官方标准**。TOML 放包里，`capabilities:` 用 `path` 指：

```yaml
capabilities:
  - name: robonix/skill/my_stack/weird_thing
    path: capabilities/weird_thing.v1.toml
```

TOML 格式和 primitive/service 的官方结构一致（`[contract]` + `[io.srv]` / `[io.msg]` + `[mode]` + `[semantics]`），但 srv/msg 的**路径解析规则不同**：

- **官方 TOML**（在 `rust/contracts/` 里）：`[io.srv] srv = "robonix_msg/srv/Foo"` → 去 `rust/crates/robonix-interfaces/lib/robonix_msg/srv/Foo.srv` 找
- **包内 TOML**（在 `capabilities/` 里）：`[io.srv] srv = "srv/Foo"` → 去**包的 `capabilities/srv/Foo.srv`** 找

典型的 skill 包 `capabilities/` 布局：

```
capabilities/
├── weird_thing.v1.toml
├── msg/
│   └── MyStructure.msg
└── srv/
    └── MyRequest.srv
```

`weird_thing.v1.toml`：
```toml
[contract]
id      = "robonix/skill/my_stack/weird_thing"
version = "1"
kind    = "skill"

[io.srv]
srv = "srv/MyRequest"    # 指向 capabilities/srv/MyRequest.srv（包内）

[mode]
type = "rpc"

[semantics]
user_invocable = true
```

`rbnx codegen` 会把包内的 `capabilities/msg/*.msg` 和 `capabilities/srv/*.srv` 也 codegen 到包的 `proto_gen/`，和官方接口一样导入使用。

### `rbnx codegen`

在包根目录跑：

```bash
rbnx codegen
```

它从 `capabilities:` 读每个 TOML 指向的 IDL / proto 路径，生成本地 `proto_gen/*_pb2.py` + `*_pb2_grpc.py`（Python）或对应 Rust stub；可选生成 `robonix_mcp_types/` — MCP tool 的类型绑定。包代码 `from proto_gen.xxx_pb2 import ...` 就能用。

`rbnx start` / `rbnx build` / `rbnx validate` 同理，在包内直接跑。

## 示例：一个 primitive 包（硬件驱动）

```
my_lidar_pkg/
├── package_manifest.yaml
├── scripts/build.sh        # 调 rbnx codegen 生成 proto_gen
├── bin/start.sh            # 启动驱动 + 注册到 atlas + 实现 driver RPC
└── src/
    └── driver.py
```

`package_manifest.yaml`:
```yaml
manifestVersion: 1
package:
  name: com.acme.primitive.sensor.lidar3d.xyz
  version: 0.1.0
  vendor: acme
  description: ACME XYZ-100 3D LiDAR driver
  license: Apache-2.0

build: bash scripts/build.sh
start: bash bin/start.sh

capabilities:
  - name: robonix/primitive/sensor/lidar3d           # 数据面：点云流
  - name: robonix/primitive/sensor/lidar3d/driver    # 生命周期：init/probe
```

`bin/start.sh`:
```bash
#!/usr/bin/env bash
set -eo pipefail
: "${RBNX_CONFIG_FILE:=/dev/null}"
IP=$(jq -r '.ip // "192.168.1.100"' < "$RBNX_CONFIG_FILE")
ros2 run acme_xyz_driver driver_node --ros-args -p ip:="$IP" &   # 驱动
python3 src/driver.py --register                                  # 注册 + 响应 driver RPC
wait
```

## 示例：一个 skill 包（agent 能力）

```
my_nav_skill/
├── package_manifest.yaml
├── capabilities/
│   ├── navigate_to_landmark.v1.toml  # 自定义 skill contract
│   └── srv/
│       └── NavigateToLandmark.srv    # 自己的 RPC 接口定义
├── scripts/build.sh
├── SKILL.md                          # agentskills.io 兼容
└── src/
    └── skill.py
```

`package_manifest.yaml`:
```yaml
manifestVersion: 1
package:
  name: com.acme.skill.navigate_to_landmark
  version: 0.1.0
  vendor: acme
  description: 给定地标名字去导航的高层 skill（调用 nav + vlm 描述目标）
  license: Apache-2.0

build: bash scripts/build.sh
start: |
  export PYTHONPATH="$(pwd)/proto_gen:${PYTHONPATH:-}"
  exec python -m src.skill

capabilities:
  # 自定义能力：本地 TOML 定义
  - name: robonix/skill/navigation/navigate_to_landmark
    path: capabilities/navigate_to_landmark.v1.toml

depends:
  - name: com.robonix.service.nav.nav2
  - name: com.robonix.system.vlm
```

`capabilities/navigate_to_landmark.v1.toml`:
```toml
# Navigate the robot to a named landmark understood by a VLM.
# Agent-facing skill — pilot dispatches this as an MCP tool call.
[contract]
id      = "robonix/skill/navigation/navigate_to_landmark"
version = "1"
kind    = "skill"

[io.srv]
srv = "srv/NavigateToLandmark"    # 相对 capabilities/ 的路径

[mode]
type = "rpc"

[semantics]
blocking = true
user_invocable = true
```

Deploy manifest 里引用：
```yaml
skill:
  - package: com.acme.skill.navigate_to_landmark
    path: ./skills/my_nav_skill
    name: nav_to_landmark
    config:
      vlm_model: claude-opus-4-7
      retry_on_ambiguous: true
```

## 后项兼容

老格式仍能加载（会打一次性 deprecation warning）：
- 包内文件名 `robonix_manifest.yaml` 当作 `package_manifest.yaml` 读
- `package.id` 回落为 `package.name`
- `build: { script: xxx }` 翻成 `build: bash xxx`
- `nodes: [{id, start}, ...]` 翻成一个 `start:`，各节点后台跑 + `wait`
- `capabilities[].definition` 等价于 `capabilities[].path`
