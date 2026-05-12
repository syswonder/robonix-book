# Robonix 包与部署配置规范

Robonix 部署分**两层** manifest：

| 文件 | 范围 | 谁读 |
|---|---|---|
| 部署根目录 `robonix_manifest.yaml` | 部署 | `rbnx boot` |
| 每个包内 `package_manifest.yaml` | 包 | `rbnx start` |

## 命令

```bash
# 整个栈一键起（读 deploy manifest，依次启动 system + 所有包）
rbnx boot -f robonix_manifest.yaml

# 只起单个包（开发调试）
rbnx start -p ./service/slam_fastlio2
```

`rbnx boot` 的流程：
1. 展开 `${VAR}` 环境变量
2. 起 `system:` 服务（atlas / executor / pilot / liaison / memory / vlm 等 cargo 二进制）
3. 对每个 `primitive` / `service` / `skill` 条目：把它的 `config` 块写到 `rbnx-boot/instances/<name>.json`，然后 `rbnx start -p <path>`，env 里带两个变量：`RBNX_CONFIG_FILE=<json-path>` 和 `RBNX_INSTANCE_NAME=<name>`
4. 日志落到 `rbnx-boot/logs/<component>.log`
5. Ctrl-C 统一 kill

> config 用文件传递（不是 env 里直接塞 JSON）——避开 bash 对引号/换行的 escape、`ARG_MAX` 限制、以及 `printenv | jq` 这种不直观的 debug 路径。包里一行 `jq` 就能读配置，同一个包的多个 instance（name 不同）各有自己的 json 文件，互不干扰。

## Deploy manifest 示例

实例参考：`examples/webots/robonix_manifest.yaml`。

```yaml
manifestVersion: 1
name: my-robot-deploy

env:
  ROS_DISTRO: humble

# system 服务（atlas / executor / pilot / memory / scene / speech / liaison /
# nexus）的 config 直接写在 key 下面。`atlas` `executor` `pilot` 是
# rbnx 自带的 Rust 二进制；其余是包。每个 key 的 config 是包自己消费的
# 任意字典，rbnx 把它 JSON 序列化后通过 Driver(CMD_INIT, config_json) 透传。
system:
  atlas:
    listen: 127.0.0.1:50051
    log: info
  executor:
    listen: 127.0.0.1:50061
    log: info
  pilot:
    listen: 127.0.0.1:50071
    log: info
    # vlm 不是独立 system 服务 —— 它是 pilot 的 upstream 配置，写在 pilot 里。
    vlm:
      upstream: ${VLM_BASE_URL}
      api_key:  ${VLM_API_KEY}
      model:    ${VLM_MODEL}
      api_format: openai
  memory:   { backend: sqlite, log: info }
  scene:    { log: info }
  speech:
    log: info
    disable_whisper: true             # 节省一份 Whisper-large 权重
    tts_voice: zh-CN-XiaoxiaoNeural

# 硬件。每个条目是一个 instance（设备）。
primitive:
  - name: tiago_chassis
    path: ./primitives/tiago_chassis
    config:
      odom_frame: odom
      base_frame: base_link

# 场景服务。path 是本地路径；url 是 git 地址（首次 clone 到 rbnx-boot/cache/）。
service:
  - name: mapping
    url: https://github.com/enkerewpo/mapping_rbnx
    branch: main
    config:
      algo: rtabmap
      sensors: { lidar2d: true, rgbd: true, odom: true }

  - name: simple_nav
    path: ./services/simple_nav
    config:
      robot_radius: 0.25
      max_linear: 0.5

skill:
  - name: explore
    url: https://github.com/enkerewpo/explore_rbnx
    branch: main
    config:
      explore_mode: frontier
      timeout_s: 600
```

### 条目字段

- **`name`**：instance 名字 / 日志前缀（不是包名）。同一个包用不同 `name` 可以起多份。
- **`path`** 或 **`url`**：二选一。`path` 相对 manifest 目录；`url` 是 git，首次 clone 到 `rbnx-boot/cache/<name>/`，可加 `branch:` 锁分支。
- **`config`**：任意 YAML 字典，rbnx 序列化为 JSON 后通过 `Driver(CMD_INIT, config_json)` 透传给包的 on_init 处理器。

## Package manifest 示例

```yaml
manifestVersion: 1

package:
  name: com.robonix.service.slam.fastlio2
  version: 0.1.0
  vendor: syswonder
  description: FASTLIO2 3D LiDAR-Inertial SLAM + PGO
  license: MulanPSL-2.0

build: bash scripts/build.sh     # build 入口
start: bash bin/start.sh         # start 入口

# 提供的能力。name 是 contract id，可选 path 指向包本地 TOML（当你自定义接口时）。
# 不写 path 就去 $(rbnx path capabilities) 找官方 TOML。
capabilities:
  - name: robonix/service/map/lio_odom
  - name: robonix/service/map/save_map
  - name: robonix/service/map/switch_mode

depends: # 库依赖，即需要用到另一个库的代码/数据（如model）
  - name: com.robonix.primitive.sensor.lidar3d.mid360
    path: ../primitive/sensor_lidar3d_mid360
  - name: com.robonix.system.xx
    url: https://github.com/syswonder/robonix-xx.git
    branch: v0.1
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

两条用同一个包、同一个 path，`name` 和 `config` 不同。rbnx 会分别 spawn 两个 `rbnx start`，每个拿到自己的 `RBNX_CAP_CONFIG_JSON`。包里要根据 config（比如 `topic_prefix`）决定发什么 topic、以什么 instance id 注册到 atlas（如 `robonix/primitive/lidar/lidar3d@front`）。

### Primitive 的 driver 生命周期

每个抽象硬件类别对应一个 driver contract（如 `robonix/primitive/lidar/lidar3d/driver`）。Driver 是普通的 RPC capability，按 `command` 字段区分四个操作 —— 同一组命令 service 和 skill 包也用，区别只在 rbnx 替哪一类自动发哪些。完整状态机见 [能力生命周期与状态机](../architecture/cap-lifecycle.md)。

Driver IDL（共享）：`rust/crates/robonix-interfaces/lib/lifecycle/srv/Driver.srv`：

```
uint8 CMD_INIT     = 0   # 解析 config_json、resolve atlas 上的依赖
uint8 CMD_SHUTDOWN = 1   # SIGTERM 之前的优雅退出（可选实现）
uint8 CMD_ACTIVATE       = 2   # 申请热资源、起线程、订阅 ROS、加载模型
uint8 CMD_DEACTIVATE     = 3   # 释放热资源；保留 atlas 注册（skill-only 才有意义）
uint8 command
string config_json       # 从 boot manifest 的 config: 块透传下来
---
bool ok
string state             # REGISTERED | INACTIVE | ACTIVE | ERROR | TERMINATED
string error
```

`rbnx boot` 对每个 primitive / service 自动发 `CMD_INIT` → `CMD_ACTIVATE`，到 ACTIVE 就常驻；对 skill 只发 `CMD_INIT`，停在 INACTIVE，`CMD_ACTIVATE` 由 executor 在第一次路由 MCP 调用时按需触发。`config_json` 永远是 manifest 的 `config:` 字段透传，包自己解析。

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
  - name: robonix/primitive/lidar/lidar3d
  - name: robonix/primitive/lidar/lidar3d/driver
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

- **官方 TOML**（在 robonix 源码仓库 `capabilities/` 里）：`[io.srv] srv = "robonix_msg/srv/Foo"` → 去 `rust/crates/robonix-interfaces/lib/robonix_msg/srv/Foo.srv` 找
- **包内 TOML**（在你本地 package 的 `capabilities/` 里）：`[io.srv] srv = "srv/Foo"` → 去**包的 `capabilities/srv/Foo.srv`** 找

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

`rbnx codegen` 会把包内的 `capabilities/msg/*.msg` 和 `capabilities/srv/*.srv` 也 codegen 到包的 `proto_gen/`，和官方接口一样导入使用。（TODO）

## Design invariants — what NOT to do in a package

These are not nice-to-haves. They are correctness constraints; violating
them silently breaks portability across robots / sims / hardware
revisions. CI should eventually enforce all of them.

### 1. Cross-package topic names go through atlas, never hardcoded

If your package consumes data produced by another package (e.g. a nav
service consuming a SLAM service's occupancy grid), look up the topic
via `QueryCapabilities + ConnectCapability` against the producer's
**contract**. Do not write the literal topic name as a constant or
default in your code.

The reason: the same nav service has to run unchanged on webots
(`/scanner_normalized`), a real Mid360 robot (`/mid360/scan`), and a
turtlebot (`/scan`). The instant any of those is hardcoded, the
package is no longer portable and the abstraction is fake.

Exceptions (allowed to be hardcoded):
- Topics WITHIN the same package (e.g. an internal sync queue).
- Topics declared by the SAME package's primitive layer for its own
  hardware fix-ups (see invariant #3).

### 2. Manifest = runtime declaration

Every capability listed in `package_manifest.yaml::capabilities` MUST
be DeclareCapability'd against atlas at startup. Aspirational entries
("we plan to implement save_map someday") rot — they make `rbnx caps`
output lie about what's actually available, and downstream consumers
that try to ConnectCapability fail at runtime instead of at deploy
parsing. If it's not implemented, it's not in the manifest.

### 3. Platform-specific compensation lives in the primitive that owns
the hardware, never in a generic service

Webots' lidar publishes a reversed-angle scan. URDF link names use
spaces ("Astra rgb") that don't match the message-stamped frame_id.
Wheel encoders dead-reckon during slip. **All such fix-ups belong in
the corresponding primitive package** (`tiago_lidar/scan_normalize.py`,
`tiago_camera`'s static TF bridge, etc.) — never in mapping or nav.

A generic service must never special-case "if running on webots, do
X". The package that owns the sensor / actuator declares clean,
spec-compliant data through its atlas contract; downstream services
trust the contract.

### 4. Algo-pluggable services expose a fixed contract surface

When a service supports multiple back-ends (mapping ships rtabmap +
dlio + fastlio2; nav will eventually ship simple_nav + nav2-wrapper),
EVERY back-end must declare the SAME set of contracts. Different
internal topic names are fine — the bridge maps each contract to
whichever topic the active algo exposes. If an algo can't natively
produce a contracted output, the launch file must spawn an adapter.

Adding a new contract to the surface is a versioning event for the
package — bump `package.version` and update every back-end. Don't add
"this contract only exists on algo X" — that violates the algo-agnostic
guarantee consumers depend on.
