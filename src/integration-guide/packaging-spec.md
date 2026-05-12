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
2. 起 `system:` 服务（atlas / executor / pilot / liaison 是 Rust 二进制；memory / scene / speech 是 Python 包）
3. 对每个 `primitive` / `service` / `skill` 条目：`rbnx start -p <path>` 拉起包进程，等它在 atlas 上注册完，然后调一次 gRPC `Driver(CMD_INIT, config_json=<manifest 里 config 块 JSON 化>)`。`on_init(cfg: dict)` 在包里接到这个 dict，没有 env、没有文件
4. 日志落到 `rbnx-boot/logs/<component>.log`
5. Ctrl-C 统一 kill

> config 通过 gRPC `Driver(CMD_INIT)` 的 `config_json` 字段透传——没有 env、没有文件、不依赖 bash 引号 escape。包代码只暴露一个 `@<provider>.on_init(cfg: dict)` 入口；同一个包的多个 instance（manifest 里 `name` 不同）各自拿到自己的 `cfg`，互不干扰。

## Deploy manifest 示例

实例参考：`examples/webots/robonix_manifest.yaml`。

```yaml
manifestVersion: 1
name: my-robot-deploy

env:
  ROS_DISTRO: humble

# system 服务（atlas / executor / pilot / liaison 是 Rust 二进制；
# memory / scene / speech 是 Python 包）的 config 直接写在 key 下面。
# 每个 key 的 config 是包自己消费的任意字典，rbnx 把它 JSON 序列化后
# 通过 Driver(CMD_INIT, config_json) 透传。
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

包代码用 `@<provider>.on_init` 注册一个 handler，框架会把 `Driver(CMD_INIT)` 带来的 `config_json` 解析成 `dict` 再调进来：

```python
from robonix_api import Service, Ok

mapping = Service(id="mapping", namespace="robonix/service/map")

@mapping.on_init
def init(cfg: dict):
    algo = cfg.get("algo", "rtabmap")
    sensors = cfg.get("sensors", {})
    # ...用 cfg 启服务...
    return Ok()
```

没有 env、没有配置文件、不需要 `jq`。同一个包的多个 instance 各自得到自己的 `cfg`。`rbnx start -p <path> -c local.yaml` 单包调试时，`-c` 的 YAML 也会走同一条路：序列化成 JSON → `Driver(CMD_INIT, config_json)` → `on_init(cfg)`。

### 多实例（同一个包跑多份）

一台车两个 MID360，或者同一个 camera 驱动挂两个摄像头：deploy manifest 里写两条，**path 相同 / name 不同 / config 不同**。

```yaml
primitive:
  - name: lidar_front
    path: ./primitives/mid360
    config: { ip: 192.168.1.161, mounted_frame: livox_front, topic_prefix: /lidar_front }
  - name: lidar_rear
    path: ./primitives/mid360
    config: { ip: 192.168.1.162, mounted_frame: livox_rear, topic_prefix: /lidar_rear }
```

`rbnx boot` 分别 spawn 两个 `rbnx start`，给两个 instance 各自下发对应 `config`。包代码在 `on_init(cfg)` 里按 `cfg["topic_prefix"]` 等决定发什么 topic、用什么 id 注册到 atlas（如 `Primitive(id="lidar_front", namespace="robonix/primitive/lidar")` vs `Primitive(id="lidar_rear", ...)`）——id 通常直接读 `cfg`，让两份实例分得清楚。

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

### 接口（contract）来源：官方 vs 包内

| 层 | contract 在哪 | 说明 |
|---|---|---|
| **primitive** | robonix 源码仓库的 `capabilities/primitive/` | 官方标准，接入新硬件按已有 contract 实现；缺接口提 PR 新增 |
| **service** | `capabilities/service/`（多数）+ 少量包内 | 场景服务大多复用官方 contract（mapping / navigation / scene 等），明确私有的才自定义 |
| **system** | `capabilities/system/` | 仓库内置（atlas / pilot / executor / liaison / memory / scene / speech），不外开 |
| **skill** | **全部在包内** | skill 是 agent 层，每个包自己定义，不进主仓库 |

### Primitive / Service 包：实现官方 contract

`capabilities:` 里只写 `name`，不写 `path`：

```yaml
capabilities:
  - name: robonix/primitive/lidar/lidar
  - name: robonix/primitive/lidar/driver
```

`rbnx codegen` 去 `<robonix-repo>/capabilities/primitive/lidar/lidar.v1.toml` 查接口形状，代码按 TOML 里指向的 ROS IDL 实现。下游消费者只看 contract id 就能对接。

### Skill 包：contract 写在包里

skill 的接口是 agent 层面的，每个应用都不一样，**没有官方标准**。把 TOML 放包内 `capabilities/`，`capabilities:` 里用 `path` 指：

```yaml
capabilities:
  - name: robonix/skill/my_stack/weird_thing
    path: capabilities/weird_thing.v1.toml
```

TOML 字段格式跟官方 contract 一致（`[contract]` + `[io.srv]` / `[io.msg]` + `[mode]`），但 IDL 的**路径解析规则不同**：

- **官方 TOML**（在 robonix 源码仓库 `capabilities/` 里）：`[io.srv] srv = "lidar/srv/Foo"` → 去 `rust/crates/robonix-interfaces/lib/lidar/srv/Foo.srv` 找
- **包内 TOML**（在自己 package 的 `capabilities/` 里）：`[io.srv] srv = "srv/Foo"` → 去**包的 `capabilities/srv/Foo.srv`** 找

典型 skill 包 `capabilities/` 布局：

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
srv = "srv/MyRequest"    # 指向包内 capabilities/srv/MyRequest.srv

[mode]
type = "rpc"
```

`rbnx codegen` 会把包内的 `capabilities/msg/*.msg` 和 `capabilities/srv/*.srv` 一起 codegen 到包的 `rbnx-build/codegen/`，跟引用官方 contract 时一样 import 使用。

## 设计不变量 — 包里**不能**做的事

下面这些不是"建议"，是**正确性约束**。违反任意一条都会让包失去跨机器人 / 跨仿真 / 跨硬件版本的可移植性，未来 CI 会强制检查。

### 1. 跨包 topic 名走 atlas，绝不硬编码

如果你的包消费另一个包产出的数据（如导航服务订阅 SLAM 服务的占据栅格），用 `ATLAS.find_capability` + `connect_capability` 按对方 **contract** 查 endpoint。**不要**把字面 topic 名写在代码里当常量或 default。

理由：同一个 nav service 要不改一行就跑在 webots（`/scanner_normalized`）、Mid360 真机（`/mid360/scan`）、turtlebot（`/scan`）上。任何一个 topic 硬编码，跨机就崩，抽象就假。

允许硬编码的例外：
- 包内自用的 topic（如内部同步队列）
- 同一个 primitive 包内的 hardware fix-up（见不变量 §3）

### 2. manifest 即运行时声明

`package_manifest.yaml::capabilities` 列出的每条 contract **必须**在启动时真的 `DeclareCapability` 上去。占位条目（"以后实现 save_map"）会腐烂——让 `rbnx caps` 输出撒谎，下游 `ConnectCapability` 时才在 runtime 炸而不是部署解析时早暴露。**没实现，就不写进 manifest。**

### 3. 平台相关补丁住在 primitive 包里，不能进通用服务

Webots 的 lidar 发反向角度的 scan、URDF link 名带空格（"Astra rgb"）跟 frame_id 不一致、轮编码器打滑时漂移——所有这类 fix-up **必须**在对应的 primitive 包里搞定（`tiago_lidar/scan_normalize.py`、`tiago_camera` 的静态 TF bridge 等），**不要**进 mapping / nav。

通用服务绝不能写 "if 跑在 webots 上 do X" 这种代码。primitive 包通过自己的 atlas contract 输出干净的、合规的数据；下游服务信 contract。

### 4. 多算法 service 共享同一组 contract

一个 service 支持多后端时（mapping 同时有 rtabmap / dlio / fastlio2；nav 未来会有 simple_nav / nav2-wrapper），**每个后端必须声明同一组 contract**。内部 topic 名可以不同——bridge 负责把 contract 映射到当前算法实际发的 topic。算法天生产不出某条契约输出时，launch 文件起一个 adapter 补上。

往 contract 集合里加新条目是**包的版本事件**——bump `package.version` + 更新所有后端。**不要**写 "这条 contract 只在算法 X 上有"——这破坏了消费者赖以为生的算法无关担保。
