# Robonix 开发者指南

本指南介绍如何开发 Robonix 软件包（Package）。一个软件包可以实现原语（Primitive）、服务（Service）或技能（Skill），通过标准能力约定接入 Atlas，并由机器人部署清单选择和配置。

章节编号保持稳定，便于其他页面和外部资料长期引用。

## 1. 5 分钟上手

最快的学习入口是可独立启动的 [`template-rbnx`](https://github.com/syswonder/template-rbnx)。它包含一个模拟底盘原语、一个导航服务和一个问候技能，不需要真实机器人或仿真器。

先按 [Webots 快速上手](getting-started/quickstart.md#1-检查主机)安装 Git、Rust、Python、Docker 和 Robonix，再执行：

```bash
git clone https://github.com/syswonder/template-rbnx.git
cd template-rbnx

cp .env.example .env
# 编辑 .env，填写可用的 VLM_BASE_URL、VLM_API_KEY 和 VLM_MODEL。
set -a
source .env
set +a

rbnx build
rbnx boot
```

在另一个终端检查运行状态并提交任务：

```bash
rbnx caps -v
rbnx chat
```

在交互界面输入 `Say hello to Alice.`。系统应首次激活 `say_hello` 技能并返回问候语。退出交互界面后，在部署终端按 `Ctrl+C`；也可以从另一个终端执行：

```bash
rbnx shutdown
```

:::tip 从模板开始修改

- 替换 `primitives/mock_chassis/` 中的模拟驱动，可以接入真实底盘。
- 替换 `services/my_navigate/` 中的占位规划逻辑，可以接入 Nav2 或其他导航器。
- 复制 `skills/say_hello/`，可以建立带自定义能力约定的技能。

:::

## 2. Robonix 是什么

Robonix 为具身智能模型提供统一的运行时。硬件驱动以原语暴露设备能力；建图、导航、语音等算法以服务暴露可复用功能；技能封装具有任务语义的行为。Atlas 维护能力目录，Pilot 把用户意图转换为执行方案，Executor 调用方案中的能力。

运行时围绕四个对象组织：

| 对象 | 作用 | 示例 |
|---|---|---|
| 软件包 | 描述一份可构建、可启动的交付物 | `primitive-realsense-camera-rbnx` |
| 能力提供者（provider） | 软件包启动后注册到 Atlas 的运行实例 | `front_camera` |
| 能力约定（contract） | 定义接口名称、数据类型和交互模式 | `robonix/primitive/camera/rgb` |
| 能力（Capability） | 某个提供者对某条能力约定的运行时实现 | `front_camera` 通过 ROS 2 发布 RGB 图像 |

能力目录属于控制面。真实图像、点云、RPC 请求或工具调用通过 ROS 2、gRPC 或模型上下文协议（Model Context Protocol，MCP）直接传输，不经过 Atlas 转发。

## 3. 原语、服务与技能

### 3.1 三类能力提供者

| 类型 | 责任 | 典型内容 |
|---|---|---|
| 原语 | 把设备和硬件状态转换为标准接口 | 底盘、机械臂、相机、雷达、音频、设备健康 |
| 服务 | 组合数据或算法，提供场景级功能 | 建图、定位、导航、语音、记忆 |
| 技能 | 暴露可由 Pilot 选择的任务级行为 | 探索、抓取、问候 |

Python 中分别使用 `Primitive`、`Service` 和 `Skill`。三者共享能力声明、生命周期和连接接口；区别主要体现在向 Atlas 注册的类型，以及 Executor 对技能的延迟激活策略。

原语命名空间不是封闭枚举。Robonix 当前提供机械臂、音频、相机、底盘、设备健康、IMU、激光雷达和机器人描述等标准能力约定；新硬件类型可以提交新的标准约定。

### 3.2 运行时身份

部署清单中实例的 `name` 必须等于代码构造能力提供者时的 `id`：

```python
camera = Primitive(
    id="front_camera",
    namespace="robonix/primitive/camera",
)
```

```yaml
primitive:
  - name: front_camera
    url: https://github.com/example/primitive-camera-rbnx
```

同一机器人可以启动多个实现相同接口的设备，例如 `front_camera` 和 `arm_camera`。调用方按 `provider_id` 选择实例，不需要为每台相机发明不同的能力约定 ID。

### 3.3 能力寻址

同一提供者可以用不同传输实现同一能力约定，因此运行时能力键包含：

```text
(provider_id, contract_id, transport)
```

Atlas 查询返回能力元数据。调用 `connect_capability(...)` 后得到的通道（Channel）才包含本次连接应使用的端点。

### 3.4 能力目录（Atlas）

Atlas 默认监听 `127.0.0.1:50051`，也可以由部署配置或 `ROBONIX_ATLAS` 修改。能力提供者启动后先注册自身，再声明能力；消费者可通过 `ATLAS.find_capability(...)` 搜索并建立连接。详细流程见 [Atlas 能力目录](architecture/atlas.md)。

## 4. 能力约定

### 4.1 描述文件与 IDL

能力约定由 TOML 描述文件和 ROS 接口定义语言（Interface Definition Language，IDL）共同组成：

```toml
[contract]
id = "robonix/service/example/hello"
version = "1"
kind = "service"
idl = "hello/srv/Hello.srv"

[mode]
type = "rpc"
```

```text title="capabilities/lib/hello/srv/Hello.srv"
string name
---
string message
```

`kind` 是能力约定元数据，不决定提供者在 Atlas 中注册成原语、服务还是技能；提供者类型由 `Primitive`、`Service` 或 `Skill` 决定。

### 4.2 模式与传输

模式（mode）描述接口语义和数据流方向；传输（transport）描述运行时通信技术。

| 模式 | 语义 | 常见传输 |
|---|---|---|
| `rpc` | 一次请求、一次响应 | gRPC、MCP、ROS 2 service |
| `rpc_server_stream` | 一次请求、连续响应 | gRPC |
| `rpc_client_stream` | 连续请求、一次响应 | gRPC |
| `rpc_bidi_stream` | 双向连续消息 | gRPC |
| `topic_out` | 提供者持续发布 | ROS 2 topic |
| `topic_in` | 提供者持续接收 | ROS 2 topic |

提供者必须显式选择传输。当前 Python 运行时不会在装饰器注册阶段完整校验“模式—传输”矩阵，因此开发者仍需遵守能力约定，并通过端到端测试确认生成代码和消费者一致。

### 4.3 全局与软件包内能力约定

标准能力约定位于 Robonix 源码的 `capabilities/`。只有软件包独有、尚未成为标准接口的能力才放到自身 `capabilities/`。软件包内 TOML 会参与合并；不要无意使用与全局约定相同的 ID 覆盖它。IDL 通过相对路径和类型名关联，不应依赖同路径文件覆盖全局 IDL。

## 5. 生命周期

### 5.1 状态

能力提供者使用以下状态。图中的蓝色路径是正常生命周期，红色路径表示命令失败或运行时异常，灰色路径表示关闭或心跳超时：

![能力提供者生命周期状态机](./cap-lifecycle.png)

| 状态 | 含义 |
|---|---|
| `REGISTERED` | 已向 Atlas 注册，但尚未完成初始化 |
| `INACTIVE` | 配置已解析，尚未占用热运行资源 |
| `ACTIVE` | 可以处理业务请求或数据流 |
| `ERROR` | 初始化或运行失败 |
| `TERMINATED` | 正在退出或已经退出 |

### 5.2 生命周期处理函数

```python
from robonix_api import Service, Ok

service = Service(id="my_service", namespace="robonix/service/example")

@service.on_init
def init(cfg: dict):
    return Ok()

@service.on_activate
def activate():
    return Ok()

@service.on_deactivate
def deactivate():
    return Ok()

@service.on_shutdown
def shutdown():
    pass
```

需要接收 `CMD_INIT`、`CMD_ACTIVATE` 或延迟激活的提供者必须有可用的 `<namespace>/driver` 能力约定。标准命名空间通常复用主仓库中的 driver 约定；自定义命名空间在没有全局约定时需要软件包内 driver TOML。没有 driver 的提供者在监听就绪后会自动进入 `ACTIVE`，但不会收到生命周期配置。

每个提供者应只声明一个 driver。当前调用方选择第一条以 `/driver` 结尾的能力，并不会替开发者消除重复声明。

原语和服务通常由启动流程初始化并激活。技能启动后保持 `INACTIVE`，Executor 第一次调用前发送激活命令；当前没有自动的空闲回收策略，技能会保持 `ACTIVE`，直到显式停用或系统关闭。

## 6. 软件包结构

### 6.1 生成骨架

在部署仓库中创建软件包：

```bash
rbnx package-new my_camera --type primitive
rbnx package-new my_mapper --type service
rbnx package-new my_skill --type skill
```

命令分别写入 `primitives/`、`services/` 和 `skills/`。也可以指定准确目录：

```bash
rbnx package-new my_camera --type primitive --path ./packages/my_camera
```

生成结构为：

```text
my_camera/
├── package_manifest.yaml
├── capabilities/
├── scripts/
│   ├── build.sh
│   └── start.sh
└── my_camera/
    ├── __init__.py
    └── main.py
```

当前脚手架会生成清单、脚本、Python 模块和空的 `capabilities/`。开发者还应在软件包根目录添加 `config.spec`，记录部署方可配置的字段、类型、单位、默认值和约束。

脚手架中的 `main.py` 会生成一个 `@provider.on_init` 处理函数，但空的 `capabilities/` 不包含 `<namespace>/driver` 能力约定。按当前运行时行为，这个处理函数不会被调用，提供者会在监听就绪后直接进入 `ACTIVE`，部署 `config` 也不会送达。若软件包不需要生命周期配置，可以删除该处理函数；若需要配置或 INIT/ACTIVATE 阶段，应添加 `<namespace>/driver` TOML（可复用 `lifecycle/srv/Driver.srv`），重新运行代码生成，并保留 `on_init`。

### 6.2 软件包清单

```yaml
manifestVersion: 1

package:
  name: robonix.service.example.my_service
  version: 0.1.0
  description: Example Robonix service.
  license: Apache-2.0
  tags: [service, example]
  maintainers:
    - Your Name <you@example.com>

build: bash scripts/build.sh
start: bash scripts/start.sh
# 可选；需要在关闭时执行软件包自有清理命令时填写。
# stop: bash scripts/stop.sh

capabilities:
  - name: robonix/service/example/hello
    path: capabilities/hello.v1.toml

depends:
  - name: robonix.lib.example
    path: ../example-lib
```

运行时要求 `package.name`、`version`、`description`、`license` 和 `start` 非空。`tags` 与 `maintainers` 对目录发布很重要，但当前运行时不把它们当成必填字段，也不解析版本是否严格符合语义化版本。

`stop` 是可选的 shell 命令。关闭顺序为 Driver `CMD_SHUTDOWN`、`stop`、进程组 TERM/KILL；命令在软件包根目录执行。

`capabilities` 是软件包元数据和预期导出清单。每项必须有 `name`，可用 `path` 指向相对软件包根目录的自定义 TOML；旧字段名 `definition` 仍作为 `path` 的别名读取。它不会替代码注册能力，也不控制代码生成；实际声明来自 `@provider.mcp`、`@provider.grpc`、`create_publisher` 等 API。

`depends` 每项使用 `name`，并可带 `path`、`url` 和 `branch`。这些字段当前只被解析为依赖元数据，不会自动克隆、安装依赖或修改 `PYTHONPATH`；构建脚本仍需自行处理依赖。

### 6.3 构建与启动脚本

纯 Python MCP 软件包的最小构建脚本：

```bash title="scripts/build.sh"
#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
rbnx codegen -p "$PKG" --mcp
```

启动脚本：

```bash title="scripts/start.sh"
#!/usr/bin/env bash
set -euo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"
export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"
exec python3 -m my_service.main
```

更多构建方式见 [构建与代码生成](integration-guide/build-and-codegen.md)。

## 7. Python 接口（API）速览

```python
from robonix_api import ATLAS, Service, Ok, Err, Deferred
from robonix_api.atlas_types import Transport

service = Service(id="my_service", namespace="robonix/service/example")

@service.on_init
def init(cfg: dict):
    return Ok()

@service.mcp("robonix/service/example/hello")
def hello(req: Hello_Request) -> Hello_Response:
    """Return a greeting for the requested name."""
    return Hello_Response(message=f"Hello, {req.name}!")

candidates = ATLAS.find_capability(
    contract_id="robonix/primitive/chassis/move",
    transport=Transport.GRPC,
)

service.run()
```

常用返回值：

- `Ok()`：生命周期动作成功；
- `Err("reason")`：不可恢复错误；
- `Deferred("reason")`：依赖尚未就绪，可稍后重试。

处理函数必须返回对应的有效类型。抛出的异常会转换为错误；漏写返回值或返回错误类型会触发 `TypeError`，并使 Driver RPC 失败。

## 8. 开发服务

下面的服务复用标准导航能力约定，因此不需要复制 driver TOML。

```bash
rbnx package-new my_navigate --type service
```

将 `scripts/build.sh` 中的代码生成命令改为 `rbnx codegen -p "$PKG" --mcp`，然后实现：

```python title="services/my_navigate/my_navigate/main.py"
from __future__ import annotations

import uuid

from robonix_api import Service, Ok
from navigation_mcp import Navigate_Request, Navigate_Response

navigate_service = Service(
    id="my_navigate",
    namespace="robonix/service/navigation",
)

@navigate_service.on_init
def init(cfg: dict):
    return Ok()

@navigate_service.mcp("robonix/service/navigation/navigate")
def navigate(req: Navigate_Request) -> Navigate_Response:
    """Accept a map-frame navigation goal."""
    run_id = str(uuid.uuid4())
    # 在这里把 req.goal 交给真正的规划器。
    return Navigate_Response(
        accepted=True,
        run_id=run_id,
        detail="goal accepted",
    )

if __name__ == "__main__":
    navigate_service.run()
```

在部署清单中加入：

```yaml
service:
  - name: my_navigate
    path: ./services/my_navigate
    config: {}
```

`name` 必须与 `Service(id="my_navigate", ...)` 一致。服务若需要底盘，应在 `on_activate` 中发现并连接目标提供者：

```python
from robonix_api import ATLAS, Deferred
from robonix_api.atlas_types import Transport

move_channel = None

@navigate_service.on_activate
def activate():
    global move_channel
    matches = ATLAS.find_capability(
        contract_id="robonix/primitive/chassis/move",
        transport=Transport.GRPC,
        provider_id="base_chassis",
    )
    if not matches:
        return Deferred("base_chassis is not ready")
    move_channel = navigate_service.connect_capability(
        matches[0],
        "robonix/primitive/chassis/move",
        Transport.GRPC,
    )
    return Ok()
```

保存通道对象，直到停用或关闭；不要在 `with Channel` 块退出后继续使用其端点。

## 9. 开发原语

原语负责把厂商 SDK、总线或 ROS 2 驱动适配为标准接口。以底盘的一次性移动命令为例：

```python
import json

from robonix_api import Primitive, Ok
import chassis_pb2
import std_msgs_pb2

chassis = Primitive(id="base_chassis", namespace="robonix/primitive/chassis")

@chassis.on_init
def init(cfg: dict):
    return Ok()

@chassis.grpc("robonix/primitive/chassis/move")
def move(req, _ctx):
    linear_x = req.command.linear_x
    angular_z = req.command.angular_z
    # 将速度传给厂商 SDK，并在实现中施加限速、超时和看门狗。
    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(
            data=json.dumps({"accepted": True, "linear_x": linear_x, "angular_z": angular_z})
        )
    )

if __name__ == "__main__":
    chassis.run()
```

ROS 2 话题可以通过便捷层创建并自动向 Atlas 声明：

```python
from nav_msgs.msg import Odometry

chassis.create_publisher(
    contract_id="robonix/primitive/chassis/odom",
    topic="/odom",
    msg_type=Odometry,
    qos="best_effort",
)

chassis.emit("robonix/primitive/chassis/odom", odom_message)
```

支持的字符串 QoS 为 `best_effort`、`reliable` 和 `latched`。也可传整数队列深度或原生 `QoSProfile`。原语既可以使用这些便捷接口，也可以直接使用 `rclpy`；无论哪种方式，都必须保证实际话题类型、方向和声明一致。

## 10. 开发技能

技能通常使用 `robonix/skill/<name>` 命名空间，使 Executor 能识别并在第一次调用前激活。自定义技能接口需要软件包内 IDL 和能力约定：

```text
skills/say_hello/capabilities/
├── driver.v1.toml
├── say.v1.toml
└── lib/say_hello/srv/SayHello.srv
```

```toml title="capabilities/driver.v1.toml"
[contract]
id = "robonix/skill/say_hello/driver"
version = "1"
kind = "skill"
idl = "lifecycle/srv/Driver.srv"

[mode]
type = "rpc"
```

```toml title="capabilities/say.v1.toml"
[contract]
id = "robonix/skill/say_hello/say"
version = "1"
kind = "skill"
idl = "say_hello/srv/SayHello.srv"

[mode]
type = "rpc"
```

```text title="capabilities/lib/say_hello/srv/SayHello.srv"
string name
---
string greeting
```

实现技能：

```python
from robonix_api import Skill, Ok
from say_hello_mcp import SayHello_Request, SayHello_Response

skill = Skill(id="say_hello", namespace="robonix/skill/say_hello")
active = False

@skill.on_init
def init(cfg: dict):
    return Ok()

@skill.on_activate
def activate():
    global active
    active = True
    return Ok()

@skill.on_deactivate
def deactivate():
    global active
    active = False
    return Ok()

@skill.mcp("robonix/skill/say_hello/say")
def say(req: SayHello_Request) -> SayHello_Response:
    """Greet the named person."""
    return SayHello_Response(greeting=f"Hello, {req.name}!")

if __name__ == "__main__":
    skill.run()
```

MCP 处理函数必须恰好接收一个参数，并为输入和返回值提供代码生成类型注解。显式 `description=` 优先作为 Atlas 中供 Pilot 使用的描述；未传时使用函数文档字符串。

## 11. 部署目录

部署仓库负责选择软件包实例和机器人专属配置：

```text
robot-acme-rover/
├── robonix_manifest.yaml
├── soma.yaml
├── urdf/
│   └── acme_rover.urdf
├── config/
├── primitives/
├── services/
└── skills/
```

`primitives/`、`services/` 和 `skills/` 是推荐布局，不是运行时强制目录。部署清单中的 `path` 可以指向仓库内其他位置；可复用软件包通常使用 `url` 引用独立仓库。

创建部署骨架：

```bash
rbnx init robot-acme-rover
cd robot-acme-rover
mkdir -p config primitives services skills urdf assets
```

`rbnx init` 当前只创建部署目录、`robonix_manifest.yaml` 和 `.gitignore`，其余目录需要按本体需要建立。

## 12. 部署清单

```yaml
manifestVersion: 1
name: robot-acme-rover

catalog:
  name: robonix.robot.acme.rover
  version: 0.1.0
  description: Robonix deployment for ACME Rover.
  license: Apache-2.0
  tags: [robot, deploy, acme, rover]
  maintainers:
    - Your Name <you@example.com>

system:
  atlas: { listen: 127.0.0.1:50051, log: info }
  executor: { listen: 127.0.0.1:50061, log: info }
  soma:
    robot_yaml: soma.yaml
  pilot:
    listen: 127.0.0.1:50071
    log: info
    vlm:
      upstream: ${VLM_BASE_URL}
      api_key: ${VLM_API_KEY}
      model: ${VLM_MODEL}
      api_format: openai
  liaison: { listen: 0.0.0.0:50081, log: info }

primitive:
  - name: base_chassis
    url: https://github.com/example/primitive-chassis-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config:
      can_interface: can_chassis

service:
  - name: my_navigate
    path: ./services/my_navigate
    config: {}

skill:
  - name: say_hello
    path: ./skills/say_hello
    config: {}
```

`system` 中 Atlas、Executor、Soma、Vitals、Pilot 和 Liaison 等内置组件由 `rbnx` 转换为各自命令行参数。非内置系统软件包、原语、服务和技能的 `config` 会被物化为实例配置文件供调试和事后检查；`rbnx boot` 不把该文件传给 `rbnx start`，而是在提供者注册后通过 Driver `CMD_INIT` 的 `config_json` 发送配置。没有 driver 的提供者不会收到这份配置。

远程软件包使用 `url`；本地软件包使用 `path`。`manifest` 用于选择该软件包提供的特定平台清单，例如 Jetson 原生构建。当前 `rbnx build -f` 在应用选择器前仍要求软件包根目录存在默认 `package_manifest.yaml`，因此提供平台变体时也应保留默认清单。完整字段见 [包与部署清单规范](integration-guide/packaging-spec.md)。

## 13. 构建、启动与停止

在部署目录执行：

```bash
rbnx build
rbnx boot
```

当前阶段顺序为：

1. 启动 Atlas、Executor 和 Soma；
2. Soma 启动原语；
3. 启动清单中声明的其余内置组件（如 Vitals、Pilot、Liaison）、额外系统软件包和服务；
4. Soma 启动技能，技能停在 `INACTIVE`。

各列表内部遵循部署清单顺序。非内置系统软件包或服务的单项失败会被汇总，启动流程可能继续检查其余软件包；不要把“出现启动摘要”当成全部成功，应检查 `failures` 和日志。

常用诊断：

```bash
rbnx caps -v
rbnx tools
rbnx channels
rbnx inspect
rbnx logs --list-tags
rbnx logs --tag my_navigate --follow
```

停止当前部署：

```bash
rbnx shutdown
```

该命令先读取并解析原部署清单，以便为停止钩子恢复同一组顶层 `env`，再读取 `rbnx-boot/state.json`，停止对应启动实例的进程组和容器，并执行软件包停止钩子。因此原部署清单必须仍然存在且可解析。

## 14. Python 接口

### 14.1 构造能力提供者

```python
Primitive(id, namespace, *, pkg_root=None, md_path=None)
Service(id, namespace, *, pkg_root=None, md_path=None)
Skill(id, namespace, *, pkg_root=None, md_path=None)
```

`id` 与 `namespace` 当前是公开属性。`pkg_root` 和 `md_path` 用于覆盖软件包根目录与 `CAPABILITY.md` 路径，通常让运行时自动发现即可。

### 14.2 查询能力目录

```python
ATLAS.find_capability(
    contract_id="robonix/primitive/camera/rgb",
    transport=Transport.ROS2,
    provider_kind="primitive",
    provider_id="front_camera",
    namespace_prefix="robonix/primitive/camera",
)
```

返回 `list[Capability]`。需要恰好一个结果时可使用 `find_unique_capability(...)`；零个或多个结果都会抛出错误。

### 14.3 连接能力

```python
channel = service.connect_capability(
    capability_or_provider,
    contract_id,
    transport,
)
```

第一个参数可以是 `Capability` 或 `CapabilityProvider`。返回的 `Channel` 包含 `endpoint` 和传输参数，并在 Atlas 中记录消费者到提供者的连接。`Capability.description` 保存自然语言描述；`McpParams` 只包含输入结构 JSON。

### 14.4 ROS 2 便捷接口

```python
provider.create_publisher(
    contract_id=...,
    topic=...,
    msg_type=...,
    qos="best_effort",
)
provider.create_subscription(
    contract_id=...,
    topic=...,
    msg_type=...,
    callback=...,
    qos="reliable",
)
provider.emit(contract_id, message)
```

`declare_ros2_topic` 和 `declare_ros2_service` 只向 Atlas 声明端点；它们不会替开发者创建 `rclpy` 发布者、订阅者或服务。

### 14.5 MCP 与 gRPC

```python
@provider.mcp(contract_id, description="...")
def tool(req: RequestType) -> ResponseType:
    ...

@provider.grpc(contract_id, description="...")
def method(req, context):
    ...
```

MCP 使用代码生成的数据类；gRPC 使用 `*_pb2.py` 消息和 `robonix_contracts_pb2_grpc.py` 中的服务定义。不要从业务消息模块导入生成的 Servicer 或 Stub。

### 14.6 关闭行为

`provider.run()` 完成注册、能力声明、服务监听、心跳和信号处理。退出时运行时把提供者状态设为 `TERMINATED`，关闭通道、受管理子进程和 gRPC 服务；当前 `_teardown()` 不会显式停止进程级 ROS 后端或 FastMCP 的 uvicorn 线程，也不会调用 Atlas unregister。通常由随后发生的进程退出回收这些资源，Atlas 通过状态和心跳回收记录。

## 15. 常用命令行接口

| 命令 | 用途 |
|---|---|
| `rbnx setup <root>` | 登记 Robonix 源码根目录 |
| `rbnx path <key>` | 输出源码树中的绝对路径 |
| `rbnx init <name>` | 创建机器人部署骨架 |
| `rbnx package-new <name> --type <type>` | 创建软件包骨架 |
| `rbnx validate [path]` | 校验软件包清单 |
| `rbnx codegen -p <package> [--mcp] [--ros2]` | 生成 gRPC、MCP 或 ROS 2 接口代码 |
| `rbnx build [-p <package> \| -f <manifest>]` | 构建软件包或整个部署 |
| `rbnx start [-p <package>]` | 单独启动一个软件包；不会自动发送完整部署的激活序列 |
| `rbnx boot [-f <manifest>]` | 启动整套部署 |
| `rbnx shutdown [-f <manifest>]` | 停止对应部署 |
| `rbnx update [-p <package> \| -f <manifest>]` | 更新远程软件包 |
| `rbnx clean [-p <package> \| -f <manifest>] [--cache]` | 清理构建和运行产物 |
| `rbnx caps [-v]` | 查看提供者和能力 |
| `rbnx contracts [-v]` | 查看 Atlas 已加载的能力约定 |
| `rbnx describe --provider <id>` | 查看提供者的 `CAPABILITY.md` |
| `rbnx tools` | 查看 Pilot 可调用工具 |
| `rbnx channels` | 查看活动连接 |
| `rbnx inspect` | 输出完整运行时状态 |
| `rbnx ask "<prompt>"` | 非交互提交一次任务 |
| `rbnx chat` | 启动交互界面 |
| `rbnx logs` | 读取结构化日志 |

`rbnx clean -f robonix_manifest.yaml` 默认保留 `rbnx-boot/cache/`；只有加 `--cache` 才删除远程软件包缓存。`rbnx start --config` 仍会读取软件包清单；任何“跳过部署清单”的选项都不等于跳过软件包清单。

## 16. 配置字段

### 16.1 软件包清单

| 字段 | 当前作用 |
|---|---|
| `manifestVersion` | 当前接受大于等于 1 的版本；新清单使用 1 |
| `package.name` | 软件包发布名，必填 |
| `package.version` | 版本字符串，必填；当前不强制解析语义化版本 |
| `package.description` | 软件包说明，必填 |
| `package.license` | 许可证字符串，必填；约定使用 SPDX 标识，当前只校验非空 |
| `package.tags` | 目录分类，可选 |
| `package.maintainers` | 维护者列表，可选 |
| `build` | 构建命令，可选；省略表示无构建步骤 |
| `start` | 启动命令，必填 |
| `stop` | 停止命令，可选；在 Driver `CMD_SHUTDOWN` 后、终止进程组前执行 |
| `capabilities` | 预期能力元数据；每项含 `name`，可选 `path`，不执行运行时声明 |
| `depends` | 依赖元数据；每项含 `name`，可选 `path`、`url`、`branch`，当前不自动获取或安装 |

底层解析器仍可读取旧 `vendor`、`id`、`nodes`、旧式构建对象和旧文件名。`id` 回退、`nodes`、旧式构建对象或旧文件名会产生迁移提示，单独出现的 `vendor` 不会。兼容范围不是所有命令都一致：显式 `rbnx start -p <目录>` 可读取旧 `robonix_manifest.yaml`，但当前 `rbnx build -p`、从当前目录自动发现软件包以及 `rbnx build -f` 的部署构建仍要求 `package_manifest.yaml`。新软件包只使用当前字段。

### 16.2 部署清单

| 字段 | 当前作用 |
|---|---|
| `manifestVersion` | 文档中的部署格式标记；当前 `rbnx` 和 Soma 不读取或校验该字段 |
| `name` | 本次部署名称 |
| `catalog` | 整机目录发布元数据；运行时忽略 |
| `env` | 在解析其余标量前设置环境变量 |
| `system` | 内置系统配置与额外系统软件包 |
| `primitive`、`service`、`skill` | 软件包实例列表 |
| 实例 `name` | 运行时提供者 ID，必须与代码一致 |
| 实例 `path` / `url` | 本地路径或远程仓库 |
| 实例 `branch` | 远程仓库分支或标签；当前通过 `git clone --branch` 实现，不支持任意提交 SHA |
| 实例 `manifest` | 选择软件包内的目标清单 |
| 实例 `config` | 透传给该实例的配置映射 |

部署清单的相对路径以部署文件所在目录为基准。软件包如何解释 `config` 由其 `config.spec` 和实现共同定义。

### 16.3 能力约定 TOML

| 字段 | 含义 |
|---|---|
| `contract.id` | 合并后能力约定目录中的接口名 |
| `contract.version` | 能力约定版本 |
| `contract.kind` | 文档和代码生成使用的分类元数据 |
| `contract.idl` | 相对 IDL 根目录的数据类型路径 |
| `contract.cross_namespace` | 允许共享接口跨提供者命名空间而不提示 |
| `mode.type` | RPC 或话题方向 |

同一 ID 在有效合并目录中只保留一个描述；后加载根可能覆盖先加载描述。软件包应复用标准能力约定，只有明确需要时才新增自定义 ID，并通过 `rbnx contracts -v` 检查最终来源。
