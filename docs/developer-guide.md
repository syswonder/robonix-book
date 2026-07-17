# Robonix 开发者指南

本指南介绍如何开发 Robonix 软件包（Package）。一个软件包可以实现原语（Primitive）、服务（Service）或技能（Skill），依据标准能力约定实现接口，在运行时向 Atlas 注册能力，再由机器人部署清单选择和配置。

首次开发先完成第 1 节，再阅读第 2—7 节；随后按软件包类型选择第 8、9 或 10 节，最后按第 11—13 节部署。第 14—16 节供查阅。

## 1. 5 分钟上手

最快的学习入口是可独立启动的 [`template-rbnx`](https://github.com/syswonder/template-rbnx/tree/60dc85834c2714022b1821e6fce6c629c0314699)。它包含一个模拟底盘原语、一个导航服务和一个问候技能，不需要真实机器人或仿真器。

先按 [Webots 快速上手](getting-started/quickstart.md#1-检查主机)安装 Git、Rust、Python、Docker 和 Robonix，再执行：

```bash
git clone https://github.com/syswonder/template-rbnx.git
cd template-rbnx
git switch --detach 60dc85834c2714022b1821e6fce6c629c0314699

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install grpcio-tools 'mcp>=1.0' 'fastmcp>=3' PyYAML

cp .env.example .env
# 编辑 .env，填写可用的 VLM_BASE_URL、VLM_API_KEY 和 VLM_MODEL。
set -a
source .env
set +a

rbnx build
rbnx boot
```

执行 `rbnx build` 和 `rbnx boot` 时保持虚拟环境处于激活状态。

在另一个终端检查运行状态并提交任务：

```bash
rbnx caps -v
rbnx chat
```

在交互界面输入 `Say hello to Alice.`。系统应首次激活 `say_hello` 技能并返回问候语。退出交互界面后，在部署终端按 `Ctrl+C`；也可以从另一个终端执行：

```bash
rbnx shutdown
```

本例安装的依赖分别用于 gRPC 类型生成、模型上下文协议服务器与工具层，以及 YAML 配置读取。若构建或启动报告 Python 模块不存在，先确认当前终端仍在 `.venv` 中，再补装报错指出的模块。

:::tip[从模板开始修改]

- 替换 `primitives/mock_chassis/` 中的模拟驱动，可以接入真实底盘。
- 替换 `services/my_navigate/` 中的占位规划逻辑，可以接入 Nav2 或其他导航器。
- 复制 `skills/say_hello/`，可以建立带自定义能力约定的技能。

:::

## 2. Robonix 是什么

Robonix 为具身智能模型提供统一的运行时。硬件驱动以原语暴露设备能力；建图、导航、语音等算法以服务暴露可复用功能；技能封装具有任务语义的行为。Atlas 维护能力目录，Pilot 把用户意图转换为执行方案，Executor 调用方案中的能力。

开发软件包时会遇到以下系统组件：

| 组件 | 作用 | 软件包开发者何时需要配置 |
|---|---|---|
| Atlas | 保存能力约定、提供方、能力和连接信息 | 每个软件包都要连接；通常只配置地址 |
| Executor | 执行 Pilot 生成的实时任务描述语言（RTDL）方案 | 技能或长任务需要验证执行、状态与取消时 |
| Soma | 保存本体结构和部件状态，管理原语与技能 | 接入新机器人、硬件部件或本体状态时 |
| Vitals | 汇总组件与设备健康状态 | 提供健康能力或接入健康仪表盘时 |
| Pilot | 把用户意图规划为 RTDL | 需要让模型发现和调用能力时 |
| Liaison | 承接客户端、文本和语音交互 | 接入外部客户端或语音链路时 |
| Scene | 维护环境对象、区域和空间关系 | 技能需要查询环境或语义目标时 |

运行时围绕以下对象组织：

| 对象 | 作用 | 示例 |
|---|---|---|
| 软件包 | 描述一份可构建、可启动的交付物 | `primitive-realsense-camera-rbnx` |
| 能力约定（Contract） | 稳定、可复用的接口描述，包含数据类型、交互模式、版本和类别 | `robonix/primitive/camera/rgb` |
| 能力（Capability） | 运行中提供方针对某条能力约定声明的实现与传输绑定 | `front_camera` + `robonix/primitive/camera/rgb` + `ros2` |
| 能力提供方（CapabilityProvider） | 软件包启动后注册到 Atlas 的运行实例 | `front_camera` |

能力约定可以在没有提供方运行时由 Atlas 从 TOML 和 IDL 加载。能力只在提供方注册并声明后存在，记录 `provider_id`、`contract_id`、传输及传输参数；调用方连接后再从通道取得端点。

能力目录属于控制面。真实图像、点云、RPC 请求或工具调用通过 ROS 2、gRPC 或模型上下文协议（Model Context Protocol，MCP）直接传输，不经过 Atlas 转发。

当前由 Pilot 规划、Executor 代表模型调用的业务能力使用 MCP 传输。gRPC 和 ROS 2 服务适合确定性客户端或组件间调用；能力约定中的 `rpc` 只表示请求—响应语义，不指定某一种通信库。

## 3. 原语、服务与技能

### 3.1 原语、服务与技能的职责

| 类型 | 责任 | 典型内容 |
|---|---|---|
| 原语 | 把设备和硬件状态转换为标准接口 | 底盘、机械臂、相机、雷达、音频、设备健康 |
| 服务 | 组合数据或算法，提供场景级功能 | 建图、定位、导航、语音、记忆 |
| 技能 | 暴露可由 Pilot 选择的任务级行为 | 探索、抓取、问候 |

Python 中分别使用 `Primitive`、`Service` 和 `Skill`。三者共享能力声明、生命周期和连接接口；区别主要体现在向 Atlas 注册的类型，以及 Executor 对技能的延迟激活策略。

原语命名空间不是封闭枚举。Robonix 当前提供机械臂、音频、相机、底盘、设备健康、IMU、激光雷达和机器人描述等标准能力约定；新硬件类型可以提交新的标准约定。

### 3.2 运行时身份

部署清单中实例的 `name` 必须等于代码创建运行实例时的 `id`：

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

同一能力提供方可以用不同传输实现同一能力约定，并为每种传输分别声明一项能力。因此运行时能力键包含：

```text
(provider_id, contract_id, transport)
```

Atlas 查询返回能力元数据。调用 `connect_capability(...)` 后得到的通道（Channel）才包含本次连接应使用的端点。

### 3.4 能力目录（Atlas）

Atlas 二进制未配置时绑定 `0.0.0.0:50051`；`rbnx init` 生成的部署清单会显式设置 `127.0.0.1:50051`。服务端监听地址可由部署中的 `system.atlas.listen` 或 `ROBONIX_ATLAS_LISTEN` 修改，能力提供方和 `rbnx` 等客户端则用 `ROBONIX_ATLAS` 选择连接端点。能力提供方启动后先注册自身，再声明能力；消费者可通过 `ATLAS.find_capability(...)` 搜索并建立连接。详细流程见 [Atlas 能力目录](architecture/atlas.md)。

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

`kind` 是能力约定元数据，不决定提供方在 Atlas 中注册成原语、服务还是技能；提供方类型由 `Primitive`、`Service` 或 `Skill` 决定。

### 4.2 模式与传输

模式（mode）描述接口语义和数据流方向；传输（transport）描述运行时通信技术。

| 模式 | 语义 | 常见传输 |
|---|---|---|
| `rpc` | 一次请求、一次响应 | gRPC、MCP、ROS 2 service |
| `rpc_server_stream` | 一次请求、连续响应 | gRPC |
| `rpc_client_stream` | 连续请求、一次响应 | gRPC |
| `rpc_bidirectional_stream` | 双向连续消息 | gRPC |
| `topic_out` | 提供方持续发布 | ROS 2 话题 |
| `topic_in` | 提供方持续接收 | ROS 2 话题 |

`rpc` 不等于 gRPC，也不要求使用 gRPC 装饰器。它只说明“一次请求、一次响应”：选择 gRPC 时使用 `@provider.grpc(...)`，选择 MCP 时使用 `@provider.mcp(...)`，选择 ROS 2 service 时由 `rclpy` 创建服务，再用 `provider.declare_ros2_service(...)` 向 Atlas 声明端点。提供方应根据现有实现、调用方和部署拓扑选择传输。

当前 Python 运行时不会在装饰器注册阶段完整校验“模式—传输”矩阵，因此还要通过端到端测试确认生成类型、提供方和消费者选择了同一传输。

### 4.3 全局与软件包内能力约定

标准能力约定位于 Robonix 源码的 `capabilities/`。只有软件包独有、尚未成为标准接口的能力约定才放到自身 `capabilities/`。`rbnx codegen` 和 `rbnx boot` 会合并全局约定与部署中软件包的局部约定。高级目录替换和同 ID 覆盖规则见 [Atlas 能力目录](architecture/atlas.md)。

## 5. 生命周期

### 5.1 状态

原语、服务与技能的运行实例使用以下状态。图中的蓝色路径是正常生命周期，红色路径表示命令失败或运行时异常，灰色路径表示关闭或心跳超时：

![能力提供方生命周期状态机](./cap-lifecycle.png)

该图突出正常启动、运行、停用和退出路径，不穷举 Atlas 接受的全部状态上报。当前 Atlas 还允许 `ERROR → INACTIVE`，用于实例修正故障后重新初始化；任何状态都可以进入 `ERROR` 或 `TERMINATED`，重复上报相同状态按幂等操作接受。

| 状态 | 含义 |
|---|---|
| `REGISTERED` | 已向 Atlas 注册，但尚未完成初始化 |
| `INACTIVE` | 配置已解析，尚未占用热运行资源 |
| `ACTIVE` | 可以处理业务请求或数据流 |
| `ERROR` | 初始化或运行失败 |
| `TERMINATED` | 正在退出或已经退出 |

### 5.2 生命周期接口与处理函数

生命周期接口（Driver）是启动器向提供方发送初始化配置、激活、停用和关闭命令的管理端点。它与普通业务能力分开。

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

| 清单中的 Driver 约定 | 启动行为 |
|---|---|
| 0 条 | 提供方监听就绪后直接进入 `ACTIVE`；不会收到初始化配置或生命周期命令 |
| 1 条 | 启动器等待同名运行时能力，再发送初始化和激活命令 |
| 多于 1 条 | 软件包启动失败；一个提供方只能有一个生命周期入口 |

需要受管配置或延迟激活的提供方必须实现 `<namespace>/driver`。标准命名空间复用主仓库中的 Driver 约定；自定义命名空间没有标准约定时，在软件包内添加 Driver TOML。清单与运行时声明必须使用同一个约定 ID。

原语和服务通常由启动流程初始化并激活。技能启动后保持 `INACTIVE`，Executor 第一次调用前发送激活命令；当前没有自动的空闲回收策略，技能会保持 `ACTIVE`，直到显式停用或系统关闭。

## 6. 软件包结构

### 6.1 生成骨架

如果沿用第 1 节的模板仓库，先回到其父目录并创建后续章节使用的空部署；不要在已经包含 `my_navigate` 和 `say_hello` 的模板中重复创建同名软件包：

```bash
cd ..
rbnx init robot-acme-rover
cd robot-acme-rover
mkdir -p config primitives services skills urdf assets
```

随后创建需要开发的软件包：

```bash
rbnx package-new my_camera --type primitive
rbnx package-new my_mapper --type service
rbnx package-new my_skill --type skill
```

命令分别写入 `primitives/`、`services/` 和 `skills/`。也可以指定准确目录：

```bash
rbnx package-new my_camera --type primitive --path ./packages/my_camera
```

从部署仓库根目录执行命令时，生成结构为：

```text
primitives/my_camera/
├── .gitignore
├── package_manifest.yaml
├── capabilities/
│   └── .gitkeep
├── scripts/
│   ├── build.sh
│   └── start.sh
└── my_camera/
    ├── __init__.py
    └── main.py
```

脚手架会生成清单、脚本、Python 模块和空的 `capabilities/`。开始实现后，还应在软件包根目录添加 `config.spec`，说明公开配置的类型、单位、默认值和约束。

`rbnx package-new` 只创建软件包目录，不会修改部署仓库的 `robonix_manifest.yaml`。要随整机启动新软件包，必须根据类型把它作为一条实例显式加入清单的 `primitive:`、`service:` 或 `skill:` 列表。

生成的 `main.py` 包含 `@provider.on_init`，但初始清单没有 Driver 约定。此时提供方会直接进入 `ACTIVE`，不会调用 `on_init` 或接收部署 `config`。无配置软件包可删除该处理函数；需要初始化配置时，按第 5.2 节添加 Driver 约定并重新运行代码生成。

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

对于由 `rbnx boot` 或 Soma 管理的软件包，`stop` 是可选的 shell 命令。受管关闭顺序为 Driver `CMD_SHUTDOWN`、`stop`、进程组 TERM/KILL；命令在软件包根目录执行。单独执行 `rbnx start` 当前不会自动运行 `stop`。

`capabilities` 列出软件包声明的全部能力约定。标准约定只写 `name`；尚未进入全局目录的约定放在本包 `capabilities/`，并用 `path` 引用相对软件包根目录的 TOML。旧字段名 `definition` 仍作为 `path` 的别名读取。清单不会替代码声明运行时能力；普通业务能力仍需通过 `@provider.mcp`、`@provider.grpc`、`create_publisher` 等 API 注册。

Driver 约定的数量和启动行为见第 5.2 节。

`depends` 每项使用 `name`，并可带 `path`、`url` 和 `branch`。这些字段当前只被解析为依赖元数据，不会自动克隆、安装依赖或修改 `PYTHONPATH`；构建脚本仍需自行处理依赖。

### 6.3 构建与启动脚本

代码生成使用当前 `python3` 生成 gRPC 类型。先在软件包实际使用的 Python 环境中安装并验证依赖：

```bash
python3 -m pip install grpcio-tools
python3 -c 'import grpc_tools'
```

第二条命令应以状态码 0 结束。若软件包使用虚拟环境，执行 `rbnx build` 和 `rbnx start` 时都要保持该环境处于激活状态。

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
from robonix_api import Service, Ok
from hello_mcp import Hello_Request, Hello_Response

service = Service(id="my_service", namespace="robonix/service/example")

@service.on_init
def init(cfg: dict):
    return Ok()

@service.mcp("robonix/service/example/hello")
def hello(req: Hello_Request) -> Hello_Response:
    """Return a greeting for the requested name."""
    return Hello_Response(message=f"Hello, {req.name}!")

service.run()
```

常用返回值：

- `Ok()`：生命周期动作成功；
- `Err("reason")`：不可恢复错误；
- `Deferred("reason")`：依赖尚未就绪，保持当前状态并把原因写入 `state_detail`。当前 `rbnx boot` 和 Executor 不会自动重试；调用方或操作方需要在修复依赖后重新发起生命周期流程。

`on_init`、`on_activate` 和 `on_deactivate` 必须返回 `Ok`、`Err` 或 `Deferred`；抛出的异常会转换为错误，漏写返回值或返回其他类型会触发 `TypeError` 并使 Driver RPC 失败。`on_shutdown` 可以省略返回值，运行时会在执行 teardown 后把 `None` 视为 `Ok()`。

## 8. 开发服务

下面的教学服务复用标准导航能力约定，因此不复制全局 TOML。软件包清单列出生命周期、提交目标、查询状态和取消目标四条能力约定；运行时再把每条约定绑定为该服务提供的能力。示例中的状态表只是可运行骨架，接入真正规划器时必须由规划器回调更新状态。

```bash
rbnx package-new my_navigate --type service
```

将 `package_manifest.yaml` 中的能力约定列表改为：

```yaml title="services/my_navigate/package_manifest.yaml"
capabilities:
  - name: robonix/service/navigation/driver
  - name: robonix/service/navigation/navigate
  - name: robonix/service/navigation/navigate/status
  - name: robonix/service/navigation/navigate/cancel
```

将 `scripts/build.sh` 中的代码生成命令改为 `rbnx codegen -p "$PKG" --mcp`，然后实现：

```python title="services/my_navigate/my_navigate/main.py"
from __future__ import annotations

import uuid

from robonix_api import Service, Ok
from navigation_mcp import (
    CancelNavigation_Request,
    CancelNavigation_Response,
    GetNavigationStatus_Request,
    GetNavigationStatus_Response,
    Navigate_Request,
    Navigate_Response,
)

navigate_service = Service(
    id="my_navigate",
    namespace="robonix/service/navigation",
)
runs: dict[str, str] = {}
latest_run_id: str | None = None

@navigate_service.on_init
def init(cfg: dict):
    return Ok()

@navigate_service.mcp("robonix/service/navigation/navigate")
def navigate(req: Navigate_Request) -> Navigate_Response:
    """Accept a map-frame navigation goal."""
    global latest_run_id
    run_id = str(uuid.uuid4())
    # 在这里把 req.goal 交给真正的规划器。
    runs[run_id] = "RUNNING"
    latest_run_id = run_id
    return Navigate_Response(
        accepted=True,
        run_id=run_id,
        detail="goal accepted",
    )

@navigate_service.mcp("robonix/service/navigation/navigate/status")
def status(req: GetNavigationStatus_Request) -> GetNavigationStatus_Response:
    run_id = req.run_id or latest_run_id
    state = runs.get(run_id) if run_id is not None else None
    return GetNavigationStatus_Response(
        known=state is not None,
        state=state or "",
        detail="known goal" if state is not None else "unknown run_id",
    )

@navigate_service.mcp("robonix/service/navigation/navigate/cancel")
def cancel(req: CancelNavigation_Request) -> CancelNavigation_Response:
    run_id = req.run_id or latest_run_id
    if run_id is None or run_id not in runs:
        return CancelNavigation_Response(accepted=False, detail="unknown run_id")
    runs[run_id] = "CANCELED"
    return CancelNavigation_Response(accepted=True, detail="cancel requested")

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

`name` 必须与 `Service(id="my_navigate", ...)` 一致。清单中的 `driver` 条目引用能力约定；`provider.run()` 根据生成代码声明对应的 driver 能力，启动流程连接该能力并发送 `CMD_INIT`，服务随后进入激活阶段。服务若需要底盘，应在 `on_activate` 中发现并连接目标运行实例：

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

### 8.1 验证

```bash
rbnx validate ./services/my_navigate
rbnx build -p ./services/my_navigate
```

两条命令都成功后，按第 13 节启动部署。`rbnx caps -v` 应显示 `my_navigate` 为 `ACTIVE`，并列出提交目标、查询状态和取消目标三条 MCP 业务能力；`rbnx tools` 应显示对应工具。若只出现 Driver，说明处理函数没有完成注册或代码生成产物未加载。

## 9. 开发原语

原语负责把厂商 SDK、总线或 ROS 2 驱动适配为标准接口。以底盘的一次性移动命令为例：

```bash
rbnx package-new base_chassis --type primitive
```

在 `primitives/base_chassis/package_manifest.yaml` 中列出本例实现的标准能力约定：

```yaml
capabilities:
  - name: robonix/primitive/chassis/driver
  - name: robonix/primitive/chassis/move
  - name: robonix/primitive/chassis/odom
```

下面的示例导入 `nav_msgs.msg.Odometry` 并发布 ROS 2 数据，因此还要构建并加载代码生成的 ROS 2 叠加层。将脚手架的构建脚本改为：

```bash title="primitives/base_chassis/scripts/build.sh"
#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROS_SETUP="${RBNX_ROS_SETUP:-/opt/ros/humble/setup.bash}"

source "$ROS_SETUP"
rbnx codegen -p "$PKG" --mcp --ros2
cd "$PKG/rbnx-build/codegen/ros2_idl"
colcon build
```

启动脚本必须先加载系统 ROS 2 环境，再加载本软件包生成的叠加层：

```bash title="primitives/base_chassis/scripts/start.sh"
#!/usr/bin/env bash
set -euo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROS_SETUP="${RBNX_ROS_SETUP:-/opt/ros/humble/setup.bash}"

source "$ROS_SETUP"
source "$PKG_ROOT/rbnx-build/codegen/ros2_idl/install/setup.bash"
cd "$PKG_ROOT"
export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"
exec python3 -m base_chassis.main
```

目标平台不是 ROS 2 Humble 时，在构建和启动前把 `RBNX_ROS_SETUP` 设为该平台的 `setup.bash`。系统 ROS 2 提供运行时；Robonix 能力约定使用的消息和服务类型仍必须来自随后加载的代码生成叠加层。

```python title="primitives/base_chassis/base_chassis/main.py"
import json

from robonix_api import Primitive, Ok
import chassis_pb2
import std_msgs_pb2
from nav_msgs.msg import Odometry

chassis = Primitive(id="base_chassis", namespace="robonix/primitive/chassis")

@chassis.on_init
def init(cfg: dict):
    # create_publisher 会立即向 Atlas 声明能力，因此应在 provider 注册后的
    # 生命周期处理函数中调用，不要在模块导入阶段调用。
    chassis.create_publisher(
        contract_id="robonix/primitive/chassis/odom",
        topic="/odom",
        msg_type=Odometry,
        qos="best_effort",
    )
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

ROS 2 话题可以通过便捷层创建并自动向 Atlas 声明。上面的 `on_init` 已创建发布者；产生里程计消息后发布：

```python
chassis.emit("robonix/primitive/chassis/odom", odom_message)
```

支持的字符串 QoS 为 `best_effort`、`reliable` 和 `latched`。也可传整数队列深度或原生 `QoSProfile`。原语既可以使用这些便捷接口，也可以直接使用 `rclpy`；无论哪种方式，都必须保证实际话题类型、方向和声明一致。

在部署清单的 `primitive` 列表中加入：

```yaml
primitive:
  - name: base_chassis
    path: ./primitives/base_chassis
    config: {}
```

### 9.1 验证

```bash
rbnx validate ./primitives/base_chassis
rbnx build -p ./primitives/base_chassis
```

按第 13 节启动部署后，`rbnx caps -v` 应显示 `base_chassis` 为 `ACTIVE`，并分别以 gRPC 和 ROS 2 传输列出 `move` 与 `odom`。本例没有真实 SDK，不要把移动接口用于实车；接入厂商驱动后再按本体接入指南的无运动、单设备和整机阶段验收。

## 10. 开发技能

技能通常使用 `robonix/skill/<name>` 命名空间，使 Executor 能识别并在第一次调用前激活。自定义技能接口需要软件包内 IDL 和能力约定：

```bash
rbnx package-new say_hello --type skill
```

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

在 `skills/say_hello/package_manifest.yaml` 中引用两条软件包内能力约定：

```yaml
capabilities:
  - name: robonix/skill/say_hello/driver
    path: capabilities/driver.v1.toml
  - name: robonix/skill/say_hello/say
    path: capabilities/say.v1.toml
```

将构建脚本的代码生成命令设为 `rbnx codegen -p "$PKG" --mcp`，然后实现技能：

```python title="skills/say_hello/say_hello/main.py"
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

在部署清单的 `skill` 列表中加入：

```yaml
skill:
  - name: say_hello
    path: ./skills/say_hello
    config: {}
```

### 10.1 验证

```bash
rbnx validate ./skills/say_hello
rbnx build -p ./skills/say_hello
```

按第 13 节启动部署后，`rbnx caps -v` 应显示 `say_hello` 为 `INACTIVE`，`rbnx tools` 应列出 `robonix/skill/say_hello/say`。通过 `rbnx chat` 提交 `Say hello to Alice.` 后，技能应只激活一次并返回问候语。

已有 Python 脚本直接调用厂商 SDK 时，不要把整段硬件访问包装进技能。先把 SDK 适配为标准原语，再让任务级技能调用原语。完整迁移示例见[以抓积木为例接入现有 Python 功能](tutorials/existing-python-feature.md)。

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

从第 6 节顺序阅读时，当前目录已经是 `robot-acme-rover`，无需再次初始化。若从本节开始阅读，则先创建部署骨架：

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
      upstream: "${VLM_BASE_URL}"
      api_key: "${VLM_API_KEY}"
      model: "${VLM_MODEL}"
      api_format: openai
  liaison: { listen: 0.0.0.0:50081, log: info }

primitive:
  - name: base_chassis
    path: ./primitives/base_chassis
    config: {}

service:
  - name: my_navigate
    path: ./services/my_navigate
    config: {}

skill:
  - name: say_hello
    path: ./skills/say_hello
    config: {}
```

实例 `config` 在初始化时通过 Driver 发送；未声明 Driver 的提供方不会收到配置。软件包应在 `on_init(cfg)` 中校验字段并返回清楚的错误，公开字段同时写入 `config.spec`。

远程软件包使用 `url`，本地软件包使用 `path`。`manifest` 选择软件包提供的平台清单，例如 Jetson 原生清单。提供平台变体时仍要保留默认 `package_manifest.yaml`；完整字段见[软件包与部署清单规范](integration-guide/packaging-spec.md)。

## 13. 构建、启动与停止

在部署目录执行：

```bash
rbnx build
rbnx boot
```

启动时可以依赖以下阶段顺序：

1. 未使用 `--skip-system` 时，启动清单中声明的 Atlas、Executor；存在原语或技能时，未显式声明的 Soma 会被自动补入并启动；
2. Soma 启动原语；没有原语时跳过；
3. 启动清单中声明的其余内置组件（如 Vitals、Pilot、Liaison）、额外系统软件包和服务；
4. Soma 启动技能，带 driver 的技能停在 `INACTIVE`；没有技能时跳过。

使用 `--skip-system` 时，`rbnx` 跳过整个 `system` 块和由本次启动管理的 Soma 原语/技能阶段，只连接外部 Atlas 并启动服务。此模式下，操作方必须另行保证外部系统、原语和技能已经就绪。

`rbnx boot` 输出启动摘要后，按以下结果判断成功：

1. 摘要没有 `failures`、`[FAIL]` 或失败的软件包；
2. `rbnx caps -v` 包含清单中预期的每个提供方及能力；
3. 原语和服务通常为 `ACTIVE`，尚未调用的技能通常为 `INACTIVE`；
4. `rbnx logs --tag <provider_id>` 没有初始化、连接或配置错误。

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

该命令读取原部署清单，依次发送生命周期关闭、执行软件包 `stop` 钩子，并终止受管进程。原部署清单必须仍然存在且可解析。容器化软件包还应在自己的 `stop` 钩子中删除或停止由它创建的容器。

## 14. Python 接口

本节解释软件包开发最常用的入口和参数。完整签名、默认值、返回类型与源码链接由 <a href="/api/python/public-api.html">Python 公共 API</a> 从当前源码生成；这里不重复内部方法。

### 14.1 构造能力提供方

```text
Primitive(id, namespace, *, pkg_root=None, md_path=None)
Service(id, namespace, *, pkg_root=None, md_path=None)
Skill(id, namespace, *, pkg_root=None, md_path=None)
```

| 参数 | 含义 |
|---|---|
| `id` | Atlas 中的提供方 ID。整机启动时必须等于部署实例的 `name`。构造函数不会自动读取环境变量；可复用软件包可把 `RBNX_INSTANCE_NAME` 显式传入。 |
| `namespace` | 提供方的主要能力命名空间。运行时会去掉首尾 `/`，空值直接报错；普通能力约定落在该命名空间之外时只记录警告。 |
| `pkg_root` | 软件包根目录。省略时先从调用文件向上寻找 `package_manifest.yaml`，找不到再使用当前工作目录；代码生成产物和默认 `CAPABILITY.md` 都从这里解析。 |
| `md_path` | 显式指定 `CAPABILITY.md`。省略时使用软件包根目录中实际存在的同名文件；传空字符串表示不注册能力说明。 |

`Primitive`、`Service` 和 `Skill` 共享同一组开发接口；选择哪一个只决定向 Atlas 注册的提供方类型和技能的延迟激活行为。

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

这些参数均为仅限关键字参数：

| 参数 | 默认值 | 过滤对象 |
|---|---|---|
| `contract_id` | `""` | 能力约定 ID；空字符串表示不限 |
| `transport` | `Transport.UNSPECIFIED` | `GRPC`、`ROS2` 或 `MCP`；未指定表示不限 |
| `provider_kind` | `Kind.UNSPECIFIED` | 原语、服务或技能；未指定表示不限 |
| `provider_id` | `""` | 指定运行实例 ID；空字符串表示不限 |
| `namespace_prefix` | `""` | 提供方主要命名空间前缀；空字符串表示不限 |

`find_capability(...)` 返回 `list[Capability]`。需要恰好一个结果时使用 `find_unique_capability(...)`；其签名要求显式传入 `contract_id`，零个或多个结果都会抛出 `ValueError`。通常应传非空 ID，把唯一性限定在一条能力约定内；多实例实现同一约定时再用 `provider_id` 消除歧义。

### 14.3 连接能力

```python
channel = service.connect_capability(
    provider,
    contract_id,
    transport,
)
```

| 参数 | 含义 |
|---|---|
| `provider` | `ATLAS.find_capability(...)` 返回的 `Capability`，或 `ATLAS.query_*` 返回的 `CapabilityProvider` |
| `contract_id` | 要连接的能力约定 ID |
| `transport` | 本次连接使用的 `Transport` 枚举、字符串或整数值 |

返回的 `Channel` 包含 `endpoint`、`channel_id` 和传输参数，并在 Atlas 中记录消费者到提供方的连接。用 `with channel:` 或显式调用 `channel.close()` 可以断开连接；提供方退出时也会关闭自己建立的全部通道。`Capability.description` 保存这项运行时实现的实例说明，能力约定的通用说明则位于 `ContractDescriptor.description`；消费者可合并两者。`McpParams` 当前只包含输入结构 JSON。

### 14.4 ROS 2 便捷接口

完整签名如下；`contract_id` 之后的参数只能按名称传入：

```text
create_publisher(contract_id, *, topic, msg_type,
                 qos="best_effort", declare=True, description="")
create_subscription(contract_id, *, topic, msg_type, callback,
                    qos="best_effort", declare=True)
emit(contract_id, msg)
```

典型用法：

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
    qos="best_effort",
)
provider.emit(contract_id, message)
```

`topic` 是实际 ROS 2 名称；`msg_type` 接受消息类或运行时可解析的类型字符串；`qos` 接受预设字符串或整数深度。`declare=True` 时，方法还会向 Atlas 声明能力；只消费别人的话题时传 `declare=False`。发布者的 `description` 是给调用方和模型看的实例说明。`emit` 按 `contract_id` 查找此前创建的发布者，未创建时抛出 `RuntimeError`。

`create_publisher` 和 `create_subscription` 会立即创建 ROS 对象，应在 provider 注册后调用，通常放在 `on_init`。`declare_ros2_topic` 和 `declare_ros2_service` 只向 Atlas 声明端点；它们不会替开发者创建 `rclpy` 发布者、订阅者或服务。

### 14.5 MCP 与 gRPC

```python
@provider.mcp(contract_id, description="...")
def tool(req: RequestType) -> ResponseType:
    ...

@provider.grpc(contract_id, description="...")
def method(req, context):
    ...
```

`contract_id` 是能力约定 ID。装饰器把处理函数和传输端点绑定到该约定，并在启动时声明运行时能力。`description` 可省略；运行时优先使用显式值，否则读取处理函数的 docstring。MCP 使用代码生成的数据类；gRPC 使用 `*_pb2.py` 消息和 `robonix_contracts_pb2_grpc.py` 中的服务定义。不要从业务消息模块导入生成的 Servicer 或 Stub。

### 14.6 关闭行为

`provider.run()` 管理注册、服务监听、心跳和信号处理。退出时会标记 `TERMINATED`，并关闭受管通道和服务。软件包自行创建的线程、设备句柄和后台进程应在 `on_shutdown` 中释放。

## 15. 常用命令行接口

| 命令 | 用途 |
|---|---|
| `rbnx setup [root]` | 登记 Robonix 源码根目录；省略时使用当前目录并向上查找源码根 |
| `rbnx path <key>` | 输出源码树中的绝对路径 |
| `rbnx init <name>` | 创建机器人部署骨架 |
| `rbnx package-new <name> --type <type>` | 创建软件包骨架 |
| `rbnx validate [path]` | 校验软件包清单 |
| `rbnx codegen -p <package> [--mcp] [--ros2]` | 生成 gRPC、MCP 或 ROS 2 接口代码 |
| `rbnx build [-p <package> \| -f <manifest>]` | 构建软件包或整个部署 |
| `rbnx start [-p <package>]` | 单独启动软件包；生命周期行为见第 5.2 节 |
| `rbnx boot [-f <manifest>]` | 启动整套部署 |
| `rbnx shutdown [-f <manifest>]` | 停止对应部署 |
| `rbnx update [-p <package> \| -f <manifest>]` | 更新远程软件包 |
| `rbnx clean [-p <package> \| -f <manifest>] [--cache]` | 清理构建和运行产物 |
| `rbnx caps [-v]` | 查看提供方和能力 |
| `rbnx contracts [-v]` | 查看 Atlas 已加载的能力约定 |
| `rbnx describe --provider <id>` | 查看提供方的 `CAPABILITY.md` |
| `rbnx tools` | 查看 Pilot 可调用工具 |
| `rbnx channels` | 查看活动连接 |
| `rbnx inspect` | 输出完整运行时状态 |
| `rbnx ask "<prompt>"` | 非交互提交一次任务 |
| `rbnx chat` | 启动交互界面 |
| `rbnx logs` | 读取结构化日志 |

`rbnx clean -f robonix_manifest.yaml` 默认保留 `rbnx-boot/cache/`；只有加 `--cache` 才删除远程软件包缓存。单独执行 `rbnx start --config <file>` 时仍会先读取软件包清单，然后在提供方注册后通过 Driver `CMD_INIT` 发送合并后的配置；配置文件路径不会暴露给提供方进程。

## 16. 配置字段

### 16.1 软件包清单

| 字段 | 当前作用 |
|---|---|
| `manifestVersion` | 当前接受大于等于 1 的版本；新清单使用 1 |
| `package.name` | 软件包发布名，必填且非空；发布到目录时必须与 `catalog.yaml` 的 `packages[].name` 一致，不要求与本地目录名一致 |
| `package.version` | 版本字符串，必填；当前不强制解析语义化版本 |
| `package.description` | 软件包说明，必填 |
| `package.license` | 许可证字符串，必填；约定使用 SPDX 标识，当前只校验非空 |
| `package.tags` | 目录分类，可选 |
| `package.maintainers` | 维护者列表，可选 |
| `build` | 构建命令，可选；省略表示无构建步骤 |
| `start` | 启动命令，必填 |
| `stop` | 停止命令，可选；由 `rbnx boot` / Soma 受管关闭时，在 Driver `CMD_SHUTDOWN` 后、终止进程组前执行；单独 `rbnx start` 不自动执行 |
| `capabilities` | 能力约定引用和预期导出清单；每项含约定 ID `name`，可选约定 TOML `path`，不替代码声明普通业务能力。恰好一条 `*/driver` 约定还会让 `rbnx start` 等待对应的运行时 driver 能力并发送生命周期配置；多条 driver 会使启动失败 |
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
| 实例 `name` | 运行时提供方 ID，必须与代码一致 |
| 实例 `path` / `url` | 本地路径或远程仓库；每个原语、服务和技能实例必须且只能设置其中一个 |
| 实例 `branch` | 仅与 `url` 一起使用；首次克隆远程仓库时选择分支或标签。不支持任意提交 SHA，已有缓存不会因字段变化自动切换 |
| 实例 `manifest` | 选择软件包内的目标清单 |
| 实例 `config` | 透传给该实例的配置映射 |

部署清单的相对路径以部署文件所在目录为基准。软件包如何解释 `config` 由其 `config.spec` 和实现共同定义。远程包缓存在 `rbnx-boot/cache/`；修改 `branch` 后需要手动切换缓存中的源码检出，或用 `rbnx clean -f <manifest> --cache` 删除缓存再重新构建。

### 16.3 能力约定 TOML

| 字段 | 含义 |
|---|---|
| `contract.id` | 合并后能力约定目录中的接口名 |
| `contract.version` | 能力约定版本 |
| `contract.kind` | 文档和代码生成使用的分类元数据 |
| `contract.idl` | 相对 IDL 根目录的数据类型路径 |
| `contract.cross_namespace` | 允许共享接口跨提供方命名空间而不提示 |
| `mode.type` | RPC 或话题方向 |

同一 ID 在有效合并目录中只保留一个描述；后加载根可能覆盖先加载描述。软件包应复用标准能力约定，只有明确需要时才新增自定义 ID，并通过 `rbnx contracts -v` 检查最终来源。
