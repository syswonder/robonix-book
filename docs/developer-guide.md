---
toc_max_heading_level: 2
---

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

方向始终以声明该能力的提供方为基准：`topic_out` 表示提供方输出，`topic_in` 表示提供方接收。它不描述调用方变量名，也不等同于某个固定 ROS 2 话题。

| 模式 | gRPC | ROS 2 | MCP |
|---|:---:|:---:|:---:|
| `rpc` | 支持 | 通过 service 支持 | 支持 |
| `rpc_server_stream` | 支持 | 不支持 | 不支持 |
| `rpc_client_stream` | 支持 | 不支持 | 不支持 |
| `rpc_bidirectional_stream` | 支持 | 不支持 | 不支持 |
| `topic_out` / `topic_in` | 以 gRPC stream 映射 | 支持 | 不支持 |

`rpc` 不等于 gRPC，也不要求使用 gRPC 装饰器。它只说明“一次请求、一次响应”：选择 gRPC 时使用 `@provider.grpc(...)`，选择 MCP 时使用 `@provider.mcp(...)`，选择 ROS 2 service 时由 `rclpy` 创建服务，再用 `provider.declare_ros2_service(...)` 向 Atlas 声明端点。提供方应根据现有实现、调用方和部署拓扑选择传输。

`rpc` 的一次响应也不表示业务动作必须已经结束。通过 MCP 启动导航、抓取等长时间任务时，提供方可以同时注册主能力及其 `/status`、`/cancel` 子能力，由 Executor 持续查询状态并响应方案取消。该约定见 [14.5 MCP 与 gRPC](#145-mcp-与-grpc)。

模型需要发现并离散调用的工具通常使用 MCP；确定性的进程间控制、生命周期和流式请求通常使用 gRPC；机器人内部已有的高频传感与控制图通常保留 ROS 2。能力约定只定义语义，不会替开发者自动选择传输。提供方声明的传输、Atlas 返回的端点和消费者建立的客户端必须一致。

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
    return Ok()
```

| 新软件包清单中的 Driver 约定 | 启动行为 |
|---|---|
| 0 条 | 自动选择并注册共享 `robonix/lifecycle/driver`；提供方仍有 Driver |
| 1 条 `robonix/lifecycle/driver` | 显式选择共享 Driver，行为与省略 Driver 条目相同 |
| 多于 1 条 | 软件包启动失败；一个提供方只能有一个生命周期入口 |

每个提供方都使用一条生命周期 Driver。新软件包通常不在清单中声明 Driver；框架会自动选择并注册共享的 `robonix/lifecycle/driver`。该约定及 `lifecycle/Driver` IDL 由 Robonix 主仓库提供，不需要创建 Driver TOML。显式声明共享 Driver 仍受支持，但不会改变运行行为。

:::warning[后向兼容：已有命名空间 Driver]
早期软件包可能自己维护 Driver TOML，并在清单中声明唯一的 `<provider-namespace>/driver`。这种完整的旧实现目前仍受支持，但计划逐步迁移到共享 Driver。维护旧包时保留原有 Driver 条目和 TOML，不要再追加共享 Driver。

若旧清单保持不变、旧生成服务完全不存在，而受管运行时只注册带兼容标记的共享 Driver，也允许单向迁移并输出警告。部分生成服务、命名空间不匹配或多条 Driver 都会使启动失败。旧包的完整验收和迁移步骤见[软件包与部署清单规范](integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

生命周期处理函数按需实现。某个回调缺失时，框架会记录警告并执行空操作；原语或服务仍会在 `CMD_INIT`、`CMD_ACTIVATE` 后进入 `ACTIVE`。处理函数显式返回 `Err` 时，启动或状态转换仍会失败。

| 处理函数 | 放入的工作 | 对称责任 |
|---|---|---|
| `on_init(cfg)` | 校验和保存配置、选择逻辑设备、检查静态依赖；不启动可延后到激活阶段的热循环 | 初始化失败返回 `Err`，依赖稍后可用返回 `Deferred` |
| `on_activate()` | 打开设备、加载模型、启动线程或订阅、连接上游能力等运行资源 | `on_deactivate()` 必须关闭本阶段获得的资源，并允许以后再次激活 |
| `on_deactivate()` | 停止运行循环、关闭连接和设备，但保留重新激活所需的配置 | 重复调用应安全，不留下后台线程、文件描述符或 Atlas channel |
| `on_shutdown()` | 处理任何状态下的最终清理 | 释放尚未由停用阶段释放的资源；不要在这里重新启动工作 |

确实需要在初始化完成前就存在的轻量端点可以在 `on_init` 创建；其最终释放仍要有明确归属。不要只在 `on_activate` 建立长期连接却省略 `on_deactivate`。

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

`--path` 只改变目标目录；`--type` 仍决定生成原语、服务还是技能的提供方类、命名空间和默认软件包名称，两者都必须正确。`package-new` 会生成清单、`build.sh`、`start.sh`、Python 模块和空的 `capabilities/`。开始实现后补上配置说明；如果软件包还需要释放框架之外的进程或资源，再增加幂等的 `stop.sh`。发布前的目录应整理为：

```text
primitives/my_camera/
├── .gitignore
├── package_manifest.yaml
├── capabilities/
│   └── .gitkeep
├── scripts/
│   ├── build.sh
│   ├── start.sh
│   └── stop.sh          # 使用清单 stop 时添加
├── config.spec             # 说明部署 config；无公开配置时可省略
└── my_camera/
    ├── __init__.py
    └── main.py
```

`config.spec` 说明公开配置的类型、单位、默认值和约束。`stop.sh` 不是必须的；只有清单声明 `stop:` 时才需要它，且重复执行不应报错或重复破坏资源。

`rbnx package-new` 只创建软件包目录，不会修改部署仓库的 `robonix_manifest.yaml`。要随整机启动新软件包，必须根据类型把它作为一条实例显式加入清单的 `primitive:`、`service:` 或 `skill:` 列表。

生成的清单不需要包含 Driver 条目；框架会自动补上共享生命周期 Driver。`main.py` 包含 `@provider.on_init` 占位实现。`rbnx boot` 会把部署项的 `config` 传入初始化回调。如果某个生命周期回调未实现，运行时记录警告并执行安全空操作；回调明确返回错误时，启动仍然失败。

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
stop: bash scripts/stop.sh

capabilities:
  - name: robonix/service/example/hello
    path: capabilities/hello.v1.toml

depends:
  - name: robonix.lib.example
    path: ../example-lib
```

运行时要求 `package.name`、`version`、`description`、`license` 和 `start` 非空。`tags` 与 `maintainers` 对目录发布很重要，但当前运行时不把它们当成必填字段，也不解析版本是否严格符合语义化版本。

对于由 `rbnx boot` 或 Soma 管理的软件包，`stop` 是可选的 shell 命令；上面的完整示例假定仓库已提供 `scripts/stop.sh`。受管关闭顺序为 Driver `CMD_SHUTDOWN`、`stop`、进程组 TERM/KILL；命令在软件包根目录执行。单独执行 `rbnx start` 当前不会自动运行 `stop`。软件包没有框架外资源需要清理时，删除清单中的 `stop:` 和对应脚本，不要保留无意义的空钩子。

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

已实现的生命周期处理函数都必须返回 `Ok`、`Err` 或 `Deferred`；抛出的异常会转换为错误，漏写返回值或返回其他类型会触发 `TypeError` 并使 Driver RPC 失败。不需要处理某个阶段时，直接省略对应回调，由框架记录警告并执行安全空操作。

## 8. 开发服务

下面的教学服务复用标准导航能力约定，因此不复制全局 TOML。软件包清单列出生命周期、提交目标、查询状态和取消目标四条能力约定；运行时再把每条约定绑定为该服务提供的能力。示例中的状态表只是可运行骨架，接入真正规划器时必须由规划器回调更新状态。

```bash
rbnx package-new my_navigate --type service
```

将 `package_manifest.yaml` 中的能力约定列表改为：

```yaml title="services/my_navigate/package_manifest.yaml"
capabilities:
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

`name` 必须与 `Service(id="my_navigate", ...)` 一致。该新清单省略 Driver 条目，框架会选择共享 `robonix/lifecycle/driver`；`provider.run()` 根据生成代码声明同一条 Driver 能力，启动流程连接它并发送 `CMD_INIT`，服务随后进入激活阶段。服务若需要底盘，应在 `on_activate` 中发现并连接目标运行实例：

```python
from robonix_api import ATLAS, Deferred, Ok
from robonix_api.atlas_types import Channel, Transport

move_channel: Channel | None = None

@navigate_service.on_activate
def activate():
    global move_channel
    if move_channel is not None:
        move_channel.close()
        move_channel = None
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

@navigate_service.on_deactivate
def deactivate():
    global move_channel
    if move_channel is not None:
        move_channel.close()
        move_channel = None
    return Ok()

@navigate_service.on_shutdown
def shutdown():
    return deactivate()
```

长期连接由获得它的生命周期阶段持有，并在停用阶段显式关闭。`Channel.close()` 是幂等操作，因此关闭阶段可以安全兜底。只为一次调用建立连接时使用 `with navigate_service.connect_capability(...) as channel:`；`with` 退出后不得继续使用其端点。

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
├── say.v1.toml
└── lib/say_hello/srv/SayHello.srv
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

在 `skills/say_hello/package_manifest.yaml` 中引用软件包内的业务能力约定；生命周期 Driver 由框架自动提供：

```yaml
capabilities:
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
  soma:
    listen: 127.0.0.1:50091
    robot_yaml: soma.yaml
    log: info
  vitals: { listen: 127.0.0.1:50093, log: info }
  scene:
    manifest: package_manifest.yaml
    config:
      web_port: 50107
  executor: { listen: 127.0.0.1:50061, log: info }
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

这个完整系统块包含本体状态（Soma）、健康状态（Vitals）、环境状态（Scene）、任务执行（Executor）、模型规划（Pilot）和交互入口（Liaison）。`scene.manifest` 选择 Scene 软件包的运行目标，`scene.config` 通过 Scene 的 Driver 传入；未指定 `camera_provider_id` 时，Scene 按能力约定自动发现可用观测源。

实例 `config` 在初始化时通过 Driver 发送。软件包应在 `on_init(cfg)` 中校验字段并返回清楚的错误，公开字段同时写入 `config.spec`。

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
4. Soma 启动技能并通过其唯一 Driver 发送 `CMD_INIT`，技能停在 `INACTIVE`；没有技能时跳过。

使用 `--skip-system` 时，`rbnx` 跳过整个 `system` 块和由本次启动管理的 Soma 原语/技能阶段，只连接外部 Atlas 并启动服务。此模式下，操作方必须另行保证外部系统、原语和技能已经就绪。

`rbnx boot` 输出启动摘要后，按以下结果判断成功：

1. 摘要没有 `failures`、`[FAIL]` 或失败的软件包；
2. `rbnx caps -v` 包含清单中预期的每个提供方及能力；
3. 原语和服务通常为 `ACTIVE`，尚未调用的技能通常为 `INACTIVE`；
4. `rbnx logs -t <provider_id>` 没有初始化、连接或配置错误。

常用诊断：

```bash
rbnx caps -v
rbnx tools
rbnx channels
rbnx inspect
rbnx logs --list-tags
rbnx logs -t my_navigate -l warn
rbnx logs -t my_navigate -f
```

以上命令应在机器人部署目录执行；从其他目录读取时使用 `rbnx logs -d /path/to/deploy/rbnx-boot/logs ...`。不带 `-f` 会读取并按时间排序已有记录；`-f` 从当前已经存在的日志文件末尾开始，只显示后续新增记录。每次 `rbnx boot` 会清空本次日志目录中已有的 `*.log`，复现故障前先保存需要保留的日志。

启动阶段需要实时逐行观察 INFO、WARN 和 ERROR 时使用：

```bash
rbnx boot -v -f robonix_manifest.yaml
```

`-v` 会关闭 spinner 与光标回写，但不会启用 DEBUG；完整记录仍写入 `rbnx-boot/logs/`。

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

`topic` 是实际 ROS 2 名称；`msg_type` 接受消息类或运行时可解析的类型字符串；`qos` 接受预设字符串或整数深度。`declare=True` 时，方法还会向 Atlas 声明能力。发布者的 `description` 是给调用方和模型看的实例说明。`emit` 按 `contract_id` 查找此前创建的发布者，未创建时抛出 `RuntimeError`。

消费另一提供方发布的话题时，常规路径不是手写话题名，而是让 Atlas 返回实际端点和 QoS，再从 `Channel` 创建订阅。下面的代码放在已经创建的 `service` 实例中；`on_image` 是接收 `sensor_msgs/msg/Image` 的业务回调：

```python
from robonix_api import ATLAS, Ok
from robonix_api.atlas_types import Transport

@service.on_init
def connect_camera(_config):
    camera_rgb = ATLAS.find_unique_capability(
        contract_id="robonix/primitive/camera/rgb",
        provider_id="front_camera",
        transport=Transport.ROS2,
    )
    channel = service.connect_capability(
        camera_rgb,
        camera_rgb.contract_id,
        Transport.ROS2,
    )
    service.create_subscription_from_channel(
        channel,
        msg_type="sensor_msgs/msg/Image",
        callback=on_image,
    )
    return Ok()

service.run()
```

`find_unique_capability` 负责发现并消除多实例歧义，`connect_capability` 在 Atlas 中建立消费者到提供方的连接，`create_subscription_from_channel` 使用返回的真实端点。当前实现尚未把 Atlas 返回的字符串 QoS 名称转换为 ROS 2 QoS 配置，而是使用后端默认值；需要精确 QoS 时，先完成能力发现，再用 `channel.endpoint` 和显式 `qos` 调用 `create_subscription(..., declare=False)`。只有在话题名由 Robonix 之外的系统固定、且调用方有意绕过 Atlas 发现时，才完全跳过能力发现；这种写法不会为消费端声明新能力。

`create_publisher` 和 `create_subscription` 会立即创建 ROS 对象，应在 provider 注册后调用，通常放在 `on_init`。`declare_ros2_topic` 和 `declare_ros2_service` 只向 Atlas 声明端点；它们不会替开发者创建 `rclpy` 发布者、订阅者或服务。

### 14.5 MCP 与 gRPC

MCP 处理函数接收代码生成的数据类，并返回对应的响应数据类：

```python
@provider.mcp(contract_id, description="...")
def tool(req: RequestType) -> ResponseType:
    ...
```

`contract_id` 是能力约定 ID。装饰器在模块导入时记录处理函数；`provider.bootstrap()` 或 `provider.run()` 才会创建服务端点、向 Atlas 声明能力并开始接收请求。`description` 可省略；显式值优先，否则读取处理函数的 docstring。装饰器会返回原函数，因此业务逻辑仍可直接单元测试。

#### MCP 长时间任务

一次调用内即可返回最终结果的能力是同步能力。导航、抓取等动作在启动后仍需持续运行时，同一个提供方应注册三条 MCP 能力约定：

| 能力约定 ID | 职责 | 最低响应要求 |
|---|---|---|
| `<id>` | 启动任务 | 返回启动结果；建议包含稳定、非空的 `run_id` |
| `<id>/status` | 查询任务状态 | 返回字符串字段 `state`，可附带字符串字段 `detail` |
| `<id>/cancel` | 请求取消任务 | 返回该能力约定定义的取消响应 |

Executor 在分发 `<id>` 前查询同一提供方的 MCP 能力目录：

- 两条子能力都不存在时，`<id>` 按同步能力执行，首次调用返回后节点结束；
- `/status` 与 `/cancel` 同时存在时，`<id>` 按异步能力执行；
- 只存在其中一条时，配置不完整，Executor 不会启动主任务，而是直接让该节点失败。

异步能力的执行顺序如下：

1. Executor 调用 `<id>` 启动任务，并从响应 JSON 中读取 `run_id`。
2. Executor 每 2 秒调用一次 `<id>/status`，直到收到终态。
3. 方案被取消且任务仍在运行时，Executor 调用 `<id>/cancel`。
4. 普通 RTDL 方案只调用 `<id>`；状态查询和取消由 Executor 管理，不应作为普通业务步骤重复编排。

`/status` 返回的 `state` 不区分大小写，可使用下列值：

| 状态 | 是否终态 | 含义 |
|---|:---:|---|
| `PENDING` | 否 | 已接受，尚未开始运行 |
| `RUNNING` | 否 | 正在运行 |
| `PAUSED` | 否 | 已暂停，尚未结束 |
| `SUCCEEDED` | 是 | 成功完成 |
| `FAILED` | 是 | 执行失败 |
| `CANCELED` / `CANCELLED` | 是 | 已取消 |
| `TIMEOUT` | 是 | 执行超时 |

缺少 `state` 或返回未知状态会使节点失败。`detail` 用于携带进度或终态说明。处理函数无法完成查询或取消时，应抛出 `RuntimeError`，不要把错误伪装成正常响应。

`run_id` 用于区分同一能力的多次运行，提供方支持并发任务时必须返回稳定、非空的 ID，并让 `/status` 与 `/cancel` 按该 ID 定位同一次运行。当前 Executor 兼容不返回 `run_id` 的旧接口：此时两条子能力收到空请求 `{}`，提供方必须明确将它解释为当前或最近一次运行；这种兼容方式不适合并发任务。

下面是接口形态的最小示例；请求和响应类型必须来自当前软件包的代码生成结果：

```python
@service.mcp("robonix/service/example/run")
def run(req: Run_Request) -> Run_Response:
    run_id = start_work(req)
    return Run_Response(accepted=True, run_id=run_id)

@service.mcp("robonix/service/example/run/status")
def run_status(req: RunStatus_Request) -> RunStatus_Response:
    state, detail = get_work_status(req.run_id)
    return RunStatus_Response(state=state, detail=detail)

@service.mcp("robonix/service/example/run/cancel")
def run_cancel(req: CancelRun_Request) -> CancelRun_Response:
    accepted = cancel_work(req.run_id)
    return CancelRun_Response(accepted=accepted)
```

三条能力分别需要与自身请求和响应匹配的 `.srv` 和能力约定 TOML。它们都使用 `mode = "rpc"` 和 MCP 传输；`/status`、`/cancel` 不是新的模式，也不是 Executor 的通用控制面接口。完整的导航实现可参考本指南第 8 节。

#### gRPC 生成代码

`rbnx codegen` 根据能力约定 TOML 中的 `mode` 和 `idl` 生成：

- 业务消息模块 `<idl-package>_pb2.py`，例如 `map_pb2.py`、`audio_pb2.py`；
- 汇总服务模块 `robonix_contracts_pb2_grpc.py`，其中包含每条能力约定的 `Stub`、`Servicer` 和注册函数；
- `robonix_contracts.proto`，可直接检查最终生成的一元或流式 RPC 形态。

业务消息模块只保存消息类。`Stub` 和 `Servicer` 必须从 `robonix_contracts_pb2_grpc.py` 导入，不能从 `<idl-package>_pb2_grpc.py` 猜测或导入。

| 输入定义 | Python 生成类型 |
|---|---|
| `capabilities/lib/<pkg>/msg/Foo.msg` | `<pkg>_pb2.Foo` |
| `capabilities/lib/<pkg>/srv/Hello.srv` | `<pkg>_pb2.Hello_Request` 与 `<pkg>_pb2.Hello_Response` |
| 能力约定 `robonix/service/example/hello` | `robonix_contracts_pb2_grpc.RobonixServiceExampleHelloStub` 与 `RobonixServiceExampleHelloServicer` |
| 上述约定引用 `Hello.srv` | Stub 和 Servicer 中的方法名为 `Hello` |

服务类名称由完整 `contract_id` 的每一段转为 PascalCase 后拼接；RPC 方法名来自 `.srv` 文件名。不要手写猜测复杂名称：构建后可在 `rbnx-build/codegen/proto_gen/robonix_contracts_pb2_grpc.py` 查到准确类名，在 `robonix_contracts.proto` 查到准确方法和 wire type。

`@provider.grpc(...)` 自动实现并注册该能力约定生成的单方法 `Servicer`，开发者只写处理函数：

```python
import grpc

@provider.grpc(contract_id, description="...")
def method(request: RequestType, context: grpc.ServicerContext) -> ResponseType:
    ...
```

处理函数的两个位置参数不是 Robonix 自定义对象：

| 参数 | 精确类型 | 用法 |
|---|---|---|
| `request` | 当前 RPC 生成的 protobuf 消息实例，继承 `google.protobuf.message.Message` | 一元请求和服务端流式响应模式中直接读取字段，例如 `request.text` |
| `request_iterator` | gRPC 提供的同步迭代器；每次迭代得到一个由 IDL 决定的 protobuf 流元素 | 客户端流式和双向流式模式中使用 `for item in request_iterator` 消费；调用方结束输入后循环自然结束 |
| `context` | gRPC 公共接口 [`grpc.ServicerContext`](https://grpc.github.io/grpc/python/grpc.html#grpc.ServicerContext) | `is_active()` 检查调用是否仍有效，`time_remaining()` 读取剩余期限，`peer()` 查看调用方，`set_code()` / `set_details()` 设置最终状态，`abort(code, details)` 立即以错误终止 RPC |

处理函数也可以只写第一个参数；框架会根据函数签名决定是否传入 `context`。需要取消检测、超时、元数据或明确错误状态时，应保留第二个参数并标注为 `grpc.ServicerContext`。不要把实际运行时的 gRPC 私有类名写进类型注解。

#### 流式模式与 IDL 约束

一元 RPC 和三种流式 RPC 都使用 `.srv`。`---` 上方是请求方向，下方是响应方向。流式方向必须恰好包含一个字段，而且该字段必须引用一个具名 ROS 消息类型；这个字段所引用的消息就是线上逐条传输的流元素。代码生成器会拒绝字段数不为 1 或以 primitive 作为流元素的定义。

| `mode` | 生成的 gRPC 形态 | `.srv` 约束 | 提供方处理函数 |
|---|---|---|---|
| `rpc` | `M(Request) returns (Response)` | 请求、响应字段数不限 | `(request, context) -> Response` |
| `rpc_server_stream` | `M(Request) returns (stream Item)` | 响应段恰好一个具名消息字段 | `(request, context) -> Iterator[Item]`，用 `yield` 发送 |
| `rpc_client_stream` | `M(stream Item) returns (Response)` | 请求段恰好一个具名消息字段 | `(request_iterator, context) -> Response` |
| `rpc_bidirectional_stream` | `M(stream In) returns (stream Out)` | 请求、响应段各恰好一个具名消息字段 | `(request_iterator, context) -> Iterator[Out]`，边读取边 `yield` |

`rpc_server_stream` 的非流式请求段如果为空或包含多个字段，参数类型是 `<SrvName>_Request`；如果请求段恰好只有一个具名消息字段，当前代码生成器会直接使用该字段的消息类型。`rpc_client_stream` 的非流式响应段有字段时返回 `<SrvName>_Response`，为空时返回 `google.protobuf.empty_pb2.Empty`。

`topic_out` 和 `topic_in` 使用 `.msg` 而不是 `.srv`。当选择 gRPC 传输时，代码生成器分别把它们映射成 `Empty -> stream Message` 和 `stream Message -> Empty`；选择 ROS 2 传输时，由提供方创建 publisher/subscription，再向 Atlas 声明端点。

下面四组示例使用主仓库中真实存在的能力约定。完整的 gRPC Python 调用语义也可参见 [gRPC Python 基础教程](https://grpc.io/docs/languages/python/basics/)。

#### 一元 RPC

`robonix/service/map/get_mode` 使用 `map/srv/GetMode.srv`，生成：

```proto
rpc GetMode(robonix.map.GetMode_Request)
    returns (robonix.map.GetMode_Response);
```

提供方实现：

```python
import grpc
import map_pb2

@provider.grpc("robonix/service/map/get_mode")
def get_mode(
    request: map_pb2.GetMode_Request,
    context: grpc.ServicerContext,
) -> map_pb2.GetMode_Response:
    del request, context
    return map_pb2.GetMode_Response(
        ok=True,
        mode="mapping",
        detail="",
    )
```

调用方先通过 Atlas 选择提供方并登记连接，再用生成的 `Stub` 调用：

```python
import grpc
import map_pb2
import robonix_contracts_pb2_grpc as contracts_grpc

from robonix_api import ATLAS
from robonix_api.atlas_types import Transport

target = ATLAS.find_unique_capability(
    contract_id="robonix/service/map/get_mode",
    transport=Transport.GRPC,
)
edge = consumer.connect_capability(
    target,
    "robonix/service/map/get_mode",
    Transport.GRPC,
)
try:
    with grpc.insecure_channel(edge.endpoint) as rpc_channel:
        stub = contracts_grpc.RobonixServiceMapGetModeStub(rpc_channel)
        response = stub.GetMode(map_pb2.GetMode_Request(), timeout=5.0)
        print(response.mode)
finally:
    edge.close()
```

`edge` 是 Atlas 中的逻辑连接，`rpc_channel` 是真正承载消息的 gRPC channel；两者都要关闭。

#### 服务端流式响应

`robonix/service/speech/tts_stream` 的请求段包含多个参数，响应段只有 `tts/SynthesizeAudioChunk chunk`。因此生成：

```proto
rpc SynthesizeStream(robonix.tts.SynthesizeStream_Request)
    returns (stream robonix.tts.SynthesizeAudioChunk);
```

提供方每次 `yield` 一个响应元素；函数结束即关闭响应流：

```python
from collections.abc import Iterator

import audio_pb2
import grpc
import tts_pb2

@provider.grpc("robonix/service/speech/tts_stream")
def synthesize_stream(
    request: tts_pb2.SynthesizeStream_Request,
    context: grpc.ServicerContext,
) -> Iterator[tts_pb2.SynthesizeAudioChunk]:
    for sequence, data in enumerate(
        synthesizer.generate(request.text, voice=request.voice)
    ):
        if not context.is_active():
            return
        yield tts_pb2.SynthesizeAudioChunk(
            chunk=audio_pb2.AudioChunk(data=data, sequence=sequence),
            encoding="pcm_s16le",
            sample_rate_hz=16000,
        )
```

调用方得到一个响应迭代器：

```python
import tts_pb2
import robonix_contracts_pb2_grpc as contracts_grpc

stub = contracts_grpc.RobonixServiceSpeechTtsStreamStub(rpc_channel)
request = tts_pb2.SynthesizeStream_Request(text="你好")
for chunk in stub.SynthesizeStream(request, timeout=30.0):
    speaker.write(chunk.chunk.data)
```

#### 客户端流式请求

`robonix/service/speech/wake_word` 的请求段只有 `audio/msg/AudioChunk chunk`，因此发送的是裸 `audio_pb2.AudioChunk`，不是 `DetectWakeWord_Request` 包装器：

```proto
rpc DetectWakeWord(stream robonix.audio.AudioChunk)
    returns (robonix.speech.DetectWakeWord_Response);
```

提供方消费请求迭代器，输入关闭后返回一个响应：

```python
from collections.abc import Iterator

import audio_pb2
import grpc
import speech_pb2

@provider.grpc("robonix/service/speech/wake_word")
def detect_wake_word(
    request_iterator: Iterator[audio_pb2.AudioChunk],
    context: grpc.ServicerContext,
) -> speech_pb2.DetectWakeWord_Response:
    keyword = detector.detect(
        bytes(chunk.data)
        for chunk in request_iterator
        if context.is_active() and chunk.data
    )
    return speech_pb2.DetectWakeWord_Response(
        detected=bool(keyword),
        keyword=keyword or "",
    )
```

调用方把任意可迭代对象传给 Stub；迭代结束表示客户端完成发送：

```python
import audio_pb2
import robonix_contracts_pb2_grpc as contracts_grpc

def audio_chunks():
    for sequence, pcm in enumerate(microphone.frames()):
        yield audio_pb2.AudioChunk(data=pcm, sequence=sequence)

stub = contracts_grpc.RobonixServiceSpeechWakeWordStub(rpc_channel)
response = stub.DetectWakeWord(audio_chunks(), timeout=30.0)
print(response.keyword)
```

#### 双向流式 RPC

`robonix/service/speech/asr_stream` 的请求段和响应段各有一个具名消息字段，因此生成：

```proto
rpc RecognizeStream(stream robonix.asr.AsrAudioChunk)
    returns (stream robonix.asr.RecognizeStreamEvent);
```

提供方可以消费若干输入后产生一个结果；两侧消息不要求一一对应：

```python
from collections.abc import Iterator

import asr_pb2
import grpc

@provider.grpc("robonix/service/speech/asr_stream")
def recognize_stream(
    request_iterator: Iterator[asr_pb2.AsrAudioChunk],
    context: grpc.ServicerContext,
) -> Iterator[asr_pb2.RecognizeStreamEvent]:
    decoder = StreamingDecoder()
    for request in request_iterator:
        if not context.is_active():
            return
        decoder.feed(request.chunk.data)
        partial = decoder.partial_text()
        if partial:
            yield asr_pb2.RecognizeStreamEvent(
                event_type=asr_pb2.RecognizeStreamEvent.PARTIAL,
                text=partial,
                is_final=False,
            )
    yield asr_pb2.RecognizeStreamEvent(
        event_type=asr_pb2.RecognizeStreamEvent.FINAL,
        text=decoder.final_text(),
        is_final=True,
    )
```

调用方传入请求迭代器，同时遍历返回的响应迭代器：

```python
import asr_pb2
import audio_pb2
import robonix_contracts_pb2_grpc as contracts_grpc

def asr_requests():
    for sequence, pcm in enumerate(microphone.frames()):
        yield asr_pb2.AsrAudioChunk(
            chunk=audio_pb2.AudioChunk(data=pcm, sequence=sequence)
        )

stub = contracts_grpc.RobonixServiceSpeechAsrStreamStub(rpc_channel)
for event in stub.RecognizeStream(asr_requests(), timeout=60.0):
    print(event.text, event.is_final)
```

建立三类流式调用的 Atlas 连接和 `rpc_channel` 与一元 RPC 示例相同。示例中的 `synthesizer`、`detector`、`decoder`、`microphone` 和 `speaker` 是软件包自己的业务对象；Robonix 固定的是处理函数形态、生成消息类型、Stub 以及能力发现和连接流程。服务端需要返回错误时，优先使用 `context.abort(grpc.StatusCode.INVALID_ARGUMENT, "...")`；该调用会抛出终止 RPC 的异常，因此后面不应再 `return` 或 `yield`。

### 14.6 关闭行为

装饰器只在导入阶段登记处理函数，不会自行监听端口。`provider.bootstrap()` 按以下顺序完成非阻塞启动：

1. 向 Atlas 注册提供方；
2. 创建共享 `robonix/lifecycle/driver`，并把 `@provider.grpc` 处理函数绑定到生成的 Servicer；
3. 监听 gRPC 端口并向 Atlas 声明这些能力；
4. 启动 MCP HTTP 端点并声明 `@provider.mcp` 能力；
5. 启动心跳；
6. 等待启动器通过 Driver 发送初始化和激活命令。回调缺失时 Driver 记录警告并执行空操作；提供方仍保留唯一的共享 Driver。

:::warning[后向兼容：已有命名空间 Driver]
旧软件包若在清单中精确声明 `<provider-namespace>/driver`，则继续按[兼容流程](integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)绑定完整的旧生成服务，或在满足单向迁移条件时使用共享运行时 Driver。该方式计划迁移；新软件包不要照此创建 Driver TOML。
:::

`provider.run()` 先执行同一套 bootstrap，再阻塞等待退出信号。Driver 的 `CMD_SHUTDOWN` 会先完成业务关闭回调和响应，再停止提供方；进程信号路径也会调用 `on_shutdown`。框架会关闭通过 `connect_capability` 建立的通道、受管子进程和 gRPC server，但软件包自行创建的线程、设备句柄和后台任务仍必须在生命周期回调中释放。

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
| `rbnx boot [-v] [-f <manifest>]` | 启动整套部署；`-v` 关闭动态启动动画并实时输出 INFO/WARN/ERROR |
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
| `rbnx logs [-d <dir>] [-t <tag>] [-l <level>] [-f] [--json]` | 读取、筛选或跟随 Scribe 结构化日志 |

`rbnx clean -f robonix_manifest.yaml` 默认保留 `rbnx-boot/cache/`；只有加 `--cache` 才删除远程软件包缓存。单独执行 `rbnx start --config <file>` 时仍会先读取软件包清单；启动器确认提供方只注册唯一的共享生命周期 Driver 后，通过 `CMD_INIT` 发送合并配置。配置文件路径不会暴露给提供方进程。

:::warning[后向兼容：`rbnx start` 启动旧软件包]
精确声明 `<provider-namespace>/driver` 和本地 Driver TOML 的旧软件包仍可使用 `rbnx start`，也可在旧生成服务完全不存在时按受管兼容标记单向使用共享运行时 Driver。该方式计划迁移，不能与共享 Driver 条目同时使用。完整规则见[软件包与部署清单规范](integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

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
| `capabilities` | 能力约定引用和预期导出清单；每项含约定 ID `name`，可选业务约定 TOML `path`，不替代码声明普通业务能力。未列 Driver 时，框架自动选择共享 `robonix/lifecycle/driver`；显式选择共享 Driver 仍受支持，多条 Driver 会使启动失败 |
| `depends` | 依赖元数据；解析、安装和构建排序仍在设计中，当前只展示记录，不自动获取、安装或注入路径 |

:::warning[后向兼容：`capabilities` 中的命名空间 Driver]
旧软件包可暂时在 `capabilities` 中保留唯一的精确 `<provider-namespace>/driver` 及其本地 TOML `path`，但计划迁移到共享 Driver。完整兼容和迁移规则见[软件包与部署清单规范](integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

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
