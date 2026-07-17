# 以抓积木为例接入现有 Python 功能

本页只讨论一个具体案例：已有 Python 程序能够通过厂商 SDK 驱动灵巧手抓取积木，现在要把这项功能接入 Robonix。示例假设目标积木已经完成识别和定位，重点说明硬件控制与抓取流程如何接入；如果原程序还直接访问相机或力传感器，也要分别接入对应原语。

## 1. 先确定边界

职责边界固定如下：

- 连接灵巧手、发送关节命令、读取关节状态属于**原语**（Primitive）。
- 根据已定位的目标生成动作序列并完成一次抓取积木属于**技能**（Skill）。
- 机器人部署仓库只选择这两个软件包，并填写本体相关配置。
- Robonix 主仓库提供运行时、Python 接口和标准能力约定，不保存厂商 SDK 或抓取算法。

现有脚本可以保留抓取流程，但直接调用 SDK 的代码必须下沉到原语。技能只能通过标准机械臂能力控制硬件。

## 2. 仓库关系

推荐发布为三个独立仓库：

| 仓库 | 保存的内容 |
|---|---|
| `primitive-<vendor>-hand-rbnx` | 厂商 SDK 适配、关节命令、关节状态、设备关闭与安全限制 |
| `skill-grasp-block-rbnx` | 原抓取算法、任务输入输出，以及对机械臂原语的调用 |
| `robot-<vendor>-<model>` | 整机 URDF、Soma、部署清单和本体配置 |

不需要把原抓取仓库复制进 `syswonder/robonix`。如果原仓库只维护抓取功能，可以保留其 Git 历史并将它整理为 `skill-grasp-block-rbnx`；厂商 SDK 适配放到新的原语仓库。

最短的标准迁移路径是：

1. 在原抓取仓库中，把算法使用的 SDK 调用收敛成 `HandPort`；
2. 新建灵巧手原语仓库，用标准关节命令和关节状态能力包装厂商 SDK；
3. 在原抓取仓库中加入技能清单、能力约定和 Robonix 入口；
4. 让技能通过 Atlas 连接灵巧手原语，不再导入厂商 SDK；
5. 在机器人部署仓库中引用两个软件包并完成整机验收。

如果原脚本还直接访问相机、力传感器或其他设备，也要按同一原则改为对应标准原语；不能只拆灵巧手。

## 3. 整理现有抓取代码

假设原脚本类似：

```python
from vendor_sdk import Hand

hand = Hand("192.168.1.20")
hand.move_joints(names, positions)
```

先把抓取算法改为依赖一个很小的硬件接口，不再导入厂商 SDK：

```python title="grasp_block/grasp_core.py"
from typing import Protocol

# 改成原项目中真实的目标读取、轨迹生成和结果确认函数。
from existing_project.grasp import (
    load_target_pose,
    plan_grasp_sequence,
    verify_grasp_result,
)


class HandPort(Protocol):
    def move_joints(
        self,
        names: list[str],
        positions: list[float],
        tolerances: list[float],
        timeout_sec: float = 5.0,
    ) -> None: ...

    def joint_positions(self) -> dict[str, float]: ...


def grasp_block(hand: HandPort, target: str) -> tuple[bool, str]:
    target_pose = load_target_pose(target)
    for step in plan_grasp_sequence(target_pose):
        hand.move_joints(
            step.names,
            step.positions,
            step.tolerances,
            timeout_sec=step.timeout_sec,
        )
    success = verify_grasp_result(target, hand.joint_positions())
    return success, f"target={target}, grasp_confirmed={success}"
```

`load_target_pose`、`plan_grasp_sequence` 和 `verify_grasp_result` 代表原项目中已经验证过的目标读取、接近、闭合、抬起与结果确认逻辑；接入时保留这些实现，只把每一步中的 SDK 调用改成 `HandPort`。`step.tolerances` 来自本体标定，旋转关节使用弧度，移动关节使用米。原脚本仍可使用一个 SDK 版 `HandPort` 做回归；Robonix 技能使用下一节的运行时版本。

## 4. 将厂商 SDK 接入机械臂原语

从一个独立仓库创建原语骨架：

```bash
rbnx package-new vendor_hand --type primitive --path ./primitive-vendor-hand-rbnx
```

把生成清单中的 `package.name`、`version`、`description`、`license`、`tags` 和 `maintainers` 改成原语仓库的真实信息，不要保留 `TODO` 或示例维护者。

完成接入后的原语仓库至少包含：

```text
primitive-vendor-hand-rbnx/
├── vendor_hand/
│   ├── __init__.py
│   └── main.py
├── capabilities/
├── scripts/
│   ├── build.sh
│   └── start.sh
├── package_manifest.yaml
└── config.spec
```

将生成的 `package_manifest.yaml` 中 `capabilities` 段替换为：

```yaml title="package_manifest.yaml 中的 capabilities"
capabilities:
  - name: robonix/primitive/arm/driver
  - name: robonix/primitive/arm/joint_command
  - name: robonix/primitive/arm/joint_states
```

三个约定分别负责生命周期、关节命令输入和关节状态输出。关节名、位置、速度和作用力使用 `sensor_msgs/JointState`；其中 `effort` 对旋转关节表示力矩，对移动关节表示力。夹爪关节也放在同一消息中。

原语的关键适配代码如下：

```python title="vendor_hand/main.py"
import os
import threading

from robonix_api import Err, Ok, Primitive
from sensor_msgs.msg import JointState
from vendor_sdk import Hand

provider = Primitive(
    id=os.environ.get("RBNX_INSTANCE_NAME", "vendor_hand"),
    namespace="robonix/primitive/arm",
)
settings: dict = {}
sdk: Hand | None = None
active = False
ros_bound = False
hardware_lock = threading.Lock()
state_stop = threading.Event()
state_thread: threading.Thread | None = None


def handle_joint_command(msg: JointState) -> None:
    if not msg.name or len(msg.name) != len(msg.position):
        return
    with hardware_lock:
        if not active or sdk is None:
            return
        # 把这一行替换成厂商 SDK 的非阻塞关节目标命令。
        sdk.send_joint_targets(list(msg.name), list(msg.position))


def publish_joint_states() -> None:
    period = float(settings.get("state_period_sec", 0.02))
    while not state_stop.wait(period):
        with hardware_lock:
            if not active or sdk is None:
                continue
            # 该 SDK 读取必须带有限超时；按真实 SDK 字段完成转换。
            state = sdk.read_joint_state(timeout_sec=period)

        msg = JointState()
        msg.name = list(state.names)
        msg.position = list(state.positions)
        msg.velocity = list(state.velocities)
        msg.effort = list(state.efforts)
        if len(msg.name) != len(msg.position):
            continue
        provider.emit("robonix/primitive/arm/joint_states", msg)


@provider.on_init
def init(cfg: dict):
    settings.clear()
    settings.update(cfg)
    device_ip = cfg.get("device_ip")
    if not device_ip:
        return Err("config.device_ip is required")
    if float(cfg.get("state_period_sec", 0.02)) <= 0:
        return Err("config.state_period_sec must be positive")
    return Ok()


@provider.on_activate
def activate():
    global active, ros_bound, sdk, state_thread
    if active:
        return Ok()
    candidate = None
    try:
        candidate = Hand(settings["device_ip"])
        if not ros_bound:
            provider.create_subscription(
                "robonix/primitive/arm/joint_command",
                topic=settings.get("command_topic", "/vendor_hand/joint_command"),
                msg_type=JointState,
                callback=handle_joint_command,
                qos="reliable",
            )
            provider.create_publisher(
                "robonix/primitive/arm/joint_states",
                topic=settings.get("state_topic", "/vendor_hand/joint_states"),
                msg_type=JointState,
                qos="best_effort",
            )
            ros_bound = True
        with hardware_lock:
            sdk = candidate
            active = True
        state_stop.clear()
        state_thread = threading.Thread(target=publish_joint_states, daemon=True)
        state_thread.start()
        return Ok()
    except Exception as exc:
        if candidate is not None:
            try:
                candidate.stop()
                candidate.close()
            except Exception:
                pass
        with hardware_lock:
            active = False
            sdk = None
        return Err(f"failed to activate vendor hand: {exc}")


@provider.on_deactivate
def deactivate():
    global active, sdk, state_thread
    with hardware_lock:
        active = False
        current = sdk
    if current is not None:
        current.stop()
    state_stop.set()
    if state_thread is not None:
        state_thread.join(timeout=2.0)
        if state_thread.is_alive():
            return Err("joint-state worker did not stop within 2 seconds")
        state_thread = None
    with hardware_lock:
        if sdk is current and current is not None:
            current.close()
            sdk = None
    return Ok()


@provider.on_shutdown
def shutdown():
    return deactivate()


if __name__ == "__main__":
    provider.run()
```

`on_init` 只校验并保存配置；真正的 SDK 连接在 `on_activate` 建立。状态线程持续把 SDK 反馈转换成 `JointState`，技能才能判断动作是否完成。命令回调与停用流程使用同一把锁，避免关闭设备时仍有命令进入。示例中的 ROS 实体只创建一次并由 `active` 门控，避免重复激活时重复订阅。完整原语还必须实现命令限位、输入超时停止、重复关闭安全和 SDK 异常处理。公开配置写入根目录 `config.spec`，不要把 IP、标定路径或关节名写死在代码中。

```yaml title="config.spec"
# 本文件说明实例 config 的字段，不会作为运行时 schema 自动加载。
config:
  # 必填，非空字符串。厂商 SDK 连接设备时使用的网络地址。
  device_ip: ""

  # ROS 2 绝对话题名；下列值也是默认值。
  command_topic: /vendor_hand/joint_command
  state_topic: /vendor_hand/joint_states

  # 状态读取周期，单位为秒；必须大于 0。
  state_period_sec: 0.02
```

该原语使用 ROS 2 传输，因此构建和启动脚本必须生成、编译并加载 Robonix ROS 2 叠加层；可直接采用[开发原语](../developer-guide.md#9-开发原语)中的脚本。

## 5. 将抓积木流程接入技能

原抓取仓库已经存在，不能对同一目录运行 `rbnx package-new`。保留原算法模块，在仓库根目录补齐下面的 Robonix 文件：

```text
skill-grasp-block-rbnx/
├── grasp_block/
│   ├── __init__.py
│   ├── grasp_core.py
│   └── main.py
├── capabilities/
│   ├── driver.v1.toml
│   ├── grasp.v1.toml
│   ├── grasp_status.v1.toml
│   ├── grasp_cancel.v1.toml
│   └── lib/grasp_block/srv/
│       ├── GraspBlock.srv
│       ├── GetGraspStatus.srv
│       └── CancelGrasp.srv
├── scripts/
│   ├── build.sh
│   └── start.sh
├── package_manifest.yaml
├── config.spec
└── CAPABILITY.md
```

如果希望复制脚手架中的基础文件，应在空的临时目录生成，再把所需文件合入现有仓库；不要覆盖原算法：

```bash
rbnx package-new grasp_block --type skill --path /tmp/grasp-block-scaffold
```

把生成清单中的 `package.name`、`version`、`description`、`license`、`tags` 和 `maintainers` 改成该仓库的真实信息，不要保留 `TODO` 或示例维护者。

一次抓取会持续一段时间，不能把整个动作写成同步 MCP 调用。技能要提供 `grasp`、`grasp/status` 和 `grasp/cancel` 三条能力：首次调用立即返回 `run_id`，Executor 根据 `run_id` 自动轮询状态，并在方案取消时调用取消能力。三个接口分别定义为：

```text title="capabilities/lib/grasp_block/srv/GraspBlock.srv"
string target
---
string run_id
string detail
```

```text title="capabilities/lib/grasp_block/srv/GetGraspStatus.srv"
string run_id
---
string state
string detail
```

```text title="capabilities/lib/grasp_block/srv/CancelGrasp.srv"
string run_id
---
bool accepted
string detail
```

抓积木不是全局标准接口，因此三条能力约定保存在技能软件包中。基础约定如下，状态与取消约定使用相同结构，只替换 `id`、`idl` 和 `description`：

```toml title="capabilities/grasp.v1.toml"
[contract]
id = "robonix/skill/grasp_block/grasp"
version = "1"
kind = "skill"
idl = "grasp_block/srv/GraspBlock.srv"
description = "Start grasping one previously located block."

[mode]
type = "rpc"
```

```toml title="capabilities/grasp_status.v1.toml"
[contract]
id = "robonix/skill/grasp_block/grasp/status"
version = "1"
kind = "skill"
idl = "grasp_block/srv/GetGraspStatus.srv"
description = "Read the state of a grasp run."

[mode]
type = "rpc"
```

```toml title="capabilities/grasp_cancel.v1.toml"
[contract]
id = "robonix/skill/grasp_block/grasp/cancel"
version = "1"
kind = "skill"
idl = "grasp_block/srv/CancelGrasp.srv"
description = "Cancel a grasp run."

[mode]
type = "rpc"
```

技能清单还要引用同命名空间的 Driver 约定，使 Executor 能在第一次调用前激活技能。按照[开发技能](../developer-guide.md#10-开发技能)创建 `driver.v1.toml`，然后将以下片段写入清单：

```yaml title="package_manifest.yaml 中的 capabilities"
capabilities:
  - name: robonix/skill/grasp_block/driver
    path: capabilities/driver.v1.toml
  - name: robonix/skill/grasp_block/grasp
    path: capabilities/grasp.v1.toml
  - name: robonix/skill/grasp_block/grasp/status
    path: capabilities/grasp_status.v1.toml
  - name: robonix/skill/grasp_block/grasp/cancel
    path: capabilities/grasp_cancel.v1.toml
```

技能的 `config.spec` 只声明它需要哪个机械臂提供方，不包含设备 IP：

```yaml title="config.spec"
# 本文件说明实例 config 的字段，不会作为运行时 schema 自动加载。
config:
  # 必填，非空字符串。机械臂原语在 Atlas 中注册的提供方 ID。
  arm_provider_id: ""
```

`CAPABILITY.md` 向模型和开发者说明任务边界：

```markdown title="CAPABILITY.md"
---
description: 抓取一个已经识别并定位的积木，并支持状态查询与取消。
---

# 抓积木

抓取一个已经识别并定位的积木。

- 输入：场景中已经解析完成的目标积木 ID。
- 依赖：`arm_provider_id` 指向的机械臂必须在线并可接收关节命令。
- 成功：技能完成抓取，并由原项目的结果确认逻辑验证目标已被夹持。
- 取消：技能停止后续动作；机械臂原语负责看门狗、限位和当前硬件运动的安全停止。
```

该技能既生成 MCP 请求/响应类型，又使用 ROS 2 的 `JointState`。它的 `scripts/build.sh` 和 `scripts/start.sh` 采用[开发原语](../developer-guide.md#9-开发原语)中的叠加层脚本：代码生成参数保持 `--mcp --ros2`，并把启动命令改为 `python3 -m grasp_block.main`。不能只运行 `rbnx codegen` 而不执行 `colcon build`，也不能在启动时省略生成叠加层的 `setup.bash`。

业务入口使用 MCP，因为当前 Pilot 和 Executor 通过 MCP 调用模型可见工具。`rpc` 在这里表示请求—响应语义，不表示必须使用 gRPC。`status` 与 `cancel` 可能出现在工具目录中，但应用开发者不应把它们手工写进普通业务计划：Executor 会自动轮询状态，根级方案取消会由 Executor 转发到该抓取任务。

下面的入口展示完整的运行状态和取消链路；`grasp_core.py` 仍使用第 3 节中现有项目的抓取算法。

```python title="grasp_block/main.py"
from __future__ import annotations

from dataclasses import dataclass, field
import os
import threading
import time
import uuid

from robonix_api import ATLAS, Deferred, Err, Ok, Skill
from robonix_api.atlas_types import Transport
from grasp_block.grasp_core import grasp_block
from grasp_block_mcp import (
    CancelGrasp_Request,
    CancelGrasp_Response,
    GetGraspStatus_Request,
    GetGraspStatus_Response,
    GraspBlock_Request,
    GraspBlock_Response,
)
from sensor_msgs.msg import JointState

skill = Skill(
    id=os.environ.get("RBNX_INSTANCE_NAME", "grasp_block"),
    namespace="robonix/skill/grasp_block",
)
arm_provider_id = ""
latest_positions: dict[str, float] = {}
state_version = 0
command_channel = None
state_channel = None
active = False
lifecycle_stop = threading.Event()
state_condition = threading.Condition()
runs_lock = threading.Lock()


@dataclass
class GraspRun:
    run_id: str
    target: str
    state: str = "PENDING"
    detail: str = "queued"
    cancel_event: threading.Event = field(default_factory=threading.Event)
    thread: threading.Thread | None = None


runs: dict[str, GraspRun] = {}
TERMINAL_STATES = {"SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"}


class GraspCanceled(RuntimeError):
    pass


def update_state(msg: JointState) -> None:
    global state_version
    if len(msg.name) != len(msg.position):
        return
    with state_condition:
        latest_positions.clear()
        latest_positions.update(zip(msg.name, msg.position, strict=True))
        state_version += 1
        state_condition.notify_all()


class RobonixHand:
    def __init__(self, run: GraspRun) -> None:
        self.run = run

    def move_joints(
        self,
        names: list[str],
        positions: list[float],
        tolerances: list[float],
        timeout_sec: float = 5.0,
    ) -> None:
        if not names or len(names) != len(positions) or len(names) != len(tolerances):
            raise ValueError("names, positions and tolerances must have equal length")
        msg = JointState()
        msg.name = names
        msg.position = positions
        with state_condition:
            version_before_command = state_version
        skill.emit("robonix/primitive/arm/joint_command", msg)

        deadline = time.monotonic() + timeout_sec
        with state_condition:
            while True:
                if self.run.cancel_event.is_set() or lifecycle_stop.is_set():
                    raise GraspCanceled("grasp canceled")
                reached = state_version > version_before_command and all(
                    abs(latest_positions.get(name, float("inf")) - target) <= tolerance
                    for name, target, tolerance in zip(
                        names, positions, tolerances, strict=True
                    )
                )
                if reached:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("arm did not reach the joint target")
                state_condition.wait(timeout=remaining)

    def joint_positions(self) -> dict[str, float]:
        with state_condition:
            return dict(latest_positions)


def set_run_state(run: GraspRun, state: str, detail: str) -> None:
    with runs_lock:
        run.state = state
        run.detail = detail


def run_grasp(run: GraspRun) -> None:
    set_run_state(run, "RUNNING", "grasping block")
    try:
        success, detail = grasp_block(RobonixHand(run), run.target)
        if run.cancel_event.is_set() or lifecycle_stop.is_set():
            set_run_state(run, "CANCELED", "grasp canceled")
        elif success:
            set_run_state(run, "SUCCEEDED", detail)
        else:
            set_run_state(run, "FAILED", detail)
    except GraspCanceled as exc:
        set_run_state(run, "CANCELED", str(exc))
    except TimeoutError as exc:
        set_run_state(run, "TIMEOUT", str(exc))
    except Exception as exc:
        set_run_state(run, "FAILED", f"grasp failed: {exc}")


@skill.on_init
def init(cfg: dict):
    global arm_provider_id
    arm_provider_id = cfg.get("arm_provider_id", "")
    if not arm_provider_id:
        return Err("config.arm_provider_id is required")
    return Ok()


@skill.on_activate
def activate():
    global active, command_channel, state_channel
    if command_channel is not None and state_channel is not None:
        active = True
        lifecycle_stop.clear()
        return Ok()

    commands = ATLAS.find_capability(
        contract_id="robonix/primitive/arm/joint_command",
        transport=Transport.ROS2,
        provider_id=arm_provider_id,
    )
    states = ATLAS.find_capability(
        contract_id="robonix/primitive/arm/joint_states",
        transport=Transport.ROS2,
        provider_id=arm_provider_id,
    )
    if len(commands) != 1 or len(states) != 1:
        return Deferred(f"arm provider {arm_provider_id!r} is not ready")

    command_channel = skill.connect_capability(
        commands[0], "robonix/primitive/arm/joint_command", Transport.ROS2
    )
    state_channel = skill.connect_capability(
        states[0], "robonix/primitive/arm/joint_states", Transport.ROS2
    )
    skill.create_publisher(
        "robonix/primitive/arm/joint_command",
        topic=command_channel.endpoint,
        msg_type=JointState,
        qos="reliable",
        declare=False,
    )
    skill.create_subscription(
        "robonix/primitive/arm/joint_states",
        topic=state_channel.endpoint,
        msg_type=JointState,
        callback=update_state,
        qos="best_effort",
        declare=False,
    )
    active = True
    lifecycle_stop.clear()
    return Ok()


@skill.on_deactivate
def deactivate():
    global active
    active = False
    lifecycle_stop.set()
    with runs_lock:
        running = [run for run in runs.values() if run.state not in TERMINAL_STATES]
        for run in running:
            run.cancel_event.set()
    with state_condition:
        state_condition.notify_all()
    for run in running:
        if run.thread is not None:
            run.thread.join(timeout=2.0)
    if any(run.thread is not None and run.thread.is_alive() for run in running):
        return Err("grasp worker did not stop within 2 seconds")
    return Ok()


@skill.on_shutdown
def shutdown():
    return deactivate()


@skill.mcp(
    "robonix/skill/grasp_block/grasp",
    description="Start grasping one previously located block.",
)
def start_grasp(req: GraspBlock_Request) -> GraspBlock_Response:
    if not active:
        raise RuntimeError("grasp skill is not active")
    target = req.target.strip()
    if not target:
        raise RuntimeError("target is required")
    run_id = f"grasp-{uuid.uuid4().hex[:12]}"
    run = GraspRun(run_id=run_id, target=target)
    run.thread = threading.Thread(target=run_grasp, args=(run,), daemon=True)
    with runs_lock:
        if any(run.state not in TERMINAL_STATES for run in runs.values()):
            raise RuntimeError("the configured arm is already running a grasp")
        runs[run_id] = run
    run.thread.start()
    return GraspBlock_Response(run_id=run_id, detail="grasp started")


@skill.mcp(
    "robonix/skill/grasp_block/grasp/status",
    description="Read the state of a grasp run.",
)
def grasp_status(req: GetGraspStatus_Request) -> GetGraspStatus_Response:
    with runs_lock:
        run = runs.get(req.run_id)
        if run is None:
            raise RuntimeError(f"unknown grasp run: {req.run_id}")
        return GetGraspStatus_Response(state=run.state, detail=run.detail)


@skill.mcp(
    "robonix/skill/grasp_block/grasp/cancel",
    description="Cancel a grasp run.",
)
def cancel_grasp(req: CancelGrasp_Request) -> CancelGrasp_Response:
    with runs_lock:
        run = runs.get(req.run_id)
        if run is None:
            raise RuntimeError(f"unknown grasp run: {req.run_id}")
        if run.state not in TERMINAL_STATES:
            run.cancel_event.set()
    with state_condition:
        state_condition.notify_all()
    return CancelGrasp_Response(accepted=True, detail="cancel requested")


if __name__ == "__main__":
    skill.run()
```

开始接口必须迅速返回非空 `run_id`，状态接口必须返回 `PENDING`、`RUNNING`、`SUCCEEDED`、`FAILED`、`CANCELED`、`TIMEOUT` 或 `PAUSED` 之一。未知 `run_id` 和无法开始的请求要抛出错误，不能返回一个看似成功但 `run_id` 为空的响应。取消处理必须幂等，并最终让状态收敛到 `CANCELED`。

示例只允许技能停止后续关节命令。当前硬件动作是否能立刻停止由机械臂原语负责，因此原语必须实现命令看门狗、限位和厂商 SDK 的停止调用。真实抓取还应按设备能力增加速度、作用力、碰撞和夹持状态检查；不要用固定 `sleep` 代替状态确认。

## 6. 在机器人部署仓库中组装

部署仓库不保存上述实现。把下面两个实例分别追加到现有 `primitive` 和 `skill` 列表；不要用该片段覆盖完整清单：

```yaml title="robonix_manifest.yaml 中的实例"
primitive:
  - name: dexterous_hand
    url: https://github.com/<organization>/primitive-<vendor>-hand-rbnx
    branch: main
    config:
      device_ip: 192.168.1.20
      command_topic: /dexterous_hand/joint_command
      state_topic: /dexterous_hand/joint_states

skill:
  - name: grasp_block
    url: https://github.com/<organization>/skill-grasp-block-rbnx
    branch: main
    config:
      arm_provider_id: dexterous_hand
```

部署实例 `name` 是运行时提供方 ID。技能通过 `arm_provider_id` 选择要控制的机械臂；它不使用厂商 IP，也不导入厂商 SDK。

## 7. 构建与验收

先分别在两个软件包根目录执行：

```bash
rbnx validate .
rbnx build -p .
```

再在机器人部署仓库构建和启动：

```bash
rbnx build -f robonix_manifest.yaml
rbnx boot -f robonix_manifest.yaml
```

另开终端检查：

```bash
rbnx caps -v
rbnx tools
rbnx describe --provider grasp_block
```

验收结果必须满足：

1. 导入模块和启动技能不会触发机械臂运动；
2. `dexterous_hand` 为 `ACTIVE`，并声明关节命令和关节状态能力；
3. `grasp_block` 首次调用前为 `INACTIVE`，`rbnx tools` 能看到抓取工具；
4. 第一次自然语言调用只激活一次技能、只执行一次抓取，并返回真实结果；
5. 抓取技能不导入厂商 SDK，切换机械臂时只修改部署绑定；
6. 取消、停用或关闭部署后，技能停止下发命令，原语停止硬件输出并释放 SDK 资源。

完成以上步骤后，原有“固定脚本”就成为可发现、可配置、可替换硬件且受生命周期管理的 Robonix 功能。
