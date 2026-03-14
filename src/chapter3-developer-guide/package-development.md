# Package 开发指南

<!-- toc -->

本文档详细说明：如何从零创建一个 Robonix package、如何根据 RIDL 找到生成代码与函数名、如何在 server/client 中补全业务逻辑、如何用 rbnx 构建与运行。按步骤操作即可完成一个可运行的 package。

---

## 1. 概念与前置条件

### 1.1 Package 是什么

- Package = 一个可被 rbnx 单独构建、启动、停止的应用单元。
- 根目录必须有 robonix_manifest.yaml，其中声明 package 身份（id、name、version 等）和 node 列表（每个 node 有一个启动 entry，如 `模块:函数`）。
- Node = 运行起来的一个进程；node id（在 manifest 的 `nodes[].id`）即通信时的目标标识，建议使用 `com.syswonder.xxx` 这类唯一 ID。
- 构建与启停只通过 rbnx：`rbnx build -p <path>` 或 `rbnx build -g <name>`（-p 本地路径，-g 系统已安装包名）、`rbnx start -p <package> -n <node>`（每次只启动一个 node，start 会阻塞直到进程退出）。

### 1.2 你需要先准备好

- 已能成功执行 `make build` 并启动 `./start_server`（robonix-server 提供 meta API 与 ping 服务）。
- 知道要实现的接口对应哪条 RIDL（例如 `robonix/system/debug/ping`、`robonix/prm/arm/joint_trajectory`）。若不清楚，见下文“从 RIDL 到生成代码的映射”或 [抽象硬件原语](primitives/index.md)。

### 1.3 谁可以用这套框架通信

不仅是 skill，**运行在系统之上的任意进程**（不属于标准服务、原语实现、skill 的“用户级”应用）都可以用同一套 RIDL 通信框架相互通信，只需约定好 RIDL 接口即可。类比 Android：系统服务、HAL、用户 app 均可通过 AIDL 通信；Robonix 中，robonix-server、prm 厂商、skill、以及任意自定义 package 都通过 RIDL 接口通信。

| 示例 | 原语 | 说明 |
|------|------|------|
| `stream_demo` | stream | stream_server 发布位姿，stream_client 订阅；两进程通过 `robonix/prm/base/pose_cov` 通信 |
| `query_demo` | query | semantic_server 提供语义查询，semantic_client 调用；通过 `robonix/system/map/semantic_query` 通信 |
| `skill_demo` | command | skill_server 提供 greet 命令，skill_client 调用；通过 package-local `skill_demo/skill/greet` 通信 |
| `python_ping_client` | query client | 调用 robonix-server 内置 ping；通过 `robonix/system/debug/ping` 通信 |

---

## 2. 从 RIDL 到生成代码的映射

### 2.1 RIDL 命名空间 -> Python 模块路径

| RIDL namespace       | Python 导入路径        |
|----------------------|------------------------|
| `robonix/system/debug` | `robonix.system.debug` |
| `robonix/prm/base`     | `robonix.prm.base`     |
| `robonix/prm/arm`      | `robonix.prm.arm`      |
| `robonix/system/map`   | `robonix.system.map`   |
| `robonix/system/skill` | `robonix.system.skill` |

规则：`robonix/a/b/c` -> `robonix.a.b.c`（斜杠变点号）。

### 2.2 接口类型 -> 文件名与函数名

| 原语     | RIDL 名示例   | 生成文件名        | 主要函数 |
|----------|----------------|-------------------|----------|
| query    | `ping`         | `ping_query.py`    | `create_ping_client`, `create_ping_server` |
| stream   | `pose`         | `pose_stream.py`   | `create_pose_publisher`, `create_pose_subscriber` |
| command  | `motion_cmd`   | `motion_cmd_command.py` | `create_motion_cmd_client`, `create_motion_cmd_server` |

### 2.3 快速查表：常见接口

| RIDL 接口 ID                    | Python 导入 | 服务端/发布方 | 客户端/订阅方 |
|----------------------------------|-------------|----------------|----------------|
| `robonix/system/debug/ping`      | `from robonix.system.debug.ping_query import create_ping_server` | `create_ping_server(runtime_client, node_id)` | `create_ping_client(runtime_client, requester_id, target)` |
| `robonix/system/map/semantic_query` | `from robonix.system.map.semantic_query_query import create_semantic_query_server` | `create_semantic_query_server(runtime_client, node_id)` | `create_semantic_query_client(...)` |
| `robonix/prm/base/move`         | `from robonix.prm.base.move_command import create_move_server` | `create_move_server(runtime_client, node_id)` | `create_move_client(...)` |
| `skill_demo/skill/greet` (package-local) | `from skill_demo.skill.greet_command import create_greet_server` | `create_greet_server(runtime_client, node_id)` | `create_greet_client(...)` |
| `robonix/prm/base/pose_cov`     | `from robonix.prm.base.pose_cov_stream import create_pose_cov_publisher` | `create_pose_cov_publisher(runtime_client, node_id)` | `create_pose_cov_subscriber(runtime_client, requester_id, target)` |

### 2.4 ROS 类型名

query 的 srv 类型：`robonix/system/debug` + `ping` -> `robonix_interfaces_ros2.srv.SystemDebugPing`。  
规则：`{NamespacePascal}{InterfacePascal}`。

### 2.5 ROS msg 包来源与 package.xml

**默认来源**：`rbnx build` 使用本仓库 `robonix-interfaces/lib` 下自维护的 IDL（`rcl_interfaces`、`common_interfaces` 等），不依赖系统 ROS msg 包。

- **来源**：`robonix-interfaces/lib/rcl_interfaces`、`robonix-interfaces/lib/common_interfaces`（如 `builtin_interfaces`、`std_msgs`、`geometry_msgs`、`action_msgs`、`nav_msgs`、`sensor_msgs`、`trajectory_msgs` 等）
- **构建流程**：ridlc 将上述包复制到 workspace 的 `vendor/`，colcon 在 vendor 中构建，产物进入 install 空间
- **使用方式**：与系统包完全一致，例如 `from std_msgs.msg import String`、`from geometry_msgs.msg import PoseWithCovarianceStamped`，字段定义以本仓库为准

**自定义 package.xml**：若需使用其他 ROS2 系统包（如 `sensor_msgs`、`geometry_msgs` 等 apt 安装的包），可在 package 根目录添加 `package.xml`，在其中声明 `<depend>`。rbnx build 会使用该文件，不会覆盖。示例见 `rust/examples/prm_camera_vendor/package.xml`、`rust/examples/skill_demo/package.xml`。

---

## 3. 创建 package 的完整步骤

### 步骤一：确定要实现的接口

- 若做 query 的 server：在 RIDL 里找到对应 query（如 `robonix/system/map/semantic_query`），记下 namespace 与接口名，得到 Python 模块 `robonix.system.map.semantic_query_query` 和 `create_semantic_query_server`。
- 若做 command 的 server：同理找到 command（如 `robonix/prm/arm/joint_trajectory`），得到 `create_joint_trajectory_server`。
- 若做 stream 的 provider（发布方）：在 RIDL 里找到对应 stream（如 `robonix/prm/base/pose_cov`），得到模块 `robonix.prm.base.pose_cov_stream` 和 `create_pose_cov_publisher(runtime_client, node_id)`；在循环中 `publish(msg)` 发布数据。
- 若做 stream 的 consumer（订阅方）：同上得到 `create_pose_cov_subscriber(...)`，需指定提供该 stream 的 target node id（与对方 manifest 里 `nodes[].id` 一致）；在回调中处理收到的消息。
- 若做 client（调用已有 query/command server）：用 `create_*_client(runtime_client, requester_id, target)`，其中 `target` 为提供该接口的 node id（与对方 manifest 里 `nodes[].id` 一致）。

### 步骤二：创建目录结构

在任意位置新建一个目录作为 package 根目录即可。推荐结构（Python package）：

```
my_package/                    # package 根目录（任意名，建议与 package.name 一致）
├── robonix_manifest.yaml      # 必选：manifest
└── my_package/                # Python 包名（与 manifest 中 package.name 一致）
    ├── __init__.py
    └── main.py                # 入口模块，manifest 里 entry 写 my_package.main:main
```

要点：

- `robonix_manifest.yaml` 必须在 package 根目录。
- `setup.py`、`setup.cfg`、`resource/` 由 `rbnx build` 根据 manifest 自动生成，无需手写。
- `package.xml`：若源码中已有，则直接使用（可添加自定义 ROS2 依赖如 `std_msgs`、`sensor_msgs` 等系统包）；若无则自动生成。
- entry 格式为 `模块:函数`，例如 `my_package.main:main` 表示执行 `from my_package.main import main` 并调用 `main()`。
- 若 package 名是 `python_ping_client`，则目录通常为 `python_ping_client/`，其下 Python 包也为 `python_ping_client/`，入口如 `python_ping_client.call_ping:main`。

### 步骤三：编写 robonix_manifest.yaml

在 package 根目录创建 `robonix_manifest.yaml`，内容示例：

```yaml
manifestVersion: 1

package:
  id: com.robonix.example.my_package      # 稳定 ID，可被其他 package 引用
  name: my_package                        # 包名，与目录名/colcon 包名一致
  version: 0.1.0
  vendor: robonix
  description: 一句话描述
  license: MulanPSL-2.0

nodes:
  - id: com.syswonder.example.my_node     # node id，通信时的目标标识，建议唯一
    type: python
    entry: my_package.main:main          # 该 node 的启动入口：模块:函数
```

字段说明：

| 字段 | 必填 | 说明 |
|------|------|------|
| `manifestVersion` | ✓ | 固定写 1 |
| `package.id` | ✓ | 稳定标识，如 `com.robonix.example.xxx` |
| `package.name` | ✓ | 包名，rbnx 的 -p 可写此名 |
| `package.version` | ✓ | 版本号 |
| `package.vendor` | ✓ | 厂商/组织 |
| `package.description` | ✓ | 简短描述 |
| `package.license` | ✓ | 许可证 |
| `nodes` | ✓ | node 列表，至少一项 |
| `nodes[].id` | ✓ | node id，通信时使用，建议 `com.syswonder.xxx` |
| `nodes[].type` | - | 默认 `python` |
| `nodes[].entry` | ✓ | 启动入口，如 `my_package.main:main` |

一个 package 可以有多个 node（多个 `nodes` 项），每次 `rbnx start -p <package> -n <node_id>` 只启动一个 node。

### 步骤四：编写业务代码

业务代码放在 package 根目录下的 Python 包内（与 `package.name` 同名的子目录），每个 node 的 entry 对应一个“模块:函数”。下面按 4.1 通用入口约定，再分 4.2–4.5 给出四种典型实现及完整目录与代码。

#### 4.1 入口函数约定与目录关系

- entry 格式：`包名.模块名:函数名`，例如 `python_ping_client.call_ping:main` 表示从 `python_ping_client` 包下的 `call_ping` 模块导入 `main` 并执行。
- 入口函数（如 `main()`）必须完成：
  1. 连接 robonix meta API（gRPC），得到 `runtime_client`（`RobonixRuntimeStub`）。
  2. 若为 server：用生成代码的 `create_*_server(runtime_client, node_id)` 创建服务，`node_id` 与 manifest 里该 component 的 `id` 一致；绑定 handler 或 execute 后调用 `server.start(...)`，最后 `rclpy.spin(server)` 常驻。
  3. 若为 client：用 `create_*_client(runtime_client, requester_id, target)` 创建客户端，构造请求并 `client.call(req, timeout_sec=...)`，打印或处理响应后退出。

目录结构关系示例（以包名 `my_package` 为例）：

```
my_package/                      # package 根目录
├── robonix_manifest.yaml
└── my_package/                  # Python 包，名与 package.name 一致
    ├── __init__.py
    ├── call_ping.py              # entry: my_package.call_ping:main
    ├── semantic_server.py        # entry: my_package.semantic_server:main
    ├── joint_trajectory_server.py
    └── skill_server.py
```

meta 地址默认 `127.0.0.1:50051`，可通过环境变量 `ROBONIX_META_GRPC_ENDPOINT` 覆盖。必须通过 `rbnx start` 启动，rbnx 会 source colcon install 的 setup，生成代码路径自动可用。

---

#### 4.2 Query Server 示例（语义地图 semantic_query）

实现 RIDL 接口 `robonix/system/map/semantic_query` 的 server：请求带 `filter`（String），响应填 `objects`（`robonix_msg/msg/Object[]`）。node id 与 manifest 中该 node 的 `id` 一致。

目录中新增文件（在 Python 包 `my_package` 下）：

- `my_package/semantic_server.py`

完整代码：

```python
# my_package/semantic_server.py
import grpc
import rclpy
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix_msgs.msg import Object
from robonix.system.map.semantic_query_query import create_semantic_query_server

def main():
    endpoint = "127.0.0.1:50051"   # 或从 ROBONIX_META_GRPC_ENDPOINT 读取
    node_id = "com.robonix.example.map_semantic"   # 与 manifest nodes[].id 一致

    rclpy.init()
    channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(channel)

    server = create_semantic_query_server(runtime_client, node_id)

    def handler(request, response):
        # 业务逻辑：根据 request.filter 查语义地图，填充 response.objects（Object[]）
        filter_str = request.filter.data if request.filter else ""
        obj1 = Object()
        obj1.id, obj1.label = "obj1", "table"
        obj2 = Object()
        obj2.id, obj2.label = "obj2", "cup"
        response.objects = [obj1, obj2]
        return response

    server.start(handler)
    rclpy.spin(server)

if __name__ == "__main__":
    main()
```

manifest 中对应 node 示例：

```yaml
nodes:
  - id: com.syswonder.map.semantic
    type: python
    entry: my_package.semantic_server:main
```

业务逻辑补全位置：`handler(request, response)` 内，根据 `request.filter` 查地图，将结果写入 `response.objects`（`robonix_msg/msg/Object[]`）。

---

#### 4.3 Query Client 示例（ping）

实现调用 `robonix/system/debug/ping` 的 client：连接 meta、解析 channel、构造 ping 请求、发送并打印响应。入口一次执行后退出。

目录中新增文件：

- `my_package/call_ping.py`（或本仓库示例中的 `python_ping_client/python_ping_client/call_ping.py`）

完整代码（与仓库内 `rust/examples/python_ping_client/python_ping_client/call_ping.py` 一致）：

```python
# my_package/call_ping.py
# Must be run via rbnx start (not direct python)
import sys
import grpc
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.system.debug.ping_query import create_ping_client
from robonix_interfaces_ros2.srv import SystemDebugPing
from std_msgs.msg import String

def main():
    endpoint = "127.0.0.1:50051"
    target = "robonix-server"   # 提供 ping 的 node id
    requester_id = "my_ping_client"
    payload = sys.argv[1] if len(sys.argv) > 1 else "hello"

    channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(channel)

    client = create_ping_client(runtime_client, requester_id, target)

    req = SystemDebugPing.Request()
    req.data = String()
    req.data.data = payload

    response = client.call(req, timeout_sec=10.0)
    print(response.data.data)

if __name__ == "__main__":
    main()
```

manifest 示例（client 作为单次调用的 node，可不被他人调用，id 仅标识本进程）：

```yaml
nodes:
  - id: my_ping_client
    type: python
    entry: my_package.call_ping:main
```

业务逻辑补全位置：在 `client.call(req)` 之后，对 `response` 做解析、打印或后续逻辑。

---

#### 4.4 Command Server 示例（机械臂 joint_trajectory）

实现 RIDL 接口 `robonix/prm/arm/joint_trajectory` 的 server：接收 `input.trajectory`（JointTrajectory），执行轨迹后返回 `result.status`（CommandResult）。生成代码提供 `create_joint_trajectory_server(runtime_client, node_id)`，返回对象的 `execute(request, goal_handle)` 由你实现，并赋给 `server.execute` 后调用 `server.start()`。

目录中新增文件：

- `my_package/joint_trajectory_server.py`

完整代码：

```python
# my_package/joint_trajectory_server.py
import grpc
import rclpy
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.prm.arm.joint_trajectory_command import create_joint_trajectory_server
from robonix_msgs.msg import CommandResult

def main():
    endpoint = "127.0.0.1:50051"
    node_id = "com.syswonder.prm.arm"   # 与 manifest nodes[].id 一致

    rclpy.init()
    channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(channel)

    server = create_joint_trajectory_server(runtime_client, node_id)

    def execute(request, goal_handle=None):
        # request 为 action 的 Goal 类型，含 trajectory 字段（JointTrajectory）
        trajectory = request.trajectory
        # 业务逻辑：将 trajectory 下发给真实机械臂驱动，等待执行完成
        result = server._action_type.Result()
        result.status = CommandResult()
        result.status.success = True
        result.status.message = "ok"
        return result

    server.execute = execute
    server.start()
    rclpy.spin(server)

if __name__ == "__main__":
    main()
```

manifest 示例：

```yaml
nodes:
  - id: com.syswonder.prm.arm
    type: python
    entry: my_package.joint_trajectory_server:main
```

业务逻辑补全位置：在 `execute(request, goal_handle)` 内解析 `request.trajectory`（JointTrajectory），驱动机械臂，构造并返回 action Result（含 `status` 字段为 CommandResult）。

---

#### 4.5 Per-Skill Command 示例（greet）

每个 skill 使用独立的 command RIDL，带类型化 input/result（ROS msg）。package 可在 `ridl/` 下定义 package-local RIDL，在 `msg/` 下放置 `.msg` 文件；rbnx build 会在 `rbnx-build/ws` 内自动生成 `{package}_msgs`、`{package}_interfaces`、`{package}_interfaces_ros2`，无需在源码中创建。

**源码目录结构**（以 skill_demo 为例，仅需手写以下内容）：

```
skill_demo/
├── robonix_manifest.yaml
├── msg/                       # 仅 .msg 文件，rbnx build 自动生成 skill_demo_msgs
│   ├── GreetRequest.msg       # string name
│   └── GreetResult.msg        # string message, bool success
├── ridl/skill/
│   └── greet.ridl             # namespace skill_demo/skill
└── skill_demo/
    ├── skill_server.py
    └── skill_client.py
```

`ridl/skill/greet.ridl` 示例：

```ridl
namespace skill_demo/skill

import skill_demo_msgs/msg/GreetRequest
import skill_demo_msgs/msg/GreetResult

command greet {
    input request skill_demo_msgs/msg/GreetRequest
    result response skill_demo_msgs/msg/GreetResult
    version 1.0;
}
```

**约定**：package-local RIDL 的 `namespace` 必须以 manifest 的 `package.name` 为前缀（如 `skill_demo/skill`）。

Server 代码：

```python
# skill_demo/skill_server.py
import grpc
import rclpy
from skill_demo.skill.greet_command import create_greet_server
from robonix_runtime_pb2_grpc import RobonixRuntimeStub

def main():
    endpoint = "127.0.0.1:50051"
    node_id = "skill_server"   # 与 manifest nodes[].id 一致
    channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(channel)
    server = create_greet_server(runtime_client, node_id=node_id)

    def execute(request, goal_handle=None):
        result = server._action_type.Result()
        name = request.request.name
        result.response.message = f"Hello, {name}!"
        result.response.success = True
        return result

    server.execute = execute
    server.start()
    rclpy.spin(server)

if __name__ == "__main__":
    main()
```

Client 代码（`skill_demo/skill_client.py`）：使用 `create_greet_client`，构造 typed Goal（`request.request = GreetRequest(); request.request.name = "world"`），处理 typed Result（`wrapped.result.response.message`、`wrapped.result.response.success`）。

manifest 示例：

```yaml
nodes:
  - id: skill_server
    type: python
    entry: skill_demo.skill_server:main
  - id: skill_client
    type: python
    entry: skill_demo.skill_client:main
```

业务逻辑补全位置：在 `execute(request, goal_handle)` 内使用 `request.request.name` 等类型化字段，返回 `result.response.message`、`result.response.success`。

---

小结：生成代码负责向 robonix-server 注册/解析 channel 和 ROS 绑定；你只在 query 的 handler、command 的 execute、或 client 的 call 之后写业务逻辑。node_id 必须与 manifest 中对应 component 的 `id` 一致。

### 步骤五：用 rbnx 构建与运行

`rbnx build` 会根据 manifest 自动生成 `setup.py`、`setup.cfg`、`resource/`。`package.xml`：若源码中已有则使用（可添加自定义 ROS2 依赖，如 `std_msgs`、`sensor_msgs`、`geometry_msgs` 等系统包）；若无则自动生成，依赖 `robonix-interfaces/lib` 下的 msg 包（见 §2.5）。在 `rust` 目录下（或 `-p` 指定为 package 路径）：

```bash
rbnx build -p my_package
rbnx start -p my_package   # 阻塞直到进程退出
```

- build：校验 manifest、执行 colcon 等，产物在 package 下的 `rbnx-build` 等目录。
- build：`rbnx build -p <path>` 或 `rbnx build -g <name>`。`-p` 为本地路径（如 examples/skill_demo）；`-g` 为系统已安装包名（`rbnx install` 安装的包）。默认增量构建；加 `--clean` 可清空 `rbnx-build` 后全量重建。
- start：`rbnx start -p <package> -n <node_id>` 每次只启动一个 node，按该 node 的 entry 启动进程并阻塞直到退出；需先启动 robonix-server，meta 地址可通过环境变量 `ROBONIX_META_GRPC_ENDPOINT` 或 rbnx `--endpoint` 传入。

`-p` 可为 package 根目录的绝对或相对路径；也可为 package 名字，此时 rbnx 在 examples/、cwd、rust/examples/ 下查找含 `robonix_manifest.yaml` 的目录，或系统安装的包（~/.robonix/packages）。

---

## 4. 三种典型 package 对照表

| 类型 | RIDL 接口 | 生成函数（server/发布方） | 业务补全位置 | 运行方式 |
|------|-----------|---------------------------|--------------|----------|
| 机械臂 PRM（command） | `robonix/prm/arm/joint_trajectory` | `create_joint_trajectory_server` | execute：收 trajectory，控机械臂，返回 CommandResult | 常驻，rbnx start |
| 语义地图（query） | `robonix/system/map/semantic_query` | `create_semantic_query_server` | start(handler)：request.filter -> 查地图 -> response.objects（Object[]） | 常驻，rbnx start |
| Skill 节点（command） | `skill_demo/skill/greet`（package-local） | `create_greet_server` | execute：typed request/result（GreetRequest/GreetResult） | 常驻，rbnx start |
| 位姿/传感器流（stream） | `robonix/prm/base/pose_cov` 等 | `create_pose_cov_publisher` | 循环中 `publish(msg)`；订阅方用 `create_pose_cov_subscriber`，在回调中处理 msg | 常驻（发布方）或按需（订阅方），rbnx start |

---

## 5. 使用生成的 Python 客户端（以 ping 为例）

```python
import grpc
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.system.debug.ping_query import create_ping_client
from robonix_interfaces_ros2.srv import SystemDebugPing
from std_msgs.msg import String

channel = grpc.insecure_channel("127.0.0.1:50051")
runtime_client = RobonixRuntimeStub(channel)
client = create_ping_client(runtime_client, "my_client", "robonix-server")

req = SystemDebugPing.Request()
req.data = String()
req.data.data = "hello"
response = client.call(req, timeout_sec=10.0)
print(response.data.data)
```

---

## 6. 使用生成的 Python 服务端

```python
server = create_ping_server(runtime_client, node_id="my_provider")

def handler(request, response):
    response.data = String()
    response.data.data = f"pong:{request.data.data}"
    return response

server.start(handler)
rclpy.spin(server)
```

---

## 7. 补全业务逻辑的位置小结

| 原语 | 补全位置 | 说明 |
|------|----------|------|
| query | `create_*_server` 返回的 `start(handler)` | handler(request, response) 内填 response |
| stream | publisher 循环里 `publish(msg)`；subscriber 回调里处理 msg | 按需发布/处理 |
| command | `create_*_server` 返回的 execute | 实现 action 的 execute，返回 result |

---

## 8. 参考

- 完整示例：本仓库 `rust/examples/` 下 `python_ping_client`（query client）、`query_demo`、`stream_demo`、`skill_demo`、`prm_camera_vendor`（相机厂商）、`prm_arm_vendor`（机械臂厂商）、`map_semantic_service`（地图服务），均含 `package.xml` 可参考；package 可放在任意路径。
- **抽象硬件原语**：相机、底盘、传感器、机械臂、夹爪、力/力矩等接口形态，见 [抽象硬件原语](primitives/index.md)。
- **硬件/服务厂商接入**：相机、机械臂、地图等厂商如何接入 RIDL 接口、按需实现接口子集，见 [硬件/服务厂商接入指南](vendor-integration.md)。
- Manifest 规范：[RFC002](../rfc/RFC002-Package-Management.md)。
- RIDL 与 channel：[RFC001](../rfc/RFC001-RIDL.md)。
