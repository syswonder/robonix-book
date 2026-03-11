# Package 开发指南

<!-- toc -->

本文档详细说明：如何从零创建一个 Robonix package、如何根据 RIDL 找到生成代码与函数名、如何在 server/client 中补全业务逻辑、如何用 rbnx 构建与运行。按步骤操作即可完成一个可运行的 package。

---

## 1. 概念与前置条件

### 1.1 Package 是什么

- Package = 一个可被 rbnx 单独构建、启动、停止的应用单元。
- 根目录必须有 robonix_manifest.yaml，其中声明 package 身份（id、name、version 等）和 node 列表（每个 node 有一个启动 entry，如 `模块:函数`）。
- Node = 运行起来的一个进程；node id（在 manifest 的 `nodes[].id`）即通信时的目标标识，建议使用 `com.syswonder.xxx` 这类唯一 ID。
- 构建与启停只通过 rbnx：`rbnx build -p <package>`、`rbnx start -p <package> -n <node>`（每次只启动一个 node，start 会阻塞直到进程退出）。

### 1.2 你需要先准备好

- 已能成功执行 `make build` 并启动 `./start_server`（robonix-server 提供 meta API 与 ping 服务）。
- 知道要实现的接口对应哪条 RIDL（例如 `robonix/system/debug/ping`、`robonix/hal/arm/joint_trajectory`）。若不清楚，见下文“从 RIDL 到生成代码的映射”。

---

## 2. 从 RIDL 到生成代码的映射

### 2.1 RIDL 命名空间 -> Python 模块路径

| RIDL namespace       | Python 导入路径        |
|----------------------|------------------------|
| `robonix/system/debug` | `robonix.system.debug` |
| `robonix/hal/base`     | `robonix.hal.base`     |
| `robonix/system/map`   | `robonix.system.map`   |
| `robonix/system/skill` | `robonix.system.skill` |

规则：`robonix/a/b/c` -> `robonix.a.b.c`（斜杠变点号）。

### 2.2 接口类型 -> 文件名与函数名

| 原语     | RIDL 名示例   | 生成文件名        | 主要函数 |
|----------|----------------|-------------------|----------|
| query    | `ping`         | `ping_query.py`    | `create_ping_client`, `create_ping_server` |
| stream   | `pose`         | `pose_stream.py`   | `create_pose_publisher`, `create_pose_subscriber` |
| command  | `motion_cmd`   | `motion_cmd_command.py` | `create_motion_cmd_client`, `create_motion_cmd_server` |
| event    | `result_feedback` | `result_feedback_event.py` | 按 event 定义生成 |

### 2.3 快速查表：常见接口

| RIDL 接口 ID                    | Python 导入 | 服务端/发布方 | 客户端/订阅方 |
|----------------------------------|-------------|----------------|----------------|
| `robonix/system/debug/ping`      | `from robonix.system.debug.ping_query import create_ping_server` | `create_ping_server(runtime_client, node_id)` | `create_ping_client(runtime_client, requester_id, target)` |
| `robonix/system/map/semantic_query` | `from robonix.system.map.semantic_query_query import create_semantic_query_server` | `create_semantic_query_server(runtime_client, node_id)` | `create_semantic_query_client(...)` |
| `robonix/hal/base/motion_cmd`    | `from robonix.hal.base.motion_cmd_command import create_motion_cmd_server` | `create_motion_cmd_server(runtime_client, node_id)` | `create_motion_cmd_client(...)` |
| `robonix/system/skill/execute`   | `from robonix.system.skill.execute_command import create_execute_server` | `create_execute_server(runtime_client, node_id)` | `create_execute_client(...)` |
| `robonix/hal/localization/pose`  | `from robonix.hal.localization.pose_stream import create_pose_publisher` | `create_pose_publisher(runtime_client, node_id)` | `create_pose_subscriber(runtime_client, requester_id, target)` |

### 2.4 ROS 类型名

query 的 srv 类型：`robonix/system/debug` + `ping` -> `robonix_interfaces_ros2.srv.SystemDebugPing`。  
规则：`{NamespacePascal}{InterfacePascal}`。

---

## 3. 创建 package 的完整步骤

### 步骤一：确定要实现的接口

- 若做 query 的 server：在 RIDL 里找到对应 query（如 `robonix/system/map/semantic_query`），记下 namespace 与接口名，得到 Python 模块 `robonix.system.map.semantic_query_query` 和 `create_semantic_query_server`。
- 若做 command 的 server：同理找到 command（如 `robonix/hal/arm/joint_trajectory`），得到 `create_joint_trajectory_server`。
- 若做 stream 的 provider（发布方）：在 RIDL 里找到对应 stream（如 `robonix/hal/localization/pose`），得到模块 `robonix.hal.localization.pose_stream` 和 `create_pose_publisher(runtime_client, node_id)`；在循环中 `publish(msg)` 发布数据。
- 若做 stream 的 consumer（订阅方）：同上得到 `create_pose_subscriber(...)`，需指定提供该 stream 的 target node id（与对方 manifest 里 `nodes[].id` 一致）；在回调中处理收到的消息。
- 若做 client（调用已有 query/command server）：用 `create_*_client(runtime_client, requester_id, target)`，其中 `target` 为提供该接口的 node id（与对方 manifest 里 `nodes[].id` 一致）。

### 步骤二：创建目录结构

在任意位置新建一个目录作为 package 根目录即可。推荐结构（Python package，且需被 colcon 构建时）：

```
my_package/                    # package 根目录（任意名，建议与 package.name 一致）
├── robonix_manifest.yaml      # 必选：manifest
├── package.xml                 # 若用 rbnx build，需 colcon 包描述
├── setup.py                    # Python 包安装
├── setup.cfg
├── resource/
│   └── my_package             # 空文件或目录，供 ament 索引
└── my_package/                 # Python 包名（与 setup.py 里 package_name 一致）
    ├── __init__.py
    └── main.py                 # 入口模块，manifest 里 entry 写 my_package.main:main
```

要点：

- `robonix_manifest.yaml` 必须在 package 根目录。
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
├── package.xml
├── setup.py
├── setup.cfg
├── resource/my_package
└── my_package/                  # Python 包，名与 package.name 一致
    ├── __init__.py
    ├── call_ping.py              # entry: my_package.call_ping:main
    ├── semantic_server.py        # entry: my_package.semantic_server:main
    ├── joint_trajectory_server.py
    └── execute_server.py
```

meta 地址从环境变量 `ROBONIX_META_GRPC_ENDPOINT` 读取（默认 `127.0.0.1:50051`）。rbnx start 时会 source colcon install 的 setup，生成的 `robonix.system.*`、`robonix_runtime_pb2_grpc` 等会从 rbnx-build 的 install 里加入路径，一般无需在业务代码里改 sys.path；若本地直接 `python3 -m my_package.call_ping` 调试，需保证 PYTHONPATH 包含生成代码所在目录（见 4.3 中的路径设置说明）。

---

#### 4.2 Query Server 示例（语义地图 semantic_query）

实现 RIDL 接口 `robonix/system/map/semantic_query` 的 server：请求带 `filter`（String），响应填 `objects`（`robonix_msg/msg/Object[]`）。node id 与 manifest 中该 node 的 `id` 一致。

目录中新增文件（在 Python 包 `my_package` 下）：

- `my_package/semantic_server.py`

完整代码：

```python
# my_package/semantic_server.py
import os
import rclpy
import grpc
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix_msgs.msg import Object
from robonix.system.map.semantic_query_query import create_semantic_query_server

def main():
    endpoint = os.environ.get("ROBONIX_META_GRPC_ENDPOINT", "127.0.0.1:50051")
    node_id = "com.syswonder.map.semantic"   # 与 manifest nodes[].id 一致

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
import os
import sys
import grpc
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.system.debug.ping_query import create_ping_client
from robonix_interfaces_ros2.srv import SystemDebugPing
from std_msgs.msg import String

# 若直接运行脚本（非通过 rbnx start），需把生成代码所在路径加入 PYTHONPATH
def _setup_path():
    try:
        import robonix_runtime_pb2_grpc  # noqa: F401
    except ImportError:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.dirname(script_dir)   # my_package 包根
        # 若在 rust/examples/my_package 下，则 rust 目录在 os.path.dirname(pkg_root)
        rust_dir = os.path.dirname(pkg_root)
        python_pkg = os.path.join(
            rust_dir, "robonix-server", "target", "rclrs_interfaces_ws", "python_pkg"
        )
        if os.path.exists(python_pkg) and python_pkg not in sys.path:
            sys.path.insert(0, python_pkg)
        # 使用 rbnx build 时，生成代码在 package 的 rbnx-build/ws/install 下，需 source install/setup.bash，一般不需此处 fallback
_setup_path()

def main():
    endpoint = os.environ.get("ROBONIX_META_GRPC_ENDPOINT", "127.0.0.1:50051")
    target = os.environ.get("ROBONIX_PING_TARGET", "robonix-server")   # 提供 ping 的 node id
    requester_id = os.environ.get("ROBONIX_QUERY_REQUESTER_ID", "my_ping_client")
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

实现 RIDL 接口 `robonix/hal/arm/joint_trajectory` 的 server：接收 `input.trajectory`（JointTrajectory），执行轨迹后返回 `result.status`（CommandResult）。生成代码提供 `create_joint_trajectory_server(runtime_client, node_id)`，返回对象的 `execute(request)` 由你实现，并赋给 `server.execute` 后调用 `server.start()`。

目录中新增文件：

- `my_package/joint_trajectory_server.py`

完整代码：

```python
# my_package/joint_trajectory_server.py
import os
import rclpy
import grpc
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.hal.arm.joint_trajectory_command import create_joint_trajectory_server
from trajectory_msgs.msg import JointTrajectory
from robonix_msgs.msg import CommandResult

def main():
    endpoint = os.environ.get("ROBONIX_META_GRPC_ENDPOINT", "127.0.0.1:50051")
    node_id = "com.syswonder.hal.arm"   # 与 manifest nodes[].id 一致

    rclpy.init()
    channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(channel)

    server = create_joint_trajectory_server(runtime_client, node_id)

    def execute(request):
        # request 为 action 的 Goal 类型，含 trajectory 字段（JointTrajectory）
        trajectory = request.trajectory
        # 业务逻辑：将 trajectory 下发给真实机械臂驱动，等待执行完成
        # 示例：仅返回成功
        result = CommandResult()
        result.status = 0   # 或使用 robonix_msgs 中定义的常量
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
  - id: com.syswonder.hal.arm
    type: python
    entry: my_package.joint_trajectory_server:main
```

业务逻辑补全位置：在 `execute(request)` 内解析 `request`（JointTrajectory），驱动机械臂，构造并返回 `CommandResult`。

---

#### 4.5 Skill Execute Server 示例（execute command）

实现 RIDL 接口 `robonix/system/skill/execute` 的 server：`input.request_json`、`result.response_json` 为 String；可选通过 `output.progress_json` 上报进度。生成代码提供 `create_execute_server(runtime_client, node_id)`，你实现 `execute(request, goal_handle)` 并赋给 `server.execute` 后 `server.start()`。需上报进度时调用 `goal_handle.publish_feedback(feedback_msg)`。

目录中新增文件：

- `my_package/execute_server.py`

完整代码：

```python
# my_package/execute_server.py
import os
import json
import rclpy
import grpc
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.system.skill.execute_command import create_execute_server
from std_msgs.msg import String

def main():
    endpoint = os.environ.get("ROBONIX_META_GRPC_ENDPOINT", "127.0.0.1:50051")
    node_id = "com.syswonder.skill.executor"

    rclpy.init()
    channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(channel)

    server = create_execute_server(runtime_client, node_id)

    def execute(request, goal_handle=None):
        # request 为 action 的 Goal 类型，含 request_json（String）
        req_str = request.request_json.data if request.request_json else "{}"
        try:
            params = json.loads(req_str)
        except json.JSONDecodeError:
            result = server._action_type.Result()
            result.response_json = String()
            result.response_json.data = json.dumps({"ok": False, "error": "invalid json"})
            return result

        # 业务逻辑：根据 params 执行技能
        # 可选上报 progress：goal_handle.publish_feedback(feedback_msg)
        if goal_handle:
            feedback = server._action_type.Feedback()
            feedback.progress_json = String()
            feedback.progress_json.data = json.dumps({"percent": 50, "status": "running"})
            goal_handle.publish_feedback(feedback)

        result = server._action_type.Result()
        result.response_json = String()
        result.response_json.data = json.dumps({"ok": True, "result": "done"})
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
  - id: com.syswonder.skill.executor
    type: python
    entry: my_package.execute_server:main
```

业务逻辑补全位置：在 `execute(request, goal_handle)` 内解析 `request.request_json`，执行动作。需上报进度时调用 `goal_handle.publish_feedback(feedback_msg)`，其中 `feedback_msg` 为 `server._action_type.Feedback()`，设置 `progress_json` 字段后传入。最后返回 `response_json`（String）。

---

小结：生成代码负责向 robonix-server 注册/解析 channel 和 ROS 绑定；你只在 query 的 handler、command 的 execute、或 client 的 call 之后写业务逻辑。node_id 必须与 manifest 中对应 component 的 `id` 一致。

### 步骤五：Colcon 配置（供 rbnx build 使用）

rbnx build 会走 colcon，因此 package 根目录需具备 colcon 可识别的 Python 包结构。

- package.xml：`<name>` 与目录名/manifest 中 `package.name` 一致；`<buildtool_depend>ament_python</buildtool_depend>`，`<depend>` 中列出 `rclpy`、`robonix_interfaces`、`robonix_interfaces_ros2` 以及用到的 ROS 消息包（如 `std_msgs`、`robonix_msgs`、`trajectory_msgs`）。
- setup.py：`package_name`、`find_packages`、`data_files`（ament 资源）、`entry_points` 中可声明 `console_scripts`（与 manifest entry 对应，便于调试）。
- resource/my_package：空文件，用于 ament 包索引。
- setup.cfg：`[develop]` / `[install]` 中 `script_dir`、`install_scripts` 指向 `$base/lib/<package_name>`。

可对照本仓库中的 `rust/examples/python_ping_client/` 的 `package.xml`、`setup.py`、`setup.cfg`、`resource/` 抄写并改名。

### 步骤六：用 rbnx 构建与运行

在 `rust` 目录下（或 `-p` 指定为 package 路径）：

```bash
rbnx build -p my_package
rbnx start -p my_package   # 阻塞直到进程退出
```

- build：校验 manifest、执行 colcon 等，产物在 package 下的 `rbnx-build` 等目录。
- start：`rbnx start -p <package> -n <node_id>` 每次只启动一个 node，按该 node 的 entry 启动进程并阻塞直到退出；需先启动 robonix-server，meta 地址可通过环境变量 `ROBONIX_META_GRPC_ENDPOINT` 或 rbnx `--endpoint` 传入。

`-p` 可为 package 根目录的绝对或相对路径；也可为 package 名字，此时 rbnx 在 examples/、cwd、rust/examples/ 下查找含 `robonix_manifest.yaml` 的目录（以实际实现为准）。

---

## 4. 三种典型 package 对照表

| 类型 | RIDL 接口 | 生成函数（server/发布方） | 业务补全位置 | 运行方式 |
|------|-----------|---------------------------|--------------|----------|
| 机械臂 HAL（command） | `robonix/hal/arm/joint_trajectory` | `create_joint_trajectory_server` | execute：收 trajectory，控机械臂，返回 CommandResult | 常驻，rbnx start |
| 语义地图（query） | `robonix/system/map/semantic_query` | `create_semantic_query_server` | start(handler)：request.filter -> 查地图 -> response.objects（Object[]） | 常驻，rbnx start |
| Skill 节点（command） | `robonix/system/skill/execute` | `create_execute_server` | execute：request_json -> 执行动作 -> response_json | 常驻，rbnx start |
| 位姿/传感器流（stream） | `robonix/hal/localization/pose` 等 | `create_pose_publisher` | 循环中 `publish(msg)`；订阅方用 `create_pose_subscriber`，在回调中处理 msg | 常驻（发布方）或按需（订阅方），rbnx start |

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

- 完整示例：本仓库 `rust/examples/python_ping_client/`（query client、manifest、package.xml、setup.py、entry），仅作参考；package 可放在任意路径。
- Manifest 规范：[RFC002](../rfc/RFC002-Package-Management.md)。
- RIDL 与 channel：[RFC001](../rfc/RFC001-RIDL.md)。
