# Package 开发指南

<!-- toc -->

本指南说明：如何根据 RIDL 找到生成代码中的模块与 **`create_*` 创建函数**，如何在 `robonix_manifest.yaml` 中描述 package 与 node，如何在 Python 入口中连接 meta 并补全 query/command/stream 业务逻辑，以及如何用 `rbnx` 完成构建与启动。建议按章节顺序阅读；示例代码可直接对照仓库 `rust/examples/` 下的同名工程。

---

## 1. 概念与前置

### 1.1 Package / Node / rbnx

- **Package**：可被 `rbnx` 单独构建、启停的应用单元。根目录必须包含 `robonix_manifest.yaml`，其中声明 `package`（id、name、version 等）以及 `nodes` 列表；每个 node 通过 `entry`（形如 `包.模块:函数`）指定启动入口。
- **Node**：运行中的一个进程。`nodes[].id` 在通信模型里作为“目标节点”标识，建议采用 `com.syswonder.xxx` 这类可读且全局唯一的字符串。
- **rbnx**：构建使用 `rbnx build -p <path>` 或 `rbnx build -g <已安装包名>`；启动使用 `rbnx start -p <package> -n <node_id>`。每次 `start` 只拉起 manifest 中指定的一个 node，进程会阻塞到该 node 退出。

### 1.2 前置

- 本机已能完成 `make build` 并成功运行 `./start_server`，确保 meta gRPC 与内置 ping 可用。
- 你已清楚（或能在 RIDL 树中查到）自己要实现的接口，例如 `robonix/system/debug/ping`。若对接的是硬件能力，可先浏览 [抽象硬件原语](primitives/index.md) 再回到本文的映射表。

### 1.3 谁在用 RIDL 通信

Robonix 的通信面并不局限于 skill：任何运行在系统之上的**用户态进程**，只要与对方约定相同的 RIDL interface，就可以用同一套生成客户端/服务端代码完成互操作。下表给出本仓库示例与接口的对应关系，便于你对照阅读源码。

| 示例 | 通信语义 | 说明 |
|------|----------|------|
| `stream_demo` | stream | `stream_server` 发布位姿，`stream_client` 订阅；经 `robonix/prm/base/pose_cov`（底盘硬件抽象中的位姿流接口） |
| `query_demo` | query | `semantic_server` 提供语义查询，`semantic_client` 调用；经 `robonix/system/map/semantic_query` |
| `skill_demo` | command | `skill_server`/`skill_client`；经 package-local `skill_demo/skill/greet` |
| `python_ping_client` | query | 调用 robonix-server 内置 ping；经 `robonix/system/debug/ping` |

---

## 2. RIDL → 生成代码

### 2.1 命名空间 → Python 模块

| RIDL namespace | Python |
|----------------|--------|
| `robonix/system/debug` | `robonix.system.debug` |
| `robonix/prm/base` | `robonix.prm.base` |
| `robonix/prm/arm` | `robonix.prm.arm` |
| `robonix/system/map` | `robonix.system.map` |
| `robonix/system/skill` | `robonix.system.skill` |

规则：`robonix/a/b/c` → `robonix.a.b.c`。

### 2.2 通信语义 → 文件与创建函数

| 通信语义 | RIDL 名示例 | 文件 | 主要 `create_*` |
|------|-------------|------|----------|
| query | `ping` | `ping_query.py` | `create_ping_client`, `create_ping_server` |
| stream | `pose` | `pose_stream.py` | `create_pose_publisher`, `create_pose_subscriber` |
| command | `motion_cmd` | `motion_cmd_command.py` | `create_motion_cmd_client`, `create_motion_cmd_server` |

### 2.3 常见接口速查

| RIDL interface ID | 导入示例 | Server / Client |
|-------------------|----------|-----------------|
| `robonix/system/debug/ping` | `from robonix.system.debug.ping_query import create_ping_server` | `create_ping_server(runtime_client, node_id)` / `create_ping_client(runtime_client, requester_id, target)` |
| `robonix/system/map/semantic_query` | `...semantic_query_query import create_semantic_query_server` | server / client 同上模式 |
| `robonix/prm/base/move` | `...move_command import create_move_server` | command 模式 |
| `skill_demo/skill/greet`（本地） | `from skill_demo.skill.greet_command import create_greet_server` | command 模式 |
| `robonix/prm/base/pose_cov` | `...pose_cov_stream import create_pose_cov_publisher` | publisher / subscriber |

### 2.4 ROS 类型名

query：`robonix/system/debug` + `ping` → `robonix_interfaces_ros2.srv.SystemDebugPing`（`{NamespacePascal}{InterfacePascal}`）。

### 2.5 ROS msg 与 package.xml

- **默认**：`rbnx build` 用仓库内 `robonix-interfaces/lib` 的 IDL（`std_msgs`、`geometry_msgs` 等），经 ridlc 复制到 workspace `vendor/` 再 colcon；用法与系统包相同，如 `from std_msgs.msg import String`。
- **自定义 `package.xml`**：需额外系统 ROS2 包时在 package 根目录放置；rbnx 不会覆盖。参考 `rust/examples/prm_camera_vendor/package.xml`、`skill_demo/package.xml`。

---

## 3. 创建 package（步骤）

### 步骤一：定接口

- **query server**：记 namespace + 接口名 → 模块如 `robonix.system.map.semantic_query_query`，`create_semantic_query_server`。
- **command server**：如 `joint_trajectory` → `create_joint_trajectory_server`。
- **stream 发布**：如 `pose_cov` → `create_pose_cov_publisher`；循环里 `publish(msg)`。
- **stream 订阅**：`create_pose_cov_subscriber(..., target=<对方 nodes[].id>)`。
- **client**：`create_*_client(runtime_client, requester_id, target)`，`target` 为对端 `nodes[].id`。

### 步骤二：目录

```
my_package/
├── robonix_manifest.yaml
└── my_package/                 # 与 package.name 一致
    ├── __init__.py
    └── main.py                 # entry: my_package.main:main
```

- `setup.py` / `setup.cfg` / `resource/` 由 build 生成；有 `package.xml` 则用你的（可加 depend），无则自动生成。
- entry：`模块:函数`。

### 步骤三：robonix_manifest.yaml

```yaml
manifestVersion: 1

package:
  id: com.robonix.example.my_package
  name: my_package
  version: 0.1.0
  vendor: robonix
  description: 一句话
  license: MulanPSL-2.0

nodes:
  - id: com.syswonder.example.my_node
    type: python
    entry: my_package.main:main
```

| 字段 | 必填 | 说明 |
|------|------|------|
| `manifestVersion` | ✓ | 固定 `1` |
| `package.id` / `name` / `version` / `vendor` / `description` / `license` | ✓ | 身份与元数据 |
| `nodes` | ✓ | 至少一项 |
| `nodes[].id` | ✓ | 通信目标 id |
| `nodes[].type` | | 默认 `python` |
| `nodes[].entry` | ✓ | `模块:函数` |

多 node 时每次 `start` 只起一个：`-n <node_id>`。

### 步骤四：业务代码

补全位置：**query** → `server.start(handler)`；**command** → `server.execute = fn`；**stream** → `publish(msg)` / `subscriber.start(callback)`。细节见 [ridlc §5](ridlc.md#5-用户逻辑补全python)。

**入口约定**：`rclpy.init()` → `grpc.insecure_channel` → `RobonixRuntimeStub` → `create_*`；server 侧 `node_id` 须与 manifest 一致；`ROBONIX_META_GRPC_ENDPOINT` 可改 meta 地址。须通过 `rbnx start` 启动。

#### 4.1 目录关系示例

- entry 格式：`包名.模块名:函数名`，例如 `python_ping_client.call_ping:main` 表示从 `python_ping_client` 包下的 `call_ping` 模块导入 `main` 并执行。
- 入口函数（如 `main()`）必须完成：
  1. 连接 robonix meta API（gRPC），得到 `runtime_client`（`RobonixRuntimeStub`）。
  2. 若为 server：用生成代码的 `create_*_server(runtime_client, node_id)` 创建服务，`node_id` 与 manifest 里该 node 的 `id` 一致；绑定 handler 或 execute 后调用 `server.start(...)`，最后 `rclpy.spin(server)` 常驻。**多个 server/publisher 时**，每个 `create_*` 返回独立 Node，需用 `MultiThreadedExecutor` 将全部节点加入后统一 spin，详见 [厂商接入指南](vendor-integration.md)。
  3. 若为 client：用 `create_*_client(runtime_client, requester_id, target)` 创建客户端，构造请求并 `client.call(req, timeout_sec=...)`，打印或处理响应后退出。

目录结构（以包名 `my_package` 为例）：

```
my_package/
├── robonix_manifest.yaml
└── my_package/
    ├── call_ping.py
    ├── semantic_server.py
    ├── joint_trajectory_server.py
    └── skill_server.py
```

#### 4.2 Query server（semantic_query）

```python
# my_package/semantic_server.py
import grpc
import rclpy
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix_msgs.msg import Object
from robonix.system.map.semantic_query_query import create_semantic_query_server

def main():
    endpoint = "127.0.0.1:50051"
    node_id = "com.robonix.example.map_semantic"

    rclpy.init()
    channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(channel)

    server = create_semantic_query_server(runtime_client, node_id)

    def handler(request, response):
        filter_str = request.filter.data if request.filter else ""
        obj1, obj2 = Object(), Object()
        obj1.id, obj1.label = "obj1", "table"
        obj2.id, obj2.label = "obj2", "cup"
        response.objects = [obj1, obj2]
        return response

    server.start(handler)
    rclpy.spin(server)

if __name__ == "__main__":
    main()
```

```yaml
nodes:
  - id: com.syswonder.map.semantic
    type: python
    entry: my_package.semantic_server:main
```

#### 4.3 Query client（ping）

```python
# my_package/call_ping.py — 须通过 rbnx start，勿直接 python
import sys
import grpc
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.system.debug.ping_query import create_ping_client
from robonix_interfaces_ros2.srv import SystemDebugPing
from std_msgs.msg import String

def main():
    endpoint = "127.0.0.1:50051"
    target = "robonix-server"
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

```yaml
nodes:
  - id: my_ping_client
    type: python
    entry: my_package.call_ping:main
```

#### 4.4 Command server（joint_trajectory）

```python
# my_package/joint_trajectory_server.py
import grpc
import rclpy
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.prm.arm.joint_trajectory_command import create_joint_trajectory_server
from robonix_msgs.msg import CommandResult

def main():
    endpoint = "127.0.0.1:50051"
    node_id = "com.syswonder.prm.arm"

    rclpy.init()
    channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(channel)

    server = create_joint_trajectory_server(runtime_client, node_id)

    def execute(request, goal_handle=None):
        trajectory = request.trajectory
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

```yaml
nodes:
  - id: com.syswonder.prm.arm
    type: python
    entry: my_package.joint_trajectory_server:main
```

#### 4.5 Per-skill command（greet）

手写部分：

```
skill_demo/
├── robonix_manifest.yaml
├── msg/
│   ├── GreetRequest.msg
│   └── GreetResult.msg
├── ridl/skill/greet.ridl
└── skill_demo/
    ├── skill_server.py
    └── skill_client.py
```

`ridl/skill/greet.ridl`：

```ridl
namespace skill_demo/skill

import skill_demo_msgs/msg/GreetRequest
import skill_demo_msgs/msg/GreetResult

command greet @desc("A simple skill to greet a person.") {
    input request GreetRequest @desc("Name to greet");
    result response GreetResult @desc("Greeting message and success");
    version 1.0;
}
```

namespace 须以 `package.name` 为前缀。`rbnx build` 生成 `{package}_msgs` 等。

```python
# skill_demo/skill_server.py
import grpc
import rclpy
from skill_demo.skill.greet_command import create_greet_server
from robonix_runtime_pb2_grpc import RobonixRuntimeStub

def main():
    endpoint = "127.0.0.1:50051"
    node_id = "skill_server"
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

```python
# skill_demo/skill_client.py
from skill_demo.skill.greet_command import create_greet_client
from skill_demo_msgs.msg import GreetRequest

client = create_greet_client(runtime_client, requester_id=..., target=...)

request = client._action_type.Goal()
request.request = GreetRequest()
request.request.name = "world"

def on_feedback(fb_msg):
    print("feedback:", fb_msg.feedback.feedback.progress)

goal_future = client._client.send_goal_async(request, feedback_callback=on_feedback)
rclpy.spin_until_future_complete(client, goal_future, timeout_sec=10.0)
goal_handle = goal_future.result()

if goal_handle and goal_handle.accepted:
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(client, result_future, timeout_sec=10.0)
    wrapped = result_future.result()
    if wrapped:
        print(wrapped.result.response.message, wrapped.result.response.success)
```

```yaml
nodes:
  - id: skill_server
    type: python
    entry: skill_demo.skill_server:main
  - id: skill_client
    type: python
    entry: skill_demo.skill_client:main
```

**小结**：生成代码负责注册/解析 channel；你在 handler / execute / publish / call 后写业务；`node_id` 与 manifest 一致。

### 步骤五：构建与运行

```bash
rbnx build -p my_package          # 增量；--clean 全量
rbnx start -p my_package -n <node_id>
```

`-p`：路径或包名（查 `examples/`、`rust/examples/`、`~/.robonix/packages` 等）。须先起 robonix-server；meta 可用 `--endpoint` 或 `ROBONIX_META_GRPC_ENDPOINT`。

---

## 4. 典型 package 对照

| 类型 | RIDL | Server / 发布方 | 补全 |
|------|------|-----------------|------|
| 机械臂 PRM | `.../arm/joint_trajectory` | `create_joint_trajectory_server` | execute：trajectory → 硬件 → CommandResult |
| 语义地图 | `.../map/semantic_query` | `create_semantic_query_server` | handler：filter → objects |
| Skill | `skill_demo/skill/greet` | `create_greet_server` | execute：typed I/O |
| 位姿流 | `.../base/pose_cov` | `create_pose_cov_publisher` | 循环 `publish`；订阅 `create_pose_cov_subscriber` |

---

## 5. 补全位置一览

| 通信语义 | Server / Provider | Client / Consumer |
|----------|-------------------|-------------------|
| query | `server.start(handler)` | `client.call(request)` |
| stream | `publish(msg)`；订阅 `start(callback)` | `start(callback)` |
| command | `server.execute = fn`；可选 `publish_feedback` | Goal → `send` / `send_goal_async` → `get_result_async`；feedback：`fb_msg.feedback.feedback.<字段>` |

详见 [ridlc §5](ridlc.md#5-用户逻辑补全python)。

---

## 6. 参考

- 示例：`rust/examples/` 下 `python_ping_client`、`query_demo`、`stream_demo`、`skill_demo`、`prm_camera_vendor`、`prm_arm_vendor`、`map_semantic_service`。
- [抽象硬件原语](primitives/index.md)、[厂商接入](vendor-integration.md)、[RFC002](../rfc/RFC002-Package-Management.md)、[RFC001](../rfc/RFC001-RIDL.md)。
