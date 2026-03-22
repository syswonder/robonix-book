# 硬件/服务厂商接入指南

本文面向**硬件驱动**与**系统服务**提供方：说明如何在 Robonix 中以 package 形式接入 RIDL，使上层 skill 或其他节点能够通过统一的接口 id 调用你的能力。各硬件类型下具体有哪些 stream/query/command、推荐实现哪些组合，请以 [抽象硬件原语](primitives/index.md) 为索引查阅子页面。

---

## 1. 流程与共性

典型接入可以概括为四步：

1. 在 `robonix-interfaces/ridl/` 中阅读与你领域相关的 RIDL，结合硬件能力选定要实现的**接口子集**（不必实现该命名空间下的全部接口）。
2. 新建 package：编写 `robonix_manifest.yaml` 与 Python 入口模块，在入口中创建 gRPC 连接并获得 `RobonixRuntimeStub`。
3. 对每一个你决定支持的 interface，调用 ridlc 生成的 `create_*_server` 或 `create_*_publisher` 等**创建函数**，把本进程注册为对应能力的服务方。
4. 使用 `rbnx build` 构建，并用 `rbnx start` 启动（不要直接用 `python -m` 绕过 rbnx 注入的环境）。

在代码层面，大多数厂商节点都会重复同一骨架：`rclpy.init()` → `grpc.insecure_channel(endpoint)` → `RobonixRuntimeStub(channel)` → 调用各接口的 **`create_*` 创建函数**。**command** 侧通过 `server.execute = callback` 挂载业务；**query** 侧通过 `server.start(handler)` 传入处理函数；**stream** 侧则在定时器或采集循环中调用 `publish(msg)`（订阅方使用 `subscriber.start(callback)`）。更细的补全位置与示例见 [ridlc 手册 §5](ridlc.md#5-用户逻辑补全python)。

---

## 2. 按需实现

下表给出常见硬件/服务形态与**建议实现的接口子集**对照，帮助你快速决定“至少要提供哪些能力”。运行时只会对你实际调用 **`create_*` 创建函数**并注册成功的接口建立 channel；未实现的接口在解析阶段会失败，这属于预期行为而非配置错误。

| 场景 | 实现 | 可省略 |
|------|------|--------|
| 仅 RGB | rgb, intrinsics | depth, rgbd, ir |
| 纯深度 | depth（+ intrinsics） | rgb, rgbd, ir |
| RGB-D | rgb, depth, rgbd, intrinsics | ir 若不支持 |
| 臂无夹爪 | move_ee, move_joint, state_joint, joint_trajectory | gripper |
| 臂+夹爪 | 上表 + close/open 等 | 按硬件 |
| 语义地图 | `robonix/system/map/semantic_query` | 其他 system |

只注册实际实现的接口；未注册则解析失败。可选在 manifest 写 `provides`（§6）供文档/工具。

---

## 3. 相机示例（RGB-D，无 ir）

```
prm_camera_vendor/
├── robonix_manifest.yaml
├── package.xml              # 可选
└── prm_camera_vendor/
    ├── __init__.py
    └── camera_node.py       # entry: prm_camera_vendor.camera_node:main
```

```yaml
manifestVersion: 1

package:
  id: com.vendor.camera.rgbd
  name: prm_camera_vendor
  version: 0.1.0
  vendor: Example Camera Co.
  description: RGB-D (rgb, depth, rgbd, intrinsics)
  license: MulanPSL-2.0

nodes:
  - id: com.vendor.camera.rgbd
    type: python
    entry: prm_camera_vendor.camera_node:main
```

### 3.4 实现要点

**代码分工**：`create_*_publisher` 等由 ridlc 生成；你只需写 main 里的连接、定时/循环 publish。

**必备流程**：`rclpy.init()` → `grpc.insecure_channel(endpoint)` → `RobonixRuntimeStub(channel)` 得到 `runtime_client`。

**stream 无挂载**：直接 `publisher.publish(msg)`，在 timer 或循环里调用即可。

**多节点 spin**：每个 `create_*_publisher` 返回独立的 rclpy Node，需用 `MultiThreadedExecutor` 将全部 publisher 加入 executor 后统一 spin，否则只 spin 一个节点会导致其他 publisher 无法正常工作。

```python
# prm_camera_vendor/camera_node.py
import rclpy
from rclpy.executors import MultiThreadedExecutor
from robonix.prm.camera.rgb_stream import create_rgb_publisher
from robonix.prm.camera.depth_stream import create_depth_publisher
from robonix.prm.camera.rgbd_stream import create_rgbd_publisher
from robonix.prm.camera.intrinsics_stream import create_intrinsics_publisher

def main():
    rclpy.init()
    runtime_client = ...  # RobonixRuntimeStub
    node_id = "com.vendor.camera.rgbd"

    rgb_pub = create_rgb_publisher(runtime_client, node_id)
    depth_pub = create_depth_publisher(runtime_client, node_id)
    rgbd_pub = create_rgbd_publisher(runtime_client, node_id)
    intrinsics_pub = create_intrinsics_publisher(runtime_client, node_id)

    def publish_frame():
        rgb, depth = camera.capture()
        rgb_pub.publish(rgb)
        depth_pub.publish(depth)
        rgbd_pub.publish(RGBD(rgb=rgb, depth=depth))
        intrinsics_pub.publish(camera.get_intrinsics())

    timer = rgb_pub.create_timer(0.5, publish_frame)

    executor = MultiThreadedExecutor()
    for node in (rgb_pub, depth_pub, rgbd_pub, intrinsics_pub):
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        timer.cancel()
        executor.shutdown()
        for node in (rgb_pub, depth_pub, rgbd_pub, intrinsics_pub):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

---

## 4. 机械臂 + 夹爪（单 node）

```
prm_arm_vendor/
├── robonix_manifest.yaml
├── package.xml
└── prm_arm_vendor/
    ├── __init__.py
    ├── arm_node.py
    └── gripper_node.py      # 可选：另起 node
```

```yaml
manifestVersion: 1

package:
  id: com.vendor.arm.robot_arm
  name: prm_arm_vendor
  version: 0.1.0
  vendor: Example Arm Co.
  description: Arm + gripper (prm::arm, prm::gripper)
  license: MulanPSL-2.0

nodes:
  - id: com.vendor.arm.robot_arm
    type: python
    entry: prm_arm_vendor.arm_node:main
```

### 4.4 实现要点

**代码分工**：`create_*_server`、`RobonixRuntimeStub` 等由 ridlc 生成；你只需写 main 里的连接、回调实现、挂载与启动。

**必备流程**：`rclpy.init()` → `grpc.insecure_channel(endpoint)` → `RobonixRuntimeStub(channel)` 得到 `runtime_client`，用于向 meta 注册 channel。

**回调挂载**：command server 通过 `server.execute = your_callback` 挂载；`start()` 后收到 action 请求时会调用该回调。

**多节点 spin**：每个 `create_*_server` 返回独立的 rclpy Node，需用 `MultiThreadedExecutor` 将全部 server 加入 executor 后统一 spin。若只 `rclpy.spin(某一个)`，其他 server 的 action 请求将无法被处理。

```python
# prm_arm_vendor/arm_node.py
import rclpy
from rclpy.executors import MultiThreadedExecutor
from robonix.prm.arm.move_ee_command import create_move_ee_server
from robonix.prm.arm.joint_trajectory_command import create_joint_trajectory_server
from robonix.prm.gripper.close_command import create_close_server
from robonix.prm.gripper.open_command import create_open_server

def main():
    rclpy.init()
    runtime_client = ...
    node_id = "com.vendor.arm.robot_arm"

    move_ee_srv = create_move_ee_server(runtime_client, node_id)
    joint_traj_srv = create_joint_trajectory_server(runtime_client, node_id)
    close_srv = create_close_server(runtime_client, node_id)
    open_srv = create_open_server(runtime_client, node_id)

    move_ee_srv.execute = lambda req: arm_driver.move_to_pose(req.pose)
    joint_traj_srv.execute = lambda req: arm_driver.execute_trajectory(req.trajectory)
    close_srv.execute = lambda req: gripper_driver.close()
    open_srv.execute = lambda req: gripper_driver.open()

    for s in (move_ee_srv, joint_traj_srv, close_srv, open_srv):
        s.start()

    # 可选：state_joint stream 发布关节状态，若实现则需加入 executor
    # state_pub = create_state_joint_publisher(runtime_client, node_id)

    executor = MultiThreadedExecutor()
    nodes = (move_ee_srv, joint_traj_srv, close_srv, open_srv)
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

---

## 5. 地图服务（semantic_query）

```
map_semantic_service/
├── robonix_manifest.yaml
├── package.xml
└── map_semantic_service/
    ├── __init__.py
    └── semantic_server.py
```

```yaml
manifestVersion: 1

package:
  id: com.robonix.service.map_semantic
  name: map_semantic_service
  version: 0.1.0
  vendor: robonix
  description: semantic_query
  license: MulanPSL-2.0

nodes:
  - id: com.robonix.map.semantic
    type: python
    entry: map_semantic_service.semantic_server:main
```

```python
# map_semantic_service/semantic_server.py
from robonix.system.map.semantic_query_query import create_semantic_query_server

def main():
    runtime_client = ...
    node_id = "com.robonix.map.semantic"

    server = create_semantic_query_server(runtime_client, node_id)

    def handler(request, response):
        filter_str = request.filter.data
        response.objects = map_backend.query(filter_str)
        return response

    server.start(handler)
    rclpy.spin(server)
```

---

## 6. Manifest：`provides`（可选）

```yaml
provides:
  - robonix/prm/camera/rgb
  - robonix/prm/camera/depth
```

非强制；运行时以实际注册为准。建议厂商与实现一致填写。

---

## 7. 参考

`rust/examples/prm_camera_vendor/`、`prm_arm_vendor/`、`map_semantic_service/`；[Package 开发指南](package-development.md)、[ridlc](ridlc.md)、[RFC001](../rfc/RFC001-RIDL.md)、[RFC002](../rfc/RFC002-Package-Management.md)。
