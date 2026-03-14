# 硬件/服务厂商接入指南

本文档说明相机、机械臂等硬件厂商以及地图等系统服务如何接入 Robonix RIDL 接口。设计参考 Android HAL：厂商实现接口子集，按需注册，无需实现全部。

---

## 1. 接入流程概览

1. 查阅 RIDL 接口定义（robonix-interfaces/ridl/prm/*）
2. 选择本硬件/服务支持的接口子集
3. 创建 package，编写 manifest 与 entry
4. 实现所选接口的 server/publisher，向 meta API 注册
5. rbnx build + rbnx start 构建与运行

**代码分工**：ridlc 生成 `create_*_server`、`create_*_publisher`、`RobonixRuntimeStub` 等；你只写 main 里的连接、回调实现、挂载与启动。

**必备流程**：`rclpy.init()` → `grpc.insecure_channel(endpoint)` → `RobonixRuntimeStub(channel)` 得到 `runtime_client`，用于向 meta 注册 channel。所有 server/publisher 都依赖此 client。

**回调/接法**：command 用 `server.execute = callback` 挂载；query 用 `server.start(handler)` 传入；stream 直接 `publish(msg)`。

**`_setup_path`**：仅当直接 `python -m` 运行（未通过 rbnx start）时需要，用于把生成代码所在目录加入 PYTHONPATH；rbnx start 会 source colcon install，无需此段。

---

## 2. 核心原则：按需实现，无需全量

**与 Android HAL 类似**：RIDL 的 `prm/` 目录下定义了大量原语接口，厂商**只需实现本硬件实际支持的接口**，不必实现整个目录。

| 场景 | 需实现的接口 | 不必实现 |
|------|--------------|----------|
| 仅 RGB 相机 | prm::camera.rgb | depth, rgbd, ir, intrinsics |
| RGB-D 相机 | rgb, depth, rgbd, intrinsics | ir（若硬件不支持） |
| 机械臂（无夹爪） | move_ee, move_joint, state_joint, joint_trajectory | gripper 全部 |
| 机械臂 + 夹爪 | 上述 + gripper.close, gripper.open 等 | 按硬件能力 |
| 语义地图服务 | robonix/system/map/semantic_query | 其他 system 接口 |

**运行时行为**：node 启动时，仅对**实际实现的接口**调用 `create_*_server` / `create_*_publisher` 并注册。未实现的接口不会被注册，调用方解析时若目标 node 未提供该接口则失败。

**可选：manifest 声明**：为便于文档与工具检查，可在 manifest 中可选声明 `provides`（见 §6）。

---

## 3. 相机厂商接入示例

### 3.1 场景

某厂商提供 RGB-D 相机，支持 rgb、depth、rgbd、intrinsics，不支持 ir。

### 3.2 目录结构

```
prm_camera_vendor/
├── robonix_manifest.yaml
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── prm_camera_vendor
└── prm_camera_vendor/
    ├── __init__.py
    └── camera_node.py      # entry: prm_camera_vendor.camera_node:main
```

### 3.3 manifest

```yaml
manifestVersion: 1

package:
  id: com.vendor.camera.rgbd
  name: prm_camera_vendor
  version: 0.1.0
  vendor: Example Camera Co.
  description: RGB-D camera driver implementing prm::camera (rgb, depth, rgbd, intrinsics)
  license: MulanPSL-2.0

# 可选：声明本 package 提供的接口，便于文档与工具
# provides:
#   - robonix/prm/camera/rgb
#   - robonix/prm/camera/depth
#   - robonix/prm/camera/rgbd
#   - robonix/prm/camera/intrinsics

nodes:
  - id: com.vendor.camera.rgbd
    type: python
    entry: prm_camera_vendor.camera_node:main
```

### 3.4 实现要点

**代码分工**：`create_*_publisher` 等由 ridlc 生成；你只需写 main 里的连接、定时/循环 publish。

**必备流程**：`rclpy.init()` → `grpc.insecure_channel(endpoint)` → `RobonixRuntimeStub(channel)` 得到 `runtime_client`。

**stream 无挂载**：直接 `publisher.publish(msg)`，在 timer 或循环里调用即可。

```python
# prm_camera_vendor/camera_node.py
from robonix.prm.camera.rgb_stream import create_rgb_publisher
from robonix.prm.camera.depth_stream import create_depth_publisher
from robonix.prm.camera.rgbd_stream import create_rgbd_publisher
from robonix.prm.camera.intrinsics_stream import create_intrinsics_publisher

def main():
    runtime_client = ...  # gRPC stub
    node_id = "com.vendor.camera.rgbd"

    # 仅注册本硬件支持的 4 个 stream
    rgb_pub = create_rgb_publisher(runtime_client, node_id)
    depth_pub = create_depth_publisher(runtime_client, node_id)
    rgbd_pub = create_rgbd_publisher(runtime_client, node_id)
    intrinsics_pub = create_intrinsics_publisher(runtime_client, node_id)

    # 循环：从相机驱动取帧，按需 publish
    while True:
        rgb, depth = camera.capture()
        rgb_pub.publish(rgb)
        depth_pub.publish(depth)
        rgbd_pub.publish(RGBD(rgb=rgb, depth=depth))
        intrinsics_pub.publish(camera.get_intrinsics())
```

---

## 4. 机械臂厂商接入示例

### 4.1 场景

某厂商提供机械臂 + 夹爪，实现 move_ee、move_joint、state_joint、joint_trajectory、gripper.close、gripper.open。

### 4.2 目录结构

```
prm_arm_vendor/
├── robonix_manifest.yaml
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── prm_arm_vendor
└── prm_arm_vendor/
    ├── __init__.py
    ├── arm_node.py         # 机械臂 command/stream
    └── gripper_node.py     # 夹爪 command（可选：与 arm 同进程或分 node）
```

### 4.3 manifest（单 node 同时提供 arm + gripper）

```yaml
manifestVersion: 1

package:
  id: com.vendor.arm.robot_arm
  name: prm_arm_vendor
  version: 0.1.0
  vendor: Example Arm Co.
  description: Robot arm + gripper implementing prm::arm and prm::gripper
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

**`_setup_path`**：仅当直接 `python -m` 运行（未通过 rbnx start）时需要，用于把生成代码所在目录加入 PYTHONPATH；rbnx start 会 source colcon install，无需此段。

```python
# prm_arm_vendor/arm_node.py
from robonix.prm.arm.move_ee_command import create_move_ee_server
from robonix.prm.arm.joint_trajectory_command import create_joint_trajectory_server
from robonix.prm.arm.state_joint_stream import create_state_joint_publisher
from robonix.prm.gripper.close_command import create_close_server
from robonix.prm.gripper.open_command import create_open_server

def main():
    runtime_client = ...
    node_id = "com.vendor.arm.robot_arm"

    # 注册 command servers
    move_ee_srv = create_move_ee_server(runtime_client, node_id)
    joint_traj_srv = create_joint_trajectory_server(runtime_client, node_id)
    close_srv = create_close_server(runtime_client, node_id)
    open_srv = create_open_server(runtime_client, node_id)

    # 实现 execute 回调，调用真实硬件驱动
    move_ee_srv.execute = lambda req: arm_driver.move_to_pose(req.pose)
    joint_traj_srv.execute = lambda req: arm_driver.execute_trajectory(req.trajectory)
    close_srv.execute = lambda req: gripper_driver.close()
    open_srv.execute = lambda req: gripper_driver.open()

    move_ee_srv.start()
    joint_traj_srv.start()
    close_srv.start()
    open_srv.start()

    # 可选：state_joint stream 发布关节状态
    state_pub = create_state_joint_publisher(runtime_client, node_id)
    # 定时 publish(state_pub, arm_driver.get_joint_state())
    rclpy.spin(...)
```

---

## 5. 地图服务接入示例

### 5.1 场景

实现 `robonix/system/map/semantic_query` 的 query server，提供语义地图查询。

### 5.2 目录结构

```
map_semantic_service/
├── robonix_manifest.yaml
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── map_semantic_service
└── map_semantic_service/
    ├── __init__.py
    └── semantic_server.py
```

### 5.3 manifest

```yaml
manifestVersion: 1

package:
  id: com.robonix.service.map_semantic
  name: map_semantic_service
  version: 0.1.0
  vendor: robonix
  description: Semantic map query service implementing robonix/system/map/semantic_query
  license: MulanPSL-2.0

nodes:
  - id: com.robonix.map.semantic
    type: python
    entry: map_semantic_service.semantic_server:main
```

### 5.4 实现要点

**代码分工**：`create_semantic_query_server` 由 ridlc 生成；你只需写 main 里的连接、handler 实现、`start(handler)`。

**必备流程**：同上（rclpy.init、grpc channel、runtime_client）。

**handler 接法**：query server 通过 `server.start(handler)` 传入，收到请求时调用 `handler(request, response)`。

```python
# map_semantic_service/semantic_server.py
from robonix.system.map.semantic_query_query import create_semantic_query_server

def main():
    runtime_client = ...
    node_id = "com.robonix.map.semantic"

    server = create_semantic_query_server(runtime_client, node_id)

    def handler(request, response):
        filter_str = request.filter.data
        # 查地图、过滤，填充 response.objects (Object[])
        response.objects = map_backend.query(filter_str)
        return response

    server.start(handler)
    rclpy.spin(server)
```

---

## 6. Manifest 可选字段：provides

为便于文档与工具（如列出某 package 提供的接口），可在 manifest 中可选声明：

```yaml
provides:
  - robonix/prm/camera/rgb
  - robonix/prm/camera/depth
  - robonix/prm/camera/rgbd
  - robonix/prm/camera/intrinsics
```

- **非强制**：运行时以实际注册为准；未在 provides 中声明但已注册的接口仍可用。
- **建议**：厂商填写 provides，与实现保持一致，便于集成方查阅。

---

## 7. 与 Android HAL 的类比

| Android | Robonix |
|---------|---------|
| HIDL 接口定义 | RIDL 接口定义（ridl/prm/*, ridl/system/*） |
| 厂商实现 HAL | 厂商实现 package，提供 server/publisher |
| 按能力实现（LEGACY/LIMITED/FULL） | 按硬件能力实现接口子集 |
| 通过 hw_module_t 加载 | 通过 rbnx start 启动 node，向 meta API 注册 |
| 框架通过 Binder 调用 HAL | 调用方通过 Resolve* 解析 channel 后 ROS 调用 |

---

## 8. 参考

- **示例 package**：`rust/examples/prm_camera_vendor/`、`rust/examples/prm_arm_vendor/`、`rust/examples/map_semantic_service/`
- [Package 开发指南](package-development.md)
- [ridlc 开发手册](ridlc.md)
- [RFC001: RIDL](../rfc/RFC001-RIDL.md)
- [RFC002: Package](../rfc/RFC002-Package-Management.md)
