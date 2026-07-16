<span id="scene-robonixsystemscene"></span>
# 场景服务（Scene）

场景服务维护**世界状态**：把相机、深度图和视觉语言模型检测结果融合成对象注册表与场景图，回答“屋里有什么、在哪里、彼此是什么关系、去某个对象附近的导航目标是什么”。它是规划器向大语言模型提供环境事实的主要来源。

能力约定 TOML 在 `capabilities/system/scene/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/system/scene/list_objects` | `rpc` | [`semantic_map/ListObjects`](../../reference/idl.md#semantic-map-srv-listobjects-srv) | `system/scene/list_objects.v1.toml` |
| `robonix/system/scene/list_relations` | `rpc` | [`semantic_map/ListRelations`](../../reference/idl.md#semantic-map-srv-listrelations-srv) | `system/scene/list_relations.v1.toml` |
| `robonix/system/scene/get_scene_graph` | `rpc` | [`semantic_map/GetSceneGraph`](../../reference/idl.md#semantic-map-srv-getscenegraph-srv) | `system/scene/get_scene_graph.v1.toml` |
| `robonix/system/scene/get_object_context` | `rpc` | [`semantic_map/GetObjectContext`](../../reference/idl.md#semantic-map-srv-getobjectcontext-srv) | `system/scene/get_object_context.v1.toml` |
| `robonix/system/scene/get_robot_context` | `rpc` | [`semantic_map/GetRobotContext`](../../reference/idl.md#semantic-map-srv-getrobotcontext-srv) | `system/scene/get_robot_context.v1.toml` |
| `robonix/system/scene/goal_near` | `rpc` | [`semantic_map/GoalNear`](../../reference/idl.md#semantic-map-srv-goalnear-srv) | `system/scene/goal_near.v1.toml` |
| `robonix/system/scene/goal_room` | `rpc` | [`semantic_map/GoalRoom`](../../reference/idl.md#semantic-map-srv-goalroom-srv) | `system/scene/goal_room.v1.toml` |

上表是当前标准能力树的 7 条 `rpc` 约定；内置场景服务使用 MCP 传输。三份场景服务软件包清单都列出这 7 条能力，`scene_service/mcp_tools.py` 也为每条能力提供了对应处理函数；其中 `get_robot_context` 返回同一快照中的机器人位姿、所在房间或区域、附近对象与时效状态。

`list_objects` 返回可见物理对象、机器人和房间 annotation，可用其稳定 ID 调用 `goal_near` 或 `goal_room`。`goal_near` 只接受物理对象，房间或命名区域应交给 `goal_room`。两个目标接口都需要已经收到且非空的 `occupancy_grid` 才能给出 `reachable=true`；否则只返回失败原因，不能直接作为[导航服务](../service/navigation.md)目标。

输入侧：场景服务使用相机 RGB/depth、相机内参和地图位姿做感知与世界坐标投影，使用 `occupancy_grid` 计算导航安全目标，也会发现并缓存里程计与二维激光雷达输入。新架构以完整 URDF 发布的 TF 作为相机外参的权威来源，显式 `camera/extrinsics` 仅作兼容回退；当前实现还没有在所有路径上严格执行该优先级，详见 [Issue #156](https://github.com/syswonder/robonix/issues/156)。

当前激光雷达尚未进入检测、数据关联或场景图更新；三维雷达发现代码仍查询非标准旧 ID `robonix/primitive/lidar/pointcloud`，不能匹配标准 `robonix/primitive/lidar/lidar3d`。上表是只读查询面；`system/scene` 还通过 HTTP 提供标记的增删改查以及地图保存、加载、删除和位姿估计操作，这些 HTTP 路由不属于 `robonix/system/scene/*` 标准能力约定。参考实现：`system/scene`。
