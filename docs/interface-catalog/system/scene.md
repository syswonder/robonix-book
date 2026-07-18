---
title: 场景服务
---
<span id="scene-robonixsystemscene"></span>
# 场景服务（Scene）

场景服务维护**世界状态**：把相机、深度图和视觉语言模型检测结果融合成对象注册表与场景图，回答“屋里有什么、在哪里、彼此是什么关系、去某个对象附近的导航目标是什么”。它是规划器向大语言模型提供环境事实的主要来源。

能力约定 TOML 在 `capabilities/system/scene/`。

## 接口

| 能力约定 ID | 模式 | 当前实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/system/scene/list_objects` | `rpc` | MCP | [`semantic_map/ListObjects`](../../reference/idl.md#semantic-map-srv-listobjects-srv) | `system/scene/list_objects.v1.toml` |
| `robonix/system/scene/list_relations` | `rpc` | MCP | [`semantic_map/ListRelations`](../../reference/idl.md#semantic-map-srv-listrelations-srv) | `system/scene/list_relations.v1.toml` |
| `robonix/system/scene/get_scene_graph` | `rpc` | MCP | [`semantic_map/GetSceneGraph`](../../reference/idl.md#semantic-map-srv-getscenegraph-srv) | `system/scene/get_scene_graph.v1.toml` |
| `robonix/system/scene/get_object_context` | `rpc` | MCP | [`semantic_map/GetObjectContext`](../../reference/idl.md#semantic-map-srv-getobjectcontext-srv) | `system/scene/get_object_context.v1.toml` |
| `robonix/system/scene/get_robot_context` | `rpc` | MCP | [`semantic_map/GetRobotContext`](../../reference/idl.md#semantic-map-srv-getrobotcontext-srv) | `system/scene/get_robot_context.v1.toml` |
| `robonix/system/scene/goal_near` | `rpc` | MCP | [`semantic_map/GoalNear`](../../reference/idl.md#semantic-map-srv-goalnear-srv) | `system/scene/goal_near.v1.toml` |
| `robonix/system/scene/goal_room` | `rpc` | MCP | [`semantic_map/GoalRoom`](../../reference/idl.md#semantic-map-srv-goalroom-srv) | `system/scene/goal_room.v1.toml` |

内置场景服务通过模型上下文协议（Model Context Protocol，MCP）提供上表 7 条 `rpc` 能力约定。`get_robot_context` 在同一快照中返回机器人位姿、所在房间或区域、附近对象与时效状态，适合在规划前一次获取当前环境上下文。

`list_objects` 返回可见物理对象、机器人和房间标记，可用其稳定 ID 调用 `goal_near` 或 `goal_room`。`goal_near` 只接受物理对象，房间或命名区域应交给 `goal_room`。两个目标接口都需要已经收到且非空的 `occupancy_grid` 才能给出 `reachable=true`；否则只返回失败原因，不能直接作为[导航服务](../service/navigation.md)目标。

输入侧：场景服务通过 Atlas 以完整能力约定 ID 发现 ROS 2 输入，包括相机彩色图、深度图、内参、外参、地图位姿、里程计、占据栅格、地图生命周期与二维/三维雷达。这些 `contract_id` 不是 ROS 2 话题名；Scene 在 `ConnectCapability` 返回实际端点后才订阅。米制 RGB-D 投影首先通过 TF2 查询当前定位世界帧到选中相机光学帧的完整变换。只在 TF2 不可用时，才组合 `robonix/service/map/pose`（或 `robonix/service/map/odom`）与 `robonix/primitive/camera/extrinsics`；再失败才使用仅供 bring-up 的机身位姿、偏航角和配置相机高度近似。新部署必须提供连通世界帧、机身帧与相机光学帧的完整 URDF/TF；当前源码不提供 `robonix/system/soma/sensor_extrinsics`。

激光雷达数据不直接创建视觉语义对象；它是否参与场景融合取决于当前服务配置和处理链。上表是只读查询接口；`system/scene` 还通过 HTTP 提供标记的增删改查，以及地图保存、加载、删除和位姿估计操作。这些 HTTP 路由不属于 `robonix/system/scene/*` 标准能力约定。参考实现位于 `system/scene`。
