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
| `robonix/system/scene/list_regions` | `rpc` | MCP | [`semantic_map/ListRegions`](../../reference/idl.md#semantic-map-srv-listregions-srv) | `system/scene/list_regions.v1.toml` |

内置场景服务通过模型上下文协议（Model Context Protocol，MCP）提供上表 8 条 `rpc` 能力约定。`get_robot_context` 在同一快照中返回机器人位姿、所在房间或区域、附近对象与时效状态，适合在规划前一次获取当前环境上下文。

`list_objects` 返回可见物理对象、机器人和兼容用途的房间条目，可用其稳定 ID 调用 `goal_near` 或 `goal_room`；新消费方应改用 `list_regions` 获取完整房间几何（多边形、朝向）与时效状态——过期（stale）标记仍然可见并被显式标出，不会被隐藏。`goal_near` 只接受物理对象，房间或命名区域应交给 `goal_room`。两个目标接口都需要已经收到且非空的 `occupancy_grid` 才能给出 `reachable=true`；否则只返回失败原因，不能直接作为[导航服务](../service/navigation.md)目标。

输入侧：场景服务通过 Atlas 以完整能力约定 ID 发现 ROS 2 输入，包括相机彩色图、深度图、内参、外参、地图位姿、里程计、占据栅格、地图生命周期与二维/三维雷达。这些 `contract_id` 不是 ROS 2 话题名；Scene 在 `ConnectCapability` 返回实际端点后才订阅。米制 RGB-D 投影首先通过 TF2 查询当前定位世界帧到选中相机光学帧的完整变换。只在 TF2 不可用时，才组合 `robonix/service/map/pose`（或 `robonix/service/map/odom`）与 `robonix/primitive/camera/extrinsics`；再失败才使用仅供 bring-up 的机身位姿、偏航角和配置相机高度近似。新部署必须提供连通世界帧、机身帧与相机光学帧的完整 URDF/TF；当前源码不提供 `robonix/system/soma/sensor_extrinsics`。

激光雷达数据不直接创建视觉语义对象；它是否参与场景融合取决于当前服务配置和处理链。上表是只读查询接口；`system/scene` 还通过 HTTP 提供标记的增删改查，以及地图保存、加载、删除和位姿估计操作。这些 HTTP 路由不属于 `robonix/system/scene/*` 标准能力约定。参考实现位于 `system/scene`。

## 地图绑定与对象持久化

Scene 启动时优先读取 Mapping 发布并保持的 `robonix/service/map/lifecycle`，以其中的 `map_id` 和 `generation` 绑定当前坐标系；没有该广播时，才依次回退到部署配置 `map_id`、`SCENE_MAP_ID` 和 `"default"`。每次启动默认建立新的实时对象会话，对象只有在地图界面执行 Save 时才与地图快照一起持久化，并在 Load 对应地图时恢复。`SCENE_RESTORE_ON_START` 只用于显式选择启动热恢复。

当前实现会监视运行期间的地图身份和 `generation` 变化并自动处理坐标系失效。同一地图的 `generation` 提升（reset 或重建图导致坐标原点改变）时，Scene 自动 flush 所有派生对象——重新观测会在新坐标系中重建它们——并把全部用户房间标记置为过期（stale）；标记是用户资产，只标记、不删除，等用户在地图界面确认仍有效或重新绘制。Mapping 在 Scene 之外切换到另一张地图时，Scene 同样 flush 失效对象，并暂停语义保存，直到在地图界面 Load 目标地图（或重启 Scene）完成重新绑定。

### 地图与房间标记的界面语义

空间地图只保存一次，其空间制品（artifact）不可变：对同一 `map_id` 再次 Save 会返回 409 并附带指引——要编辑该地图的房间和对象，应以定位模式 Load 它；要重建空间制品，只能删除该地图后重新保存。房间标记是用户资产：地图重建（`generation` 提升）不会删除它们，而是标为过期并在界面上以黄色显示，同时出现“map was rebuilt — review stale rooms”横幅；用户对每个过期标记点击“Still valid”确认仍有效（清除 stale），或重新绘制。

Scene Web 管理界面默认绑定 `0.0.0.0`；只允许本机操作时设置 `SCENE_WEB_HOST=127.0.0.1`。
