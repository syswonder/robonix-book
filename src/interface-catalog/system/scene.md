# Scene robonix/system/scene

Scene 维护**世界状态**：把感知（相机 + 深度 + VLM 检测）融合成一个对象注册表 + 场景图，回答"屋里有什么、在哪、彼此什么关系、去某个对象附近的导航目标是什么"。是 Pilot 给 LLM 接地世界模型的主要来源。

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

上表是当前标准能力树的 7 条约定。`system/scene` 当前启动代码声明其中 6 条 MCP 能力；`get_robot_context` 已有标准约定，但尚未加入运行时声明列表。`list_objects` 返回注册表里的对象，`goal_near` / `goal_room` 把对象或房间解析成可交给[导航服务](../service/navigation.md)的 map 帧目标。

输入侧：scene 消费相机 RGB/depth、相机内外参、2D/3D lidar、map pose/odom/occupancy_grid 等标准能力约定，跑感知、数据关联和场景图更新。上表的标准能力约定是只读查询面；`system/scene` 还通过 HTTP 提供 annotation CRUD 以及地图保存、加载、删除和位姿估计操作，这些 HTTP 路由不属于 `robonix/system/scene/*` 标准能力约定。参考实现：`system/scene`。
