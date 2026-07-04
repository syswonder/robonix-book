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
| `robonix/system/scene/goal_near` | `rpc` | [`semantic_map/GoalNear`](../../reference/idl.md#semantic-map-srv-goalnear-srv) | `system/scene/goal_near.v1.toml` |

`list_objects` 不带过滤——返回注册表里所有对象，LLM 每轮调一次接地世界模型，按 label / 距离等在客户端筛（比把这些 knob 烤进 schema 便宜）。`goal_near` 把"到 X 附近"翻成一个可导航的 map 帧目标，喂给 [导航服务](../service/navigation.md)。

输入侧：scene 消费相机 RGB/depth、相机内外参、2D/3D lidar、map pose/odom/occupancy_grid 等标准能力约定，跑感知、数据关联和场景图更新。当前实现公开的是只读查询面；语义地图持久化、对象手工 add/edit/delete 等写接口应作为后续 scene map API 补齐。参考实现：`system/scene`。
