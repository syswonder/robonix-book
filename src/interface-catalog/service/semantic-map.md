# 语义地图服务（设计稿 / TODO）

Status：设计中，未实现。

## 核心目标

语义地图是 Robonix 里纯粹的"语义信息数据结构 + 查询服务"，不是仿真器的控制接口。它的职责很窄：在内存里维护一份场景的语义视图（物体、类别、属性、状态、物体间关系），提供统一的查询 RPC 供大脑使用，能落盘加载跨进程交换。

三件事是：

一，一套统一的数据结构，把空间几何 + 物体语义 + 物体间拓扑关系装进一份 `SemanticScene`。大脑推理只查这一份（"cup 在哪个 table 上"、"forklift 离我多近"），不关心这些信息是来自哪条感知链路。

二，一套统一的 RPC 接口（`srv/semantic_map/*`）。所有对语义地图的读写都走这个接口。上层只认这组 RPC，跨进程、跨机器也都一致。

三，save / load / 跨进程交换都可用，落盘格式固定（MJCF + `<custom>` 语义段），任何 Robonix 节点之间能传一份完整场景。

语义地图和仿真器是两回事：
- 仿真器是语义地图的一个**数据来源**：把 MuJoCo/Isaac Sim/Webots 的场景导入成 `SemanticScene`（初始化或定期同步）。
- 仿真器也是语义地图的一个**消费者**：给定一份 `SemanticScene`，仿真器能做场景重建（load 成可跑的 scene）或预演（rollout 一段假设的操作看会发生什么）。
- 真机同样两边都可以：SLAM + VLM 产 `SemanticScene`（数据源），或者拿一份已有的 `SemanticScene` 做导航规划（消费者）。

仿真器/真机对**物理世界**本身的操作（让机械臂动、让门打开）走的是 `srv/navigation/*`、`srv/manipulation/*` 这些执行类服务，不是语义地图的职责。语义地图里的"写"只是对这份数据结构本身的写（新增标注、更新观察到的 pose 估计、记录一条新的 `on` 关系）。

## Robonix 内存里的数据结构 ≠ 序列化文件格式

这里先把最关键的区分讲清楚，后面的所有设计都围绕这个点展开：

Robonix 运行时在内存里维护的是**自己定义的数据结构**（Rust struct / Python class / gRPC message），由 `rust/contracts` 下的 IDL 生成，语言无关，可以扩展任意字段（空间关系、affordance、时序状态等）。这个结构的形状由我们自己决定，和 MuJoCo 无关。

只有当数据要写入磁盘、或者通过一条 ROS2/gRPC 通路送到另一台机器 / 另一个进程时，我们才把内存结构序列化成一个约定的 wire format。wire format 选择了 **MuJoCo MJCF XML** 作为场景几何部分的底层表示，并在其上挂我们自己的语义扩展（`<custom>` 节 + JSON payload）。

也就是说：

- 内存态：`robonix::SemanticScene { bodies: Vec<Body>, labels: HashMap<...>, relations: Vec<Relation>, ... }`
- 序列化态：一份 MJCF XML + 附带的 JSON 语义 payload + 可选的 occupancy PNG

同一个语义信息在内存里可能用 `HashMap<String, SemanticLabel>` 存，落盘时把它挤进 MJCF 的 `<custom><text name="robonix.semantic.labels" data='{...}'/>`。加载回来时再反序列化成内存结构。两侧形状不对等，而且以内存侧为准。

## 为什么选 MJCF 作为序列化格式

场景几何 + 机器人描述需要一个共识格式。候选：

| 格式 | 归属 | 问题 |
|---|---|---|
| USD | Pixar / NVIDIA | 二进制，需要 Pixar SDK，和 Isaac Sim 深度绑定，LLM 不友好 |
| WBT | Webots / Cyberbotics | 纯文本但生态封闭，转出工具少 |
| SDF | Gazebo | 文本 OK，但 Gazebo 生态在萎缩，新仿真器少有支持 |
| MJCF | MuJoCo / Google DeepMind | 纯 XML、与 URDF 互转成熟、MuJoCo/Brax/dm_control 官方支持、可被 Isaac Sim / Webots 通过 adapter 读入 |

选 MJCF 的决定因素：
- 不和 NVIDIA 绑定，避免把 Robonix 的仿真能力锁死在 Isaac Sim
- URDF→MJCF 在 `mujoco` CLI 里一行命令，绝大多数机器人的 URDF 可以零代价接入
- XML 格式可以 diff、可以 code review、可以被 LLM 生成和修改
- `<custom>` 节提供了语义扩展挂载点，不需要 fork MJCF 规范
- 社区适配器矩阵（MJCF → USD、MJCF → WBT、MJCF → SDF）已经存在或容易写

MJCF 本身不完备，尤其在语义侧（它只描述物理几何）。所以我们在它上面加一层 Robonix-specific 语义 schema，用 `<custom>` 节承载。序列化时一并写入，加载时一并解析。

## 抽象接口（contract + IDL 草案）

所有接口走 Robonix 标准的 `srv/semantic_map/*` 命名空间，IDL 定义在 `rust/contracts/srv/semantic_map_*.v1.toml`。每条服务对应一个内存操作，返回类型都是 Robonix 自己的 message，不是 MJCF 片段。分三组：

读：

| Contract | 行为 |
|---|---|
| `srv/semantic_map/get_scene` | 拉取当前内存里的整个场景或一个局部区域 |
| `srv/semantic_map/query_by_name` | 按 object 名查单个物体，返回 pose + bbox + labels + relations |
| `srv/semantic_map/query_by_class` | 按语义类别查一组物体，可叠空间过滤 |
| `srv/semantic_map/query_relations` | 查两个 object 之间的空间关系（on / in / near / left_of ...） |
| `srv/semantic_map/raycast` | 从 origin 沿方向投射，返回首个命中 object 及距离 |

写（只改语义地图自己这份数据结构，不驱动任何仿真器或物理动作）：

| Contract | 行为 |
|---|---|
| `srv/semantic_map/add_object` | 在语义地图里新增一个 Object，返回 `ObjectId` |
| `srv/semantic_map/remove_object` | 从语义地图里删除一个 Object 及其所有 Relation |
| `srv/semantic_map/update_pose` | 更新一个 Object 在语义地图里记录的位姿（比如感知新观察到的值） |
| `srv/semantic_map/update_state` | 更新一个 Object 的 `state` 字段（VLM 观察到 `drawer.open=true` 等） |
| `srv/semantic_map/upsert_relation` | 新增/更新一条 Relation |
| `srv/semantic_map/remove_relation` | 删除一条 Relation |
| `srv/semantic_map/label` | 批量 upsert 语义标签（感知链路回灌的快捷入口） |

持久化：

| Contract | 行为 |
|---|---|
| `srv/semantic_map/save` | 把当前内存结构序列化到磁盘（MJCF + `<custom>` 语义段 + 可选 occ PNG） |
| `srv/semantic_map/load` | 从磁盘或 URL 反序列化一份场景到内存 |

`get_scene` 返回 Robonix 的 `SemanticScene` message（IDL 生成），不是 MJCF 字符串。需要原始 MJCF 的场合走 `save`，或者后续加 `export_mjcf`。

这里所有"写"都停留在语义地图自己这层。让物理世界真的发生改变（把门打开、让机械臂去抓杯子）是另外几个服务的事：`srv/navigation/*` 负责底盘，`srv/manipulation/*` 负责末端，`prm/gripper/*` 之类负责更底层的执行。这些执行服务完成后，感知链路会再把新的观察写回 `srv/semantic_map/*` 保持一致。

## Robonix 内存侧的数据结构

顶层类型 `SemanticScene` 包含四部分：`frame_info`（坐标系元信息）、`objects`（场景里的语义物体）、`relations`（物体间关系）、`version`（版本号用于增量同步）。可选再带一个 `occupancy` 缓存（2D/3D 占据栅格，由 SLAM 层派生）。

`Object` 字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `id` | `ObjectId` | 唯一稳定标识，重新加载后不变 |
| `name` | string | 人类可读名（"red cup"），允许重复 |
| `class` | string | 语义类别（`cup` / `table` / `aisle`） |
| `pose` | 7-DoF (xyz + quat) | 世界系位姿 |
| `bbox` | 包围盒（轴对齐 XYZ min/max，或带旋转的 OBB） | 用于 raycast、邻近查询 |
| `mesh_ref` | 可选 URI | 指向外部几何 asset |
| `attributes` | Map<string, Value> | VLM/上层可任意扩展 |
| `affordances` | List<string> | `graspable` / `container` / `pushable` ... |
| `state` | Map<string, Value> | 运行时状态（`open`、`held_by=robot0`） |

`Relation` 字段（`a` 和 `b` 用来代替语法上的 subject/object，避免和 `Object` 类型重名）：

| 字段 | 类型 | 说明 |
|---|---|---|
| `kind` | string | `on` / `in` / `near` / `left_of` / 用户扩展 |
| `a` | `ObjectId` | 关系的第一方 |
| `b` | `ObjectId` | 关系的第二方 |
| `metadata` | Map<string, Value> | 置信度、观察时间等 |

查询 "cup 在哪个 table 上" = 一次 `query_relations(kind="on", a=<cup_id>)`，返回 `b` 列表。这是大脑推理最常走的入口。

## MJCF 序列化约定

落盘或跨进程传输时按下面的约定写 MJCF。每个 Robonix `Object` 对应一个 MJCF `<body>`（MJCF 原生物体节点），`name` 属性就是 `ObjectId`。Robonix 专属的语义字段（class / affordances / state / relations）挂在 `<custom>` 下，key 带 `robonix.` 前缀以免和上层 MJCF 规范冲突：

```xml
<mujoco model="robonix_scene_001">
  <worldbody>
    <body name="table_01" pos="0 0 0">
      <geom type="box" size="0.6 0.4 0.02"/>
    </body>
    <body name="cup_red" pos="0.2 0.1 0.05">
      <geom type="mesh" mesh="cup"/>
    </body>
  </worldbody>

  <custom>
    <numeric name="robonix.schema_version" data="1"/>
    <text name="robonix.objects" data='{
      "table_01": {"class":"table", "affordances":["support"]},
      "cup_red":  {"class":"cup",   "affordances":["graspable"], "state":{"held":false}}
    }'/>
    <text name="robonix.relations" data='[
      {"kind":"on", "a":"cup_red", "b":"table_01"}
    ]'/>
  </custom>
</mujoco>
```

`robonix.objects` 是一个 JSON map，key 是 MJCF body name，value 放 Robonix 独有的字段。加载时先走标准 MJCF 解析拿到每个 body 的几何骨架（pose、bbox），再从 `<custom>` 读 JSON 补上语义字段得到完整的 `Object`。

MJCF → USD / MJCF → WBT 的适配器只处理 `<worldbody>` 部分，`<custom>` 里的 `robonix.*` 对它们透明。

## Fast Lookup Table

在线查询（`query_by_name`、`query_by_class`、`query_relations`）不能每次重算，需要从 `SemanticScene` 派生一层索引：

- `by_name`:  `ObjectId → object_ref`
- `by_class`: `class → [ObjectId]`
- `spatial_kd`: object 质心的 KD-tree，用于 `k-nearest`、`within_radius` 查询
- `rel_index`: `(subject, kind) → [object]` 反向表

索引随 scene 增量更新；每次 `label` / `load` 之后重建相关 bucket。

## 对接不同仿真环境

Agent/Pilot 只认 `SemanticScene` 这一层接口。每个仿真环境或真机数据源通过一个"挂载驱动"把自己原生的世界描述桥接进来，类似 Linux VFS 下挂不同文件系统的思路：上层文件操作语义统一，具体读写由对应驱动处理。Robonix 这里的"挂载驱动"做两件事：读原生格式转成 `SemanticScene`；接到 `save` 时反向把 `SemanticScene` 写回原生格式（如果需要）。

| 环境 | 原生世界描述 | 挂载驱动做的事 |
|---|---|---|
| MuJoCo / Brax / dm_control | MJCF | 直接读写，无转换 |
| Isaac Sim | USD | USD 解析成 `Object` + 几何，或先 `usd → mjcf` 再走 MJCF 路径 |
| Webots | WBT / Proto | `wbt → urdf → mjcf`，然后走 MJCF 路径 |
| Gazebo | SDF | `sdf → urdf → mjcf` 或直接 `sdf → Object` |
| 真机 | 无统一格式 | SLAM 产几何 + VLM 产 class/relations，合并成 `Object` |

MJCF 在这套体系里是 Robonix 规范的"默认磁盘格式"——用它作为 save/load 默认序列化路径，并不意味着内存里就用 MJCF。需要切换到另一种原生格式时只挂另一个驱动，`SemanticScene` 接口不变。

## 与其他服务的关系

- `srv/slam/*`：产出几何（point cloud、occupancy grid）。语义地图消费它 + VLM 标注 → `SemanticScene`。
- `srv/navigation/*`：接到"去 A 附近"时先调 `query_by_name("A")` 拿 pose，再喂给 Nav2。
- `srv/memory_search`：历史观察可以回灌 `label` RPC。
- `prm/sensor/camera`：VLM pipeline 拿帧 → 生成 objects/relations 增量 → `label`。

## 当前状态 · TODO

- schema 版本 v1 的字段确定
- `rust/contracts/srv/semantic_map_*.v1.toml` 写完并跑 `rbnx codegen`
- `SemanticScene` 内存结构 Rust + Python 两端实现
- 索引层（FLT）实现
- `save`/`load` MJCF 读写路径
- 至少一个 adapter 跑通：`mjcf → usd` 用于 Isaac Sim demo
- Carter warehouse 作为参考场景写一份 Robonix MJCF

