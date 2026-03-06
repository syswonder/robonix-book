# robonix-sdk 使用说明

<!-- toc -->

robonix-sdk 是 Robonix 的 ROS2 接口包，提供与 robonix-core 及标准能力交互所需的消息（msg）、服务（srv）定义，以及 Python 客户端辅助库。算法与服务开发、技能开发、原语查询等都会用到。

## 用户配置 SDK 路径

使用依赖 ROS2 的包或通过 rbnx 启动 provider 能力（原语/服务/技能）时，需要让系统能找到 robonix-sdk（以便 source `install/setup.bash`、加载 msg/srv 与 Python 模块）。推荐通过 rbnx 统一配置，供 rbnx 与各包内的 start 脚本使用。

配置方式：

- rbnx config（推荐）：在 `robonix/rust` 目录下执行 `rbnx config --set-sdk-path $(pwd)/robonix-sdk`。路径须指向包含 `install/setup.bash` 的 robonix-sdk 目录。配置写入 `~/.robonix/config.yaml`，rbnx 与多数 provider 的 start 脚本会读取该配置并设置环境变量 `ROBONIX_SDK_PATH`。
- 环境变量：若不使用 config，可在启动 robonix-core 或执行 `rbnx deploy start` 前，在当前 shell 中设置 `export ROBONIX_SDK_PATH=/path/to/robonix-sdk`。部分 start 脚本会优先使用该变量以 source SDK。

查看当前配置：`rbnx config --show` 会显示已设置的 Robonix SDK path；未设置时显示为 "(not set)"。

使用前：在需要调用 ROS2 或 `robonix_sdk` 的终端中，先 `source $ROBONIX_SDK_PATH/install/setup.bash`（若已通过 rbnx config 或环境变量设置 `ROBONIX_SDK_PATH`），或在 `robonix/rust` 目录下执行 `eval $(make source-sdk)`。

## 构建与环境

在 `robonix/rust` 目录执行 `make build-sdk`（需已安装并 source ROS2）。使用前按上节配置 SDK 路径，并在需要调用 ROS2 或 robonix_sdk 的终端中 source `robonix-sdk/install/setup.bash`。

## 消息类型（msg）

以下为 robonix-sdk 常用 msg（包名 `robonix_sdk`），每个类型的字段与内部结构见下表。复合类型中嵌套的 msg 在对应小节单独列出。

### 基础几何与枚举

| 类型 | 字段 | 数据类型 | 说明 |
|------|------|----------|------|
| `Point3D` | `x`, `y`, `z` | `float64` | 3D 中心点坐标 |
| `BoundingBox` | `scale_x`, `scale_y`, `scale_z` | `float64` | 3D 包围框尺度 |
| | `yaw` | `float64` | 偏航角 |
| `RelationType` | `type` | `uint8` | 枚举：`CHILD_OF=0`, `ON_TOP=1`, `INSIDE=2`, `NEAR=3`, `CUSTOM=4` |
| | `custom_type` | `string` | 仅当 `type == CUSTOM` 时使用 |
| `TextureType` | `type` | `uint8` | 枚举：`WOOD=0`, `PLASTIC=1`, `METAL=2`, `UNKNOWN=3` |

### 关系与帧

| 类型 | 字段 | 数据类型 | 说明 |
|------|------|----------|------|
| `Relation` | `relation_type` | `RelationType` | 关系类型（见上表） |
| | `target_entity_id` | `string` | 目标实体 ID |
| `FrameMapping` | `center` | `Point3D` | 在 3D 空间中的中心 |
| | `bbox` | `BoundingBox[]` | 包围框列表 |
| | `texture` | `TextureType[]` | 纹理类型列表 |
| | `frame_id` | `string` | 坐标系 ID |

### 语义地图物体（Object）

| 字段 | 数据类型 | 说明 |
|------|----------|------|
| `id` | `string` | 物体唯一 ID |
| `label` | `string` | 类别标签（如 "table", "room"） |
| `relations` | `Relation[]` | 与其他实体的关系列表（见 Relation 结构） |
| `registered_skills` | `string[]` | 在该物体上注册的技能名列表 |
| `registered_primitives` | `string[]` | 在该物体上注册的原语名列表 |
| `frame_mapping` | `FrameMapping[]` | 与 3D 空间的帧映射（见 FrameMapping 结构） |

### 空间地图相关

| 类型 | 字段 | 数据类型 | 说明 |
|------|------|----------|------|
| `SpatialMapEntry` | `frame_id` | `string` | 参考帧 ID |
| | `timestamp` | `uint64` | Unix 时间戳（纳秒） |
| | `source_skill` | `string` | 提供该地图的技能 |
| `Occupancy3D` | `header` | `std_msgs/Header` | 标准头 |
| | `width`, `height`, `depth` | `uint32` | 3D 栅格维度 |
| | `resolution` | `float32` | 分辨率 |
| | `origin` | `geometry_msgs/Point` | 原点 |
| | `data` | `int8[]` | 占用栅格数据 |

### 能力查询返回的实例

| 类型 | 字段 | 数据类型 | 说明 |
|------|------|----------|------|
| `PrimitiveInstance` | `provider` | `string` | 提供方标识 |
| | `version` | `string` | 实现版本 |
| | `input_schema` | `string` | JSON：输入参数名与 Topic 等的映射 |
| | `output_schema` | `string` | JSON：输出参数名与 Topic 等的映射 |
| | `metadata` | `string` | JSON：元数据，供过滤 |
| `ServiceInstance` | `provider` | `string` | 提供方标识 |
| | `version` | `string` | 实现版本 |
| | `entry` | `string` | 实际 ROS2 服务名（调用时用） |
| | `metadata` | `string` | JSON：元数据 |
| `SkillInstance` | `skill_id` | `string` | 技能实例 ID |
| | `provider` | `string` | 提供方标识 |
| | `version` | `string` | 实现版本 |
| | `type` | `string` | `"basic"` 或 `"rtdl"` |
| | `start_topic` | `string` | 启动 Topic |
| | `status_topic` | `string` | 状态反馈 Topic |
| | `entry` | `string` | basic 技能入口（type=basic 时） |
| | `skill_dir` | `string` | RTDL 技能目录（type=rtdl 时） |
| | `main_rtdl` | `string` | RTDL 主文件名（type=rtdl 时） |
| | `start_args` | `string` | JSON：启动参数 schema |
| | `status` | `string` | JSON：状态 schema |
| | `metadata` | `string` | JSON：元数据 |

### 通用

| 类型 | 字段 | 数据类型 | 说明 |
|------|------|----------|------|
| `Dict` | `keys` | `string[]` | 键列表 |
| | `values` | `string[]` | 值列表（与 keys 一一对应，用于 srv 中 JSON 形参数） |

---

## 服务类型（srv）

以下为 robonix-sdk 中与 core、任务、标准算法相关的 srv。请求/响应字段均用表格列出；响应中返回的 `PrimitiveInstance[]`、`ServiceInstance[]`、`SkillInstance[]` 等，其单条结构见上一节对应 msg 表。

### 原语（Primitive）

| 服务 | 方向 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| `RegisterPrimitive`<br>`/rbnx/prm/register` | 请求 | `name` | `string` | 标准原语名 |
| | | `input_schema` | `string` | JSON：`{"argname":"/topic", ...}` |
| | | `output_schema` | `string` | JSON：同上 |
| | | `metadata` | `string` | JSON：供查询时过滤 |
| | | `provider` | `string` | 提供方标识 |
| | | `version` | `string` | 版本号 |
| | 响应 | `ok` | `bool` | 是否成功 |
| `QueryPrimitive`<br>`/rbnx/prm/query` | 请求 | `name` | `string` | 标准原语名 |
| | | `filter` | `string` | JSON 过滤（如 `{"resolution":">=720p"}`），空串表示不过滤 |
| | 响应 | `instances` | `PrimitiveInstance[]` | 匹配的实例列表（结构见 msg 小节 PrimitiveInstance） |

### 服务（Service）

| 服务 | 方向 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| `RegisterService`<br>`/rbnx/srv/register` | 请求 | `name` | `string` | 标准服务名 |
| | | `srv_type` | `string` | ROS2 服务类型全名（如 `robonix_sdk/srv/service/spatial_map/GetSpatialMap`） |
| | | `entry` | `string` | 实际 ROS2 服务名 |
| | | `metadata` | `string` | JSON：供查询时过滤 |
| | | `provider` | `string` | 提供方标识 |
| | | `version` | `string` | 版本号 |
| | 响应 | `ok` | `bool` | 是否成功 |
| `QueryService`<br>`/rbnx/srv/query` | 请求 | `name` | `string` | 标准服务名 |
| | | `filter` | `string` | JSON 过滤，空串表示不过滤 |
| | 响应 | `instances` | `ServiceInstance[]` | 匹配的实例列表（结构见 msg 小节 ServiceInstance） |

### 技能（Skill）

| 服务 | 方向 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| `RegisterSkill`<br>`/rbnx/skl/register` | 请求 | `name` | `string` | 技能名 |
| | | `type` | `string` | `"basic"` 或 `"rtdl"` |
| | | `start_topic` | `string` | 启动 Topic |
| | | `status_topic` | `string` | 状态反馈 Topic |
| | | `entry` | `string` | basic 时必填：入口命令/脚本 |
| | | `skill_dir` | `string` | rtdl 时必填：技能目录路径 |
| | | `main_rtdl` | `string` | rtdl 时必填：主 RTDL 文件名 |
| | | `start_args` | `string` | JSON：启动参数 schema |
| | | `status` | `string` | JSON：状态 schema |
| | | `metadata` | `string` | JSON：供查询时过滤 |
| | | `provider` | `string` | 提供方标识 |
| | | `version` | `string` | 版本号 |
| | 响应 | `ok` | `bool` | 是否成功 |
| | | `skill_id` | `string` | 注册得到的技能实例 ID |
| `QuerySkill`<br>`/rbnx/skl/query` | 请求 | `name` | `string` | 技能名 |
| | | `filter` | `string` | JSON 过滤，空串表示不过滤 |
| | 响应 | `instances` | `SkillInstance[]` | 匹配的实例列表（结构见 msg 小节 SkillInstance） |

### 任务（Task）

| 服务 | 方向 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| `SubmitTask`<br>`/rbnx/task/submit` | 请求 | `description` | `string` | 自然语言任务描述 |
| | | `params` | `string` | 可选 JSON 参数 |
| | 响应 | `task_id` | `string` | 任务 ID |
| `TaskData`<br>`/rbnx/task/...` | 请求 | `task_id` | `string` | 任务 ID |
| | 响应 | `data` | `string` | 完整任务数据（JSON） |

### 标准算法服务（供实现方实现、core/规划器调用）

| 服务 | 方向 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| `QuerySemanticMap`<br>`srv::semantic_map` | 请求 | `types` | `string[]` | 按物体类型过滤（如 `["room","window","robot"]`） |
| | 响应 | `objects` | `Object[]` | 语义物体列表（结构见 msg 小节 Object） |
| | | `stamp` | `builtin_interfaces/Time` | 时间戳 |
| `PlanTask`<br>`srv::task_plan` | 请求 | `description` | `string` | 自然语言任务描述 |
| | | `params` | `Dict` | 可选参数（见 msg Dict） |
| | 响应 | `rtdl` | `string` | RTDL 代码 |
| | | `rtdl_type` | `string` | RTDL 风格 |
| | | `stamp` | `builtin_interfaces/Time` | 时间戳 |
| `GetSpatialMap`<br>`srv::spatial_map` | 请求 | `map_type` | `uint8` | `0`=2D 占用栅格，`1`=3D 占用栅格，`2`=点云 |
| | 响应 | `occupancy_grid` | `nav_msgs/OccupancyGrid` | 当 map_type=0 时有效 |
| | | `occupancy_3d` | `Occupancy3D` | 当 map_type=1 时有效（见 msg 小节） |
| | | `cloud` | `sensor_msgs/PointCloud2` | 当 map_type=2 时有效 |
| | | `stamp` | `builtin_interfaces/Time` | 时间戳 |
| `TransformScan`<br>`srv::transform.scan` | 请求 | `action` | `uint8` | `0`=CONVERT（单次），`1`=START_STREAM，`2`=STOP_STREAM |
| | | `input_topic` | `string` | CONVERT 时若非空则用该 Topic 最新一帧；START_STREAM 时为点云源 Topic |
| | | `pointcloud` | `sensor_msgs/PointCloud2` | CONVERT 且 input_topic 为空时使用 |
| | | `stream_id` | `string` | STOP_STREAM 时传入 START_STREAM 返回的 id；START_STREAM 时留空由服务端分配 |
| | 响应 | `success` | `bool` | 是否成功 |
| | | `scan` | `sensor_msgs/LaserScan` | CONVERT 时返回结果 |
| | | `output_topic` | `string` | START_STREAM 时返回的专属输出 Topic |
| | | `stream_id` | `string` | START_STREAM 时返回，用于后续 STOP_STREAM 释放 |

SimulatePlan、ResultFeedback、QueryMemory 等 srv 的字段定义见仓库内 `robonix-sdk/srv/service/` 下对应 `.srv` 文件。

## Python 库（robonix_sdk.client）

在 Python 节点中可 `from robonix_sdk.client import RobonixClient`，配合已创建的 ROS2 节点使用：

- 创建查询客户端：`client.create_query_primitive_client(service_name="/rbnx/prm/query")`
- `query_primitive(primitive_name, filter_dict=None, ...)`：带重试地调用 QueryPrimitive，返回 response 或 None；可选 `raise_on_error`、`max_retries`、`wait_timeout`、`call_timeout`。
- `get_primitive_output_schema(response, instance_index=0)`：从 QueryPrimitive 响应中解析出 `output_schema` 字典。
- `query_primitive_and_get_schema(...)`：先 query 再返回 output_schema。
- `query_primitive_and_extract_field(primitive_name, field_name, filter_dict=None, ...)`：查询并取出 `output_schema` 中指定字段（如 topic 路径）。

示例（在 ROS2 节点内）：

```python
from robonix_sdk.client import RobonixClient

# self 为 rclpy Node
client = RobonixClient(self)
response = client.query_primitive("prm::camera.rgb", filter_dict={"robot": "tiago"})
if response and response.instances:
    schema = client.get_primitive_output_schema(response)
    image_topic = schema.get("image")  # 例如 "/head_front_camera/rgb/image_raw"
```

服务和技能的查询与调用目前没有与 QueryPrimitive 同等级的 Python 封装，可直接使用 `rclpy` 的 `create_client`、`create_service` 配合上述 srv 类型进行调用。

## 与 CLI / core 的关系

`rbnx deploy register` 会通过 daemon 调用 core 的 RegisterPrimitive、RegisterService、RegisterSkill 等服务，所用字段与上述 srv 定义一致。能力的提供方（例如语义地图服务、技能进程）通过 QueryPrimitive、QueryService、QuerySkill 解析出 Topic 或服务名后，再与硬件或算法交互。任务执行器在运行 RTDL 时，通过 QuerySkill 解析技能名得到 `start_topic`、`status_topic`，再通过 Topic 发布与订阅触发技能并等待状态。
