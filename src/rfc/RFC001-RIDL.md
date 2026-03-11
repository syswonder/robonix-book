# RFC001: Robonix Interface Definition Language (RIDL)

| 版本 | 日期 | 作者 |
|------|------|------|
| 0.1 | 2026-03-11 | 韩喻泷 |

<!-- toc -->

## 1. 目标与范围

### 1.1 目标

RIDL（Robonix Interface Definition Language）定义接口契约：接口身份（命名空间 + 名称）、通信语义（stream/command/query/event）、请求与载荷的类型、以及接口版本。RIDL 仅描述“有什么接口、长什么样”；不定义 package 如何构建、不定义 channel 名或部署拓扑，channel 由运行时分配（见 §6）。

### 1.2 不在本 RFC 范围内的内容

- Package 目录结构、构建流程、manifest 格式 -> RFC002
- 运行时 channel 分配与解析的实现细节 -> robonix-server / meta API 文档
- 具体编程语言绑定（Python/Rust 生成规则）→ 以实现为准，本文给出语义与命名约定

---

## 2. 命名空间

### 2.1 规范格式

Canonical 格式：`robonix/{domain}/{subdomain}/...`，层级用 `/` 分隔，全部小写，子域可多级。

示例：

- `robonix/hal/base` — 硬件抽象层基础
- `robonix/hal/base/status` — 设备状态 query
- `robonix/hal/localization/pose` — 定位 pose stream
- `robonix/system/debug/ping` — 调试用 ping query
- `robonix/system/map/semantic_query` — 语义地图 query
- `robonix/system/skill/execute` — skill 执行 command

### 2.2 与 Python/ROS 的对应关系

- RIDL 命名空间 `robonix/a/b/c` -> Python 模块路径 `robonix.a.b.c`（斜杠变点号）。
- ROS 2 类型名（srv/action/msg）由 codegen 根据 namespace + 接口名生成，如 `robonix/system/debug` + `ping` -> `SystemDebugPing`（具体规则以 ridlc 输出为准）。

---

## 3. 通信原语

RIDL 支持四类通信原语，分别对应不同的通信语义与 ROS 2 映射。

| 原语 | 含义 | 典型用法 | ROS 2 映射 |
|------|------|----------|------------|
| stream | 数据流，可单向或双向 | 位姿/传感器（output）、双向流（input+output） | topic |
| command | 带进度与结果的控制命令 | 运动指令、skill 执行、长时间任务 | action |
| query | 请求-响应、同步 | 状态查询、语义地图查询、调试 ping | service |
| event | 离散、异步事件 | 结果反馈、告警 | topic |

### 3.1 query

- 语义：一对一、同步请求-响应。调用方阻塞直到得到响应或超时。
- RIDL 结构：`request` 与 `response` 各声明若干字段，类型引用 ROS message。
- 适用：状态查询、配置查询、简单 RPC 式调用（如 ping）。
- 示例：见 §5.1。

### 3.2 command

- 语义：带输入、可选进度反馈、最终结果的长时间操作。调用方可等待 result 或取消。
- RIDL 结构：`input`（请求）、`output`（进度反馈）、`result`（最终结果），类型均为 ROS message。
- 适用：机械臂运动、skill 执行、导航等。
- 示例：见 §5.2。

### 3.3 stream

- 语义：数据流，可为单向（仅 output 或仅 input）或双向（同时有 input 与 output）。生产者发布、消费者订阅；若有 input，则订阅方也可向该流发送数据。
- RIDL 结构：`output` 声明“发布出去”的消息类型，`input` 声明“接收进来”的消息类型，二者均可选，但至少有一个。
- 适用：位姿流、点云、图像流（仅 output）；双向控制/反馈流（input + output）。
- 示例：见 §5.3。

### 3.4 event

- 语义：离散事件，一次发布、多次订阅。与 stream 的差别在于“离散”与“事件”语义（可扩展 QoS/重放等）。
- RIDL 结构：`payload` 声明事件载荷类型。
- 映射：当前实现为 topic。

---

## 4. 类型系统

- RIDL 不定义独立的数据类型 IDL，而是引用现有 ROS 2 类型。
- 使用 `import` 引入所需 message/srv 类型，在 request/response/input/output/result/payload 中引用。
- 引用格式与 ROS 一致：`std_msgs/msg/String`、`geometry_msgs/msg/Twist`、`nav_msgs/msg/Odometry` 等。
- 自定义类型由其他 ROS 包提供（如 `robonix_msg`/`robonix_msgs`），在 RIDL 中按包名/类型路径引用。
- **数组类型**：支持 `[]` 后缀表示数组，如 `robonix_msg/msg/Object[]` 表示 Object 数组。
- **robonix_msg**：`rust/robonix-interfaces/lib/robonix_msg/` 定义了语义相关结构（Object、Relation、FrameMapping、SkillInstance、ServiceInstance、Dict 等），RIDL 中应优先使用这些类型而非全用 `std_msgs/msg/String`。

### 4.1 语义注解（Annotations）

ROS IDL 仅有 `#` 注释，无法被工具链解析。RIDL 支持结构化注解，供文档生成、IDE 提示、运行时策略与校验使用。

#### 4.1.1 字段级注解

在字段类型后追加 `@key` 或 `@key("value")`：

```ridl
query semantic_query {
    request filter std_msgs/msg/String @doc("JSON filter, e.g. {\"room\":\"kitchen\"}");
    response objects robonix_msg/msg/Object[] @ref("Object.id") @doc("Objects matching filter");
    version 1.0;
}
```

| 注解 | 含义 | ROS 能否表达 |
|------|------|--------------|
| `@doc("...")` | 字段说明，可被生成文档/IDE 提示 | 否（comment 非结构化） |
| `@ref("Object.id")` | 该字段为对 Object 的引用，语义上对应 Object.id | 否 |
| `@frame("map")` | 位姿/点所在坐标系 | 否 |
| `@unit("m")` | 物理单位 | 否 |

#### 4.1.2 接口级注解

在接口名后、大括号前追加注解：

```ridl
query semantic_query
    @idempotent
    @cacheable(ttl=60)
{
    request filter std_msgs/msg/String;
    response objects robonix_msg/msg/Object[];
    version 1.0;
}

command execute
    @skill
    @timeout(30)
    @requires("semantic_map")
{
    input goal robonix_msgs/msg/SkillGoal;
    result status robonix_msgs/msg/CommandResult;
    version 1.0;
}
```

| 注解 | 含义 | ROS 能否表达 |
|------|------|--------------|
| `@idempotent` | 幂等，可安全重试 | 否 |
| `@cacheable(ttl=N)` | 响应可缓存 N 秒 | 否 |
| `@skill` | 该 command 为 skill（与 primitive 区分） | 否 |
| `@timeout(N)` | 默认超时秒数 | 否 |
| `@requires("X")` | 调用方需具备能力 X | 否 |

注解会写入生成的 `InterfaceDescriptor`，供运行时与文档使用。wire 格式仍为 ROS msg/srv/action，与 ROS 互通。

### 4.2 引用类型（Semantic References）

ROS 仅有具体 msg 类型。RIDL 可定义「引用类型」，在语义上指向运行时实体，而非完整数据结构。

```ridl
query get_object {
    request id ObjectRef;           // 语义：引用 Object，wire 格式可为 string
    response obj robonix_msg/msg/Object;
    version 1.0;
}

command move_to {
    input target FrameRef;          // 语义：引用地图中的 frame
    result status robonix_msgs/msg/CommandResult;
    version 1.0;
}
```

| 类型 | 含义 | Wire 格式（可映射） | ROS 能否表达 |
|------|------|---------------------|--------------|
| `ObjectRef` | 语义地图中 Object 的引用 | string (Object.id) | 否（ROS 只有 string，无语义） |
| `FrameRef` | 坐标系引用 | string (frame_id) | 否 |
| `SkillRef` | 技能实例引用 | string (skill_id) | 否 |

codegen 将引用类型映射到 `std_msgs/msg/String` 或 robonix_msg 中对应类型，但生成代码/文档中保留「引用」语义，便于校验与工具链。

---

## 5. 语法示例

### 5.1 query 示例

```ridl
namespace robonix/system/debug

import std_msgs/msg/String

query ping {
    request data std_msgs/msg/String;
    response data std_msgs/msg/String;
    version 1.0;
}
```

- 接口身份：`robonix/system/debug/ping`。
- 请求、响应各一个字段 `data`，类型为 `std_msgs/msg/String`。
- `version` 为接口版本号，用于兼容与演进。

### 5.1.1 semantic_query（使用 robonix_msg 与数组）

```ridl
namespace robonix/system/map

import std_msgs/msg/String
import robonix_msg/msg/Object

query semantic_query {
    request filter std_msgs/msg/String;
    response objects robonix_msg/msg/Object[];
    version 1.0;
}
```

- 响应 `objects` 为 `Object[]`，使用 `robonix_msg` 中定义的语义对象结构（含 id、label、relations 等）。

### 5.2 command 示例

```ridl
namespace robonix/hal/base

import geometry_msgs/msg/Twist
import nav_msgs/msg/Odometry
import robonix_msgs/msg/CommandResult

command motion_cmd {
    input cmd geometry_msgs/msg/Twist;
    output odom nav_msgs/msg/Odometry;
    result status robonix_msgs/msg/CommandResult;
    version 1.0;
}
```

- 接口身份：`robonix/hal/base/motion_cmd`。
- `input`：下发运动指令（Twist）。
- `output`：执行过程中的进度反馈（如 Odometry）。
- `result`：最终执行结果（CommandResult）。

### 5.3 stream 示例

仅 output（常见：发布位姿）：

```ridl
namespace robonix/hal/localization

import geometry_msgs/msg/PoseWithCovarianceStamped

stream pose {
    output state geometry_msgs/msg/PoseWithCovarianceStamped;
    version 1.0;
}
```

- 接口身份：`robonix/hal/localization/pose`。
- `output`：流中“发布出去”的消息类型。

也可声明 `input`（接收方向的消息类型）；stream 允许仅有 output、仅有 input、或两者皆有，至少其一。

### 5.4 通用语法要点

- 每个文件有且仅有一个 `namespace`，末尾无分号。
- `import` 可多条，格式为 `package/type_kind/TypeName`。
- 接口体大括号内字段以 `;` 结尾；`version` 为必填。
- 字段名（如 `data`、`cmd`、`state`）在 request/response/input/output/result/payload 内唯一即可，codegen 会映射到对应语言字段名。

---

## 6. 运行时 channel

### 6.1 channel 的定义

channel（通道）：某一 interface 在运行时的具体通信端点。

- 对 query / command：channel 即 ROS 2 的 service 名 / action 名（由 robonix-server 分配）。
- 对 stream / event：channel 即 ROS 2 的 topic 名。

RIDL 与 package manifest 中不写 channel 名；channel 由 robonix-server 在以下时机分配：

- Server 注册：某 node 提供某 interface 时，向 meta API 注册，获得并对外公布一个 channel（service/topic/action 名）。
- Client 解析：调用方指定“目标 node id + 接口（如 robonix/hal/base/motion_cmd）”后，通过 meta API 解析得到 channel，再发起 ROS 调用。

### 6.2 与实现的衔接

生成代码（ridlc 输出）通过 gRPC meta API：

- Server 端：调用 RegisterQuery / RegisterStream / RegisterCommand 等，拿到 channel，再创建 ROS service/topic/action 并绑定。
- Client 端：调用 ResolveQuery / ResolveStream / ResolveCommand 等，传入 requester_id、target node id、接口 namespace/name，拿到 channel，再创建 ROS client/subscriber 并调用。

因此 RIDL 只描述接口形态；channel 的分配、解析与生命周期由运行时与生成代码负责。

---

## 7. 接口版本与演进

- 每个接口声明中有 `version` 字段（如 `1.0`），用于标识该接口定义的版本。
- 后续若 request/response 或字段类型发生不兼容变更，应递增版本或新命名空间，以便实现与调用方协同演进。具体版本策略可另行约定。

---

## 8. 与 RFC002 的边界

- RFC001：仅定义接口契约（RIDL 语法、原语、类型、channel 概念）；不涉及 package 目录、manifest、构建与启停。
- RFC002：定义 package、manifest、node、entry、rbnx build/start/stop；不涉及 RIDL 语法或 channel 分配算法。

两者通过“接口身份（命名空间 + 名称）”与“node id”在运行时衔接：调用方指定 (node_id, interface_id)，解析得到 channel，再完成通信。
