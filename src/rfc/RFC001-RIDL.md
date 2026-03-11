# RFC 001: Robonix Interface Definition Language (RIDL)：Robonix 接口定义语言

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
- `robonix/system/execution/result_feedback` — 执行结果 event（中断式通知）

### 2.2 与 Python/ROS 的对应关系

- RIDL 命名空间 `robonix/a/b/c` -> Python 模块路径 `robonix.a.b.c`（斜杠变点号）。
- ROS 2 类型名（srv/action/msg）由 codegen 根据 namespace + 接口名生成，如 `robonix/system/debug` + `ping` -> `SystemDebugPing`（具体规则以 ridlc 输出为准）。

---

## 3. 通信原语

RIDL 支持四类通信原语，分别对应不同的通信语义与 ROS 2 映射。

| 原语 | 含义 | 典型用法 | ROS 2 映射 |
|------|------|----------|------------|
| stream | 数据流，单向（每个 stream 仅 output 或仅 input） | 位姿/传感器（output）、控制指令（input） | topic |
| command | 带进度与结果的控制命令 | 运动指令、skill 执行、长时间任务 | action |
| query | 请求-响应、同步 | 状态查询、语义地图查询、调试 ping | service |
| event | 离散、异步事件（见 §3.4） | 结果反馈、告警、中断式通知 | topic |

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

**一个 stream 只有一个方向**：要么 `output`，要么 `input`，不能同时有两者。

`output` 与 `input` 均以 **stream 提供方（provider）** 为视角定义：

| 字段 | 提供方（provider）角色 | 订阅方（consumer）角色 | 数据方向 |
|------|------------------------|------------------------|----------|
| **output** | 发布（publish） | 订阅（subscribe） | provider → consumer |
| **input** | 订阅（subscribe） | 发布（publish） | consumer → provider |

- **output 流**：provider 向外发布，consumer 订阅。典型：位姿、点云、图像、传感器数据。
- **input 流**：provider 接收，consumer 发布。典型：控制指令、参数下发、用户输入。
- 示例：见 §5.3。

### 3.4 event

- **语义**：离散、一次触发、多订阅方。发布方发出一次事件，订阅方异步接收。与 stream 的差别：不强调持续流，强调「发生了某事」的即时通知。
- **RIDL 结构**：`payload` 声明事件载荷类型（单条消息）。
- **适用**：skill 执行结果反馈、任务完成/失败告警、异常上报、状态突变通知等。
- **传输映射**：event 与 stream 相同，使用 **ROS topic** 传输；channel 由 robonix-server 分配。与 stream 的差别在 QoS：event 通常 depth=1（不保留历史）、一次发布即达，stream 则 depth>1 保留最近若干条。
- **载荷格式**：ROS CDR 二进制，与 ROS 2 一致。
- **非功能要求**：低延迟、稳定送达（不丢事件）、一次发布多订阅。

示例：见 §5.4。

---

## 4. 类型系统

- RIDL 不定义独立的数据类型 IDL，而是引用现有 ROS 2 类型。
- **Wire 格式**：所有 RIDL 载荷（query request/response、command goal/feedback/result、stream 消息、event payload）在传输时统一采用 **ROS CDR（Common Data Representation）** 二进制序列化，与 ROS 2 一致；无论是 Zenoh、HTTP、DDS等，传输数据格式以 ROS CDR 为准。
- 使用 `import` 引入所需 message/srv 类型，在 request/response/input/output/result/payload 中引用。
- 引用格式与 ROS 一致：`std_msgs/msg/String`、`geometry_msgs/msg/Twist`、`nav_msgs/msg/Odometry` 等。
- 自定义类型由其他 ROS 包提供（如 `robonix_msg`/`robonix_msgs`），在 RIDL 中按包名/类型路径引用。
- **数组类型**：支持 `[]` 后缀表示数组，如 `robonix_msg/msg/Object[]` 表示 Object 数组。
- **robonix_msg**：`rust/robonix-interfaces/lib/robonix_msg/` 定义了语义相关结构（Object、Relation、FrameMapping、SkillInstance、ServiceInstance、Dict 等），RIDL 中应优先使用这些类型而非全用 `std_msgs/msg/String`。

### 4.1 语义注解（Annotations）

RIDL 支持结构化注解，格式为 `@key` 或 `@key(param=value)`。注解可出现在接口名后、`{` 前。

**实现者职责**：注解由实现者解析并暴露为只读或 RW 的元数据。

#### 4.1.1 接口层次关系

| 注解 | 参数 | 含义 | 实现者暴露 |
|------|------|------|------------|
| `@requires_interface` | `("ns/iface1", "ns/iface2")` 接口列表 | 实现本接口时，必须同时实现所列接口；用于约束实现组合 | 只读 |

示例：机械臂控制 command 需配套参数查询/设置 query，否则实现不完整。

```ridl
command arm_control
    @requires_interface("robonix/hal/arm/get_params", "robonix/hal/arm/set_params")
{
    input cmd robonix_msgs/msg/ArmCommand;
    result status robonix_msgs/msg/CommandResult;
    version 1.0;
}
```

#### 4.1.2 安全扩展（IDL 层）

ROS IDL 无安全语义。RIDL 在 IDL 层扩展安全相关信息，供实现者做参数校验、运行时防护：

| 注解 | 作用域 | 参数 | 含义 |
|------|--------|------|------|
| `@range` | 字段 | 无 | 该字段为有界数值，合法范围由实现者提供（只读或 RW 暴露） |
| `@unit` | 字段 | `("m")` | 物理单位，便于文档与单位换算 |
| `@frame` | 字段 | `("map")` | 该字段（位姿/点）的参考系，强制约定通信通道的坐标系 |

**用法**：注解写在字段类型后。`@frame` 强制要求该通道的位姿/点数据以指定 frame 为参考系，收发双方据此对齐坐标系：

```ridl
stream pose {
    output pose geometry_msgs/msg/PoseStamped @frame("map");
    version 1.0;
}
```

`@range` 仅声明“有界”，具体 min/max 由实现者在运行时或配置中提供，接口定义不写死数值：

```ridl
query set_max_speed {
    request speed float32 @range @unit("m/s");
    response ok std_msgs/msg/Bool;
    version 1.0;
}
```

**设计理由**：接口定义描述契约，不写死具体数值。不同实现（不同硬件、不同安全等级）的合法范围不同，同一接口在不同机器人上 min/max 可能各异。若在 IDL 中写死 `min=0, max=2.0`，则实现者无法表达差异，且接口升级时需改 IDL。因此 `@range` 仅声明“该字段有界、需校验”，实现者通过 `InterfaceDescriptor` 或参数服务暴露实际范围，调用方可据此做入参校验或限幅。

安全相关注解写入 `InterfaceDescriptor`，实现者可按需提供只读（文档）或 RW（可配置限幅）的暴露方式。超时等运行时策略由实现者或配置层决定，不放在接口定义中。

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

#### 5.3.1 仅 output（provider 发布，consumer 订阅）

常见：位姿流、传感器数据、图像流。

```ridl
namespace robonix/hal/localization

import geometry_msgs/msg/PoseWithCovarianceStamped

stream pose {
    output state geometry_msgs/msg/PoseWithCovarianceStamped;
    version 1.0;
}
```

- 接口身份：`robonix/hal/localization/pose`。
- provider：发布 `state`（位姿）；consumer：订阅接收。
- 典型用法：定位节点发布位姿，导航/显示等节点订阅。

#### 5.3.2 仅 input（provider 订阅，consumer 发布）

常见：控制指令流、参数下发、用户输入。

```ridl
namespace robonix/hal/control

import geometry_msgs/msg/Twist

stream velocity_cmd {
    input cmd geometry_msgs/msg/Twist;
    version 1.0;
}
```

- 接口身份：`robonix/hal/control/velocity_cmd`。
- provider：订阅接收 `cmd`（速度指令）；consumer：发布发送。
- 典型用法：底盘控制节点接收速度指令，遥控/导航节点发布。

**说明**：一个 stream 只能声明 `output` 或 `input` 其一，不能同时有两者。双向通信需定义两条 stream。

### 5.4 event 示例

```ridl
namespace robonix/system/execution

import robonix_msg/msg/Dict

event result_feedback {
    payload report robonix_msg/msg/Dict;
    version 1.0;
}
```

- 接口身份：`robonix/system/execution/result_feedback`。
- `payload`：事件载荷，类型为 `robonix_msg/msg/Dict`（可携带 skill 执行结果、任务状态等）。
- 典型用法：skill 执行完成后，向订阅方推送一次 `result_feedback`，实现类似中断的即时通知。

### 5.5 通用语法要点

- 每个文件有且仅有一个 `namespace`，末尾无分号。
- `import` 可多条，格式为 `package/type_kind/TypeName`。
- 接口体大括号内字段以 `;` 结尾；`version` 为必填。
- 字段名（如 `data`、`cmd`、`state`）在 request/response/input/output/result/payload 内唯一即可，codegen 会映射到对应语言字段名。

---

## 6. 运行时 channel

### 6.1 channel 的定义

channel（通道）：某一 interface 在运行时的具体通信端点。

- 对 query / command：channel 即 ROS 2 的 service 名 / action 名（由 robonix-server 分配）。
- 对 stream / event：channel 即 ROS 2 的 topic 名。二者均用 topic 传输，event 与 stream 的差别在 QoS（event 通常 depth=1，不保留历史）。

RIDL 与 package manifest 中不写 channel 名；channel 由 robonix-server 在以下时机分配：

- Server 注册：某 node 提供某 interface 时，向 meta API 注册，获得并对外公布一个 channel（service/topic/action 名）。
- Client 解析：调用方指定“目标 node id + 接口（如 robonix/hal/base/motion_cmd）”后，通过 meta API 解析得到 channel，再发起 ROS 调用。

### 6.2 与实现的衔接

生成代码（ridlc 输出）通过 gRPC meta API：

- Server 端：调用 RegisterQuery / RegisterStream / RegisterCommand / RegisterEvent 等，拿到 channel，再创建 ROS service/topic/action 并绑定。
- Client 端：调用 ResolveQuery / ResolveStream / ResolveCommand / ResolveEvent 等，传入 requester_id、target node id、接口 namespace/name，拿到 channel，再创建 ROS client/subscriber 并调用。
- 所有 RIDL 载荷的 wire 格式统一为 **ROS CDR** 二进制。

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

---

## 附录 A：代码生成规范 {#codegen}

本节规定 RIDL 到 Python/Rust 的生成命名与主要导出项。实现（ridlc）应遵循本规范。

### A.1 命名规律总表

| 原语 | 生成文件名 | Python 主要函数 | Rust 模块后缀 |
|------|------------|-----------------|---------------|
| query | `{name}_query.py` | `create_{name}_client`, `create_{name}_server` | `{name}_query` |
| stream | `{name}_stream.py` | `create_{name}_publisher`, `create_{name}_subscriber` | `{name}_stream` |
| command | `{name}_command.py` | `create_{name}_client`, `create_{name}_server` | `{name}_command` |
| event | `{name}_event.py` | `{Name}Publisher` 类 + `emit(msg)` | `{name}_event` |

- 命名空间 `robonix/a/b/c` → Python 包 `robonix.a.b.c`，Rust 模块 `robonix_a_b_c`（下划线连接）。
- ROS 2 类型名：`namespace` 去掉 `robonix/` 后按层级 PascalCase 拼接 + 接口名，如 `robonix/system/debug` + `ping` → `SystemDebugPing`（srv/action）。

### A.2 query 规范

**RIDL 示例**：

```ridl
namespace robonix/system/debug
import std_msgs/msg/String
query ping {
    request data std_msgs/msg/String;
    response data std_msgs/msg/String;
    version 1.0;
}
```

**Python 生成**：

| 项目 | 规范 |
|------|------|
| 模块路径 | `robonix.system.debug.ping_query` |
| 文件 | `ping_query.py` |
| 工厂函数 | `create_ping_client(runtime_client, requester_id, target)` → 返回带 `call(request)` 的 client |
| | `create_ping_server(runtime_client, node_id)` → 返回带 `start(handler)` 的 server |
| 辅助函数 | `resolve_ping_service(...)`, `register_ping_server(...)` |

**Rust 生成**（robonix-server）：

| 项目 | 规范 |
|------|------|
| 模块路径 | `robonix_server::generated::robonix_system_debug::ping_query` |
| 主要项 | `register(...)`, `resolve(...)`, `Request`, `Response`, `QueryService`, `DESCRIPTOR` |

### A.3 stream 规范

**input/output 方向**：一个 stream 只有一个方向（output 或 input，不能同时有）。以 provider 为视角：`output` = provider 发布、consumer 订阅；`input` = provider 订阅、consumer 发布。见 RFC001 §3.3。

**RIDL 示例**（仅 output）：

```ridl
namespace robonix/hal/localization
import geometry_msgs/msg/PoseWithCovarianceStamped
stream pose {
    output state geometry_msgs/msg/PoseWithCovarianceStamped;
    version 1.0;
}
```

**Python 生成**（output 流）：

| 项目 | 规范 |
|------|------|
| 模块路径 | `robonix.hal.localization`（从 `pose_stream` 导入） |
| 文件 | `pose_stream.py` |
| 工厂函数 | `create_pose_publisher(runtime_client, node_id)` → provider 发布 |
| | `create_pose_subscriber(runtime_client, requester_id, target)` → consumer 订阅 |
| 辅助函数 | `register_pose_provider(...)`, `resolve_pose_consumer_topic(...)` |

**仅 input 流**：生成 `create_{name}_subscriber`（provider 用）与 `create_{name}_publisher`（consumer 用），方向与 output 流相反。

**Rust 生成**：

| 项目 | 规范 |
|------|------|
| 模块路径 | `robonix_server::generated::robonix_hal_localization::pose_stream` |
| 主要项 | `Payload`（消息类型）, `DESCRIPTOR` |

### A.4 command 规范

**RIDL 示例**：

```ridl
namespace robonix/hal/base
command motion_cmd {
    input cmd geometry_msgs/msg/Twist;
    output odom nav_msgs/msg/Odometry;
    result status robonix_msgs/msg/CommandResult;
    version 1.0;
}
```

**Python 生成**：

| 项目 | 规范 |
|------|------|
| 模块路径 | `robonix.hal.base.motion_cmd_command` |
| 文件 | `motion_cmd_command.py` |
| 工厂函数 | `create_motion_cmd_client(runtime_client, requester_id, target)` → 返回带 `send(goal)` 的 client |
| | `create_motion_cmd_server(runtime_client, node_id)` → 返回带 `execute` 回调的 server |
| 辅助函数 | `register_motion_cmd_server(...)`, `resolve_motion_cmd_client_action(...)` |

**Rust 生成**：

| 项目 | 规范 |
|------|------|
| 模块路径 | `robonix_server::generated::robonix_hal_base::motion_cmd_command` |
| 主要项 | `Goal`, `Feedback`, `Result`, `DESCRIPTOR` |

### A.5 event 规范

**RIDL 示例**：

```ridl
namespace robonix/system/execution
import robonix_msg/msg/Dict
event result_feedback {
    payload report robonix_msg/msg/Dict;
    version 1.0;
}
```

**Python 生成**：

| 项目 | 规范 |
|------|------|
| 模块路径 | `robonix.system.execution.result_feedback_event` |
| 文件 | `result_feedback_event.py` |
| 主要类 | `ResultFeedbackPublisher(topic_name, msg_type)`，方法 `emit(msg)` |
| 说明 | event 无 `create_*` 工厂，需配合 runtime 注册获取 topic 后实例化 Publisher |

**Rust 生成**：

| 项目 | 规范 |
|------|------|
| 模块路径 | `robonix_server::generated::robonix_system_execution::result_feedback_event` |
| 主要项 | `Payload`, `DESCRIPTOR` |

### A.6 调用约定（Python）

- **query**：`client.call(request, timeout_sec=10.0)` 阻塞至响应；`server.start(handler)` 绑定 `handler(request) -> response`。
- **command**：`client.send(goal)` 返回 goal_handle，可 `get_result_async()`；`server.execute` 为回调，接收 request 返回 result。
- **stream**：`publisher.publish(msg)`；`subscriber.start(callback)` 中 `callback(msg)` 处理每条消息。
