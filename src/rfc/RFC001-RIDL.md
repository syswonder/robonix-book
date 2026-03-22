# RFC 001: Robonix Interface Definition Language (RIDL)

| 版本 | 日期 | 作者 |
|------|------|------|
| 0.1 | 2026-03-11 | 韩喻泷 |

## 1. 目标与范围

RIDL 定义接口契约：接口身份（命名空间 + 名称）、通信语义（stream/command/query）、载荷类型、接口版本。RIDL 仅描述"有什么接口、长什么样"；不定义 package 构建、channel 名或部署拓扑，channel 由运行时分配（§7）。

**不在本 RFC 内**：Package 目录与 manifest（RFC002）、channel 分配实现细节、ridlc 代码生成与通信实现（见开发手册）。

---

## 2. 命名空间

格式：`robonix/{domain}/{subdomain}/...`，层级用 `/` 分隔，小写。

示例：`robonix/prm/base`、`robonix/system/debug/ping`、`robonix/system/map/semantic_query`。

---

## 3. 语法速览

非正式说明，便于快速上手。

### 3.1 接口语法

```
command 接口名 [注解...] {
    input 字段名 类型 [注解...];      // 至少 input 或 result 之一
    [output 字段名 类型 [注解...];]   // 可选，进度反馈
    [result 字段名 类型 [注解...];]
    version 版本号;
}

stream 接口名 [注解...] {
    output 字段名 类型 [注解...];    // 或 input，二选一
    version 版本号;
}

query 接口名 [注解...] {
    request 字段名 类型 [注解...];
    response 字段名 类型 [注解...];
    version 版本号;
}
```

注解为 `@key` 或 `@key(param=value)`，可多个。接口级写在接口名后、`{` 前；字段级写在字段类型后、`;` 前。`@desc("...")` 为 LLM/Agent 友好描述，与 `@frame`、`@interruptible` 等并列。

### 3.2 文件结构

```
namespace robonix/域/子域
[import 类型路径;]...

接口定义（见上）
```

### 3.3 规则摘要

- **关键词**：`namespace`、`import`、`stream`、`command`、`query`、`input`、`output`、`result`、`request`、`response`、`version`。注解 `@desc` 等。
- **文件规则**：一文件一接口；接口名必须与文件名一致（如 `depth.ridl` 定义 `stream depth`）。
- **命名**：namespace 小写、用 `/` 分隔；接口名、字段名小写或 snake_case；类型用 `package/msg/Name` 或 import 后短名。
- **字段**：以 `;` 结尾。

---

## 4. 通信语义（stream / command / query）

> 说明：本节说的是**单个 RIDL 接口**的通信形态（stream、command、query）。在 Robonix 文档中，**「原语」一词保留给硬件抽象层**（如 `robonix/prm/*` 下的相机、底盘、机械臂等接口集合），请勿与下表三者的名称混用。

RIDL 用下列三种**通信语义**描述接口，均为**传输无关**的抽象；具体承载方式由 codegen/运行时选择（当前参考实现常对应 ROS 2 的 topic、action、service）。

| 通信语义 | 含义 | 典型用法 |
|----------|------|----------|
| stream | 单向数据流 | 位姿/传感器、控制指令 |
| command | 长任务：输入 + 进度 + 结果 | 运动、skill 执行 |
| query | 同步请求-响应 | 状态查询、ping |

### 4.1 query

一对一、同步请求-响应。RIDL 结构：`request`、`response` 各声明字段，类型引用 message。

### 4.2 command

带输入、可选进度反馈、最终结果的长任务。RIDL 结构：`input`、`output`（进度）、`result`。

### 4.3 stream

**单向**：每个 stream 仅 `output` 或仅 `input`。以 provider 为视角：`output` = provider 发布；`input` = provider 订阅。

---

## 5. 类型系统

- RIDL 不定义独立 IDL，引用兼容 ROS IDL 规范的类型（`package/msg/Name`、`package/srv/Name`）。
- **Wire 格式**：具体序列化与传输由实现决定。
- **数组**：`[]` 后缀，如 `robonix_msg/msg/Object[]`。
- **robonix_msg**：定义 Object、Relation、FrameMapping、SkillInstance、Dict 等语义结构，RIDL 中优先使用。

### 5.1 类型引用与 import

`import` 引入类型后，字段中可使用**短名**（仅类型名），无需写全称：

```ridl
namespace robonix/prm/base
import geometry_msgs/msg/PoseStamped

command navigate @interruptible {
    input goal PoseStamped;   // 短名，等价于 geometry_msgs/msg/PoseStamped
    version 1.0;
}
```

全称 `geometry_msgs/msg/PoseStamped` 仍可使用。短名与全称可混用。

### 5.2 注解

格式 `@key` 或 `@key(param=value)`。接口级注解写在接口名后、`{` 前；字段级注解写在字段类型后、`;` 前。

| 注解 | 作用域 | 参数 | 含义 |
|------|--------|------|------|
| `@desc` | 接口/字段 | `("...")` | **LLM/Agent 友好**：接口用途或字段含义、格式、示例，供 AI 理解与正确构造请求 |
| `@requires_interface` | 接口 | `("ns/iface1", "ns/iface2")` | 实现本接口时必须同时实现所列接口 |
| `@frame` | 字段 | `("map")` | 字段（位姿/点）的参考系，收发双方据此对齐坐标系 |
| `@interruptible` | 接口 | 无 | command 支持运行时中断（如 cancel）；ROS action 有 cancel API 但 IDL 不声明 |

```ridl
// navigate.ridl
namespace robonix/hal/base
import geometry_msgs/msg/PoseStamped
command navigate @interruptible @desc("Navigate robot to goal pose.") {
    input goal PoseStamped @desc("Target pose (frame_id, position, orientation)");
    version 1.0;
}

// arm_control.ridl
namespace robonix/hal/arm
import robonix_msg/msg/ArmCommand
import robonix_msg/msg/CommandResult
command arm_control @requires_interface("robonix/hal/arm/get_params") @desc("Arm control command.") {
    input cmd ArmCommand @desc("Arm command payload");
    result status CommandResult @desc("Success/failure");
    version 1.0;
}

// pose.ridl
namespace robonix/hal/localization
import geometry_msgs/msg/PoseStamped
stream pose @desc("Localization pose stream.") {
    output pose PoseStamped @frame("map") @desc("Pose in map frame");
    version 1.0;
}
```

---

## 6. 语法示例

```ridl
namespace robonix/system/debug
import std_msgs/msg/String
query ping @desc("Echo test; verify connectivity.") {
    request data String @desc("Arbitrary string to echo");
    response data String @desc("Same string echoed back");
    version 1.0;
}
```

```ridl
namespace robonix/system/map
import std_msgs/msg/String
import robonix_msg/msg/Object
query semantic_query @desc("Query objects in semantic map by filter.") {
    request filter String @desc("JSON filter, e.g. {\"room\":\"kitchen\"}");
    response objects Object[] @desc("Objects with id, label, pose");
    version 1.0;
}
```

```ridl
namespace robonix/hal/base
import geometry_msgs/msg/Twist
import nav_msgs/msg/Odometry
import robonix_msg/msg/CommandResult
command motion_cmd @desc("Direct velocity control.") {
    input cmd Twist @desc("Linear/angular velocities");
    output odom Odometry @desc("Current odometry");
    result status CommandResult @desc("Success/failure");
    version 1.0;
}
```

```ridl
namespace robonix/hal/localization
import geometry_msgs/msg/PoseWithCovarianceStamped
stream pose @desc("Pose with covariance stream.") {
    output state PoseWithCovarianceStamped @desc("Pose + covariance matrix");
    version 1.0;
}
```

---

## 7. 运行时 channel

**channel** 是某个 interface 在运行时的具体通信端点（例如对应的 ROS 2 topic/service/action 名称）。它由 robonix-server（meta）在运行时分配；RIDL 与 manifest **不承载** channel 的静态命名，从而避免把部署拓扑硬编码进接口描述。

当 node 作为服务方提供某个 interface 时，会通过 meta API **注册**并获得 channel；当调用方要访问该 interface 时，会携带 `(node_id, interface_id)` 让 meta **解析**出 channel，再发起实际的 ROS 通信。channel 的分配策略与生命周期由运行时与生成代码共同完成。

---

## 8. 接口版本与演进

每个接口声明 `version`（如 `1.0`）。不兼容变更应递增版本或新命名空间。

---

## 9. 与 RFC002 的边界

**RFC001** 描述接口契约：RIDL 语法、**通信语义**（stream/command/query）、类型与 channel 的概念模型。**RFC002** 描述交付与进程模型：package 目录、`robonix_manifest.yaml`、node、entry，以及 `rbnx build`/`start` 等工具职责。二者在运行时的衔接点是：node 进程使用 manifest 中的 **node id** 向 meta 注册或解析 channel，调用方用 `(node_id, interface_id)` 定位通信端点。

## 10. 硬件抽象原语域（prm）

`robonix/prm/*` 命名空间下的接口构成 Robonix 的**抽象硬件原语**（文档中常简称**原语**）：用于描述相机、底盘、机械臂等能力边界。每个具体接口仍会用 §4 中的某一种 **通信语义**（stream、command 或 query）来声明。厂商通常只需实现与本机硬件能力匹配的**子集**；同一领域内设备形态差异很大（例如纯深度相机、RGB 相机、红外相机），因此也不存在“所有设备都必须实现”的统一最小契约集合。调用方应通过运行时能力查询或约定，确认目标 node 是否提供所需 interface。

### 10.1 ROS IDL 无法做到的部分

| 能力 | ROS IDL | RIDL |
|------|---------|------|
| 消息/服务类型定义 | ✓ | 引用 ROS msg/srv |
| 通信语义（stream/command/query） | ✗ 仅 topic/service/action | ✓ 传输无关抽象 |
| 接口身份（命名空间 + 名称） | ✗ 依赖 topic 名约定 | ✓ 显式 namespace |
| 接口间依赖（requires_interface） | ✗ 无 | ✓ `@requires_interface` |
| 参考系声明（frame） | ✗ 无 | ✓ `@frame` |
| 可中断性（interruptible） | ✗ action 有 cancel 但 IDL 不声明 | ✓ `@interruptible` |
| 运行时 channel 分配 | ✗ 固定 topic 名 | ✓ 由 meta 分配 |

ROS IDL（.msg/.srv/.action）只描述**载荷结构**，不描述**接口契约**或 **stream/command/query 这类通信语义**。RIDL 在接口描述层补充这些能力。
