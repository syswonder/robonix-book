# RFC 001: Robonix Interface Definition Language (RIDL)

| 版本 | 日期 | 作者 |
|------|------|------|
| 0.2 | 2026-03-13 | 韩喻泷 |

## 1. 目标与范围

RIDL 定义接口契约：接口身份（命名空间 + 名称）、通信语义（stream/command/query/event）、载荷类型、接口版本。RIDL 仅描述"有什么接口、长什么样"；不定义 package 构建、channel 名或部署拓扑，channel 由运行时分配（§6）。

**不在本 RFC 内**：Package 目录与 manifest（RFC002）、channel 分配实现细节、ridlc 代码生成与通信实现（见开发手册）。

---

## 2. 命名空间

格式：`robonix/{domain}/{subdomain}/...`，层级用 `/` 分隔，小写。

示例：`robonix/hal/base`、`robonix/system/debug/ping`、`robonix/system/execution/result_feedback`。

---

## 3. 通信原语

RIDL 四类原语均为**传输无关**的语义抽象，具体实现由 codegen/运行时选择。

| 原语 | 语义 | 典型用法 |
|------|------|----------|
| stream | 单向数据流 | 位姿/传感器、控制指令 |
| command | 长任务：输入 + 进度 + 结果 | 运动、skill 执行 |
| query | 同步请求-响应 | 状态查询、ping |
| event | 离散、一次触发、多订阅 | 结果反馈、告警、中断式通知 |

### 3.1 query

一对一、同步请求-响应。RIDL 结构：`request`、`response` 各声明字段，类型引用 message。

### 3.2 command

带输入、可选进度反馈、最终结果的长任务。RIDL 结构：`input`、`output`（进度）、`result`。

### 3.3 stream

**单向**：每个 stream 仅 `output` 或仅 `input`。以 provider 为视角：`output` = provider 发布；`input` = provider 订阅。

### 3.4 event

离散、一次触发、fire-and-forget、多订阅方。与 stream 区别：不强调持续流，强调"发生了某事"的即时通知。

---

## 4. 类型系统

- RIDL 不定义独立 IDL，引用兼容 ROS IDL 规范的类型（`package/msg/Name`、`package/srv/Name`）。
- **Wire 格式**：具体序列化与传输由实现决定。
- **数组**：`[]` 后缀，如 `robonix_msg/msg/Object[]`。
- **robonix_msg**：定义 Object、Relation、FrameMapping、SkillInstance、Dict 等语义结构，RIDL 中优先使用。

### 4.1 注解

格式 `@key` 或 `@key(param=value)`，出现在接口名后、`{` 前。

| 注解 | 参数 | 含义 |
|------|------|------|
| `@requires_interface` | `("ns/iface1", "ns/iface2")` | 实现本接口时必须同时实现所列接口 |
| `@frame` | `("map")` | 字段（位姿/点）的参考系，收发双方据此对齐坐标系 |

```ridl
command arm_control @requires_interface("robonix/hal/arm/get_params") {
    input cmd robonix_msgs/msg/ArmCommand;
    result status robonix_msgs/msg/CommandResult;
    version 1.0;
}

stream pose {
    output pose geometry_msgs/msg/PoseStamped @frame("map");
    version 1.0;
}
```

---

## 5. 语法示例

```ridl
namespace robonix/system/debug
import std_msgs/msg/String
query ping {
    request data std_msgs/msg/String;
    response data std_msgs/msg/String;
    version 1.0;
}
```

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

```ridl
namespace robonix/hal/localization
import geometry_msgs/msg/PoseWithCovarianceStamped
stream pose {
    output state geometry_msgs/msg/PoseWithCovarianceStamped;
    version 1.0;
}
```

```ridl
namespace robonix/system/execution
import robonix_msg/msg/Dict
event result_feedback {
    payload report robonix_msg/msg/Dict;
    version 1.0;
}
```

**语法要点**：每文件一个 `namespace`；`import` 多条；字段以 `;` 结尾；`version` 必填。

---

## 6. 运行时 channel

channel 为 interface 在运行时的具体通信端点，由 robonix-server 分配，RIDL 与 manifest 不写 channel 名。

- Server 注册：node 提供 interface 时向 meta API 注册，获得 channel。
- Client 解析：调用方指定 (node_id, interface_id)，通过 meta API 解析得到 channel 后发起通信。

channel 分配与生命周期由运行时与生成代码负责。

---

## 7. 接口版本与演进

每个接口声明 `version`（如 `1.0`）。不兼容变更应递增版本或新命名空间。

---

## 8. 与 RFC002 的边界

RFC001：接口契约（RIDL 语法、原语、类型、channel 概念）。RFC002：package、manifest、node、rbnx 构建与启停。二者通过 (node_id, interface_id) 在运行时衔接。

