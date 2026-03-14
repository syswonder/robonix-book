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
command 接口名 [注解] {
    input 字段名 类型;           // 至少 input 或 result 之一
    [output 字段名 类型;]        // 可选，进度反馈
    [result 字段名 类型;]
    version 版本号;
}

stream 接口名 [注解] {
    output 字段名 类型;          // 或 input，二选一
    version 版本号;
}

query 接口名 [注解] {
    request 字段名 类型;
    response 字段名 类型;
    version 版本号;
}
```

注解为 `@key` 或 `@key(param=value)`，写在接口名后、`{` 前，可多个。

### 3.2 文件结构

```
namespace robonix/域/子域
[import 类型路径;]...

接口定义（见上）
```

### 3.3 规则摘要

- **关键词**：`namespace`、`import`、`stream`、`command`、`query`、`input`、`output`、`result`、`request`、`response`、`version`。
- **文件规则**：一文件一接口；接口名必须与文件名一致（如 `depth.ridl` 定义 `stream depth`）。
- **命名**：namespace 小写、用 `/` 分隔；接口名、字段名小写或 snake_case；类型用 `package/msg/Name` 或 import 后短名。
- **字段**：以 `;` 结尾。

---

## 4. 通信原语

RIDL 四类原语均为**传输无关**的语义抽象，具体实现由 codegen/运行时选择。

| 原语 | 语义 | 典型用法 |
|------|------|----------|
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

格式 `@key` 或 `@key(param=value)`，出现在接口名后、`{` 前。

| 注解 | 参数 | 含义 |
|------|------|------|
| `@requires_interface` | `("ns/iface1", "ns/iface2")` | 实现本接口时必须同时实现所列接口 |
| `@frame` | `("map")` | 字段（位姿/点）的参考系，收发双方据此对齐坐标系 |
| `@interruptible` | 无 | command 支持运行时中断（如 cancel）；ROS action 有 cancel API 但 IDL 不声明 |

```ridl
command navigate @interruptible {
    input goal geometry_msgs/msg/PoseStamped;
    version 1.0;
}

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

## 6. 语法示例

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

---

## 7. 运行时 channel

channel 为 interface 在运行时的具体通信端点，由 robonix-server 分配，RIDL 与 manifest 不写 channel 名（§7）。

- Server 注册：node 提供 interface 时向 meta API 注册，获得 channel。
- Client 解析：调用方指定 (node_id, interface_id)，通过 meta API 解析得到 channel 后发起通信。

channel 分配与生命周期由运行时与生成代码负责。

---

## 8. 接口版本与演进

每个接口声明 `version`（如 `1.0`）。不兼容变更应递增版本或新命名空间。

---

## 9. 与 RFC002 的边界

RFC001：接口契约（RIDL 语法、原语、类型、channel 概念）。RFC002：package、manifest、node、rbnx 构建与启停。二者通过 (node_id, interface_id) 在运行时衔接。

---

## 10. 原语领域（prm）

`robonix/prm/*` 命名空间下的接口为**抽象硬件原语**。厂商按需实现子集，无需实现全部；同一领域内硬件形态各异（如纯深度相机、RGB 相机、红外相机），不存在统一最小契约。运行时通过 meta 解析 channel，调用方按需查询可用接口。

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

ROS IDL（.msg/.srv/.action）只描述**载荷结构**，不描述**接口契约**或**通信语义**。RIDL 在原语层补充这些能力。
