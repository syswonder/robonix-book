# RFC001: Robonix Interface Definition Language (RIDL)

<!-- toc -->

| 版本 | 日期 | 作者 |
|------|------|------|
| Internet Draft | 2026-03-10 | 韩喻泷 |

## 概念与语义

### 概述

Robonix Interface Definition Language (RIDL) 是 Robonix 用于描述能力接口的接口定义语言，面向 hardware primitives、算法服务与 skills 等各类能力，提供与具体实现细节尽量解耦的接口描述。RIDL 描述接口名称、版本、通信模式、输入输出及其类型与依赖，但不在接口层写死 topic 名或底层 RMW 配置；在当前实现中，**所有接口均绑定到 ROS2 的 topic / service / action 等原生机制**，代码生成直接面向 ROS 接口，runtime 在此基础上附加注册、发现与安全约束。

### 通信语义：Embodied AI OS 需要什么

Robonix 的通信模型从 Robonix 逻辑空间推导而来。

感知空间对应 Stream：高频、单向、带时间戳的数据流，可丢弃、可只消费一次。动作空间对应 Command：强调安全与严格所有权，支持超时与 watchdog，带时间戳的控制语义。感知 + 认知 + 动作对应 Query：请求–响应模式，多为低频。Event 对应 OS 意义上的“中断线”：边沿触发、异步信号，表示“某事发生”，用于唤醒或触发响应，非连续数据流。RIDL 被 HAL（Primitive）节点、LLM 节点、服务节点等共同使用；所有节点仅通过 stream / command / query / event 交互。**Skill 与 HAL、Service 一致**：每个 skill 在 skill namespace 下定义自己的 RIDL（如 `robonix/skill/pick_place`、`robonix/skill/navigate`），包含该技能所需的 stream、command、query、event，不再共用“统一 skill RIDL”。一个 HAL 节点、或任意一个 service、一个 skill 实现，都可能由**多个** stream、event、command、query 共同构成；RIDL 文件里可以包含多个接口定义，ridlc 根据整份（或若干份）RIDL 及其中 import、各 stream/command/event/query 定义，生成对应代码，通信细节隐藏，底层传输可替换（如 Zenoh、gRPC）。

## 设计目标

### 通信实现无关

RIDL 必须独立于任何具体通信实现。接口定义中不出现 topic 名、RPC endpoint、HTTP path、DDS 配置等；这些由后续 binding 或部署配置提供。更重要的是，拿到 RIDL 生成代码并补全逻辑的开发者也不应接触 topic、endpoint 或任何 channel 名——不硬编码、不配置映射。Channel 由 robonix server（runtime）在能力注册时自动分配；生成代码在运行时从 runtime 读入本次分配的 channel 再创建订阅/发布或连接，对用户完全隐藏。

### 统一接口语义

RIDL 为系统定义统一的接口语义。当前支持的通信模式及其含义如下。

| 模式 | 含义 |
|------|------|
| stream | 单向、连续数据流（感知等） |
| command | 带安全与所有权语义的控制命令（动作等） |
| query | 请求–响应（服务、LLM 等） |
| event | OS 中断线式：边沿触发、异步信号，用于“有事发生”时的唤醒与响应，非连续数据流 |

### 统一类型系统

Robonix 统一使用 ROS IDL message definitions 作为唯一数据类型来源；RIDL 不引入新的数据结构语言。所有在 RIDL 中引用的类型均须来自 ROS message 定义（如 `geometry_msgs/msg/Twist`、`nav_msgs/msg/Odometry`、`sensor_msgs/msg/Image`、`robonix_msgs/msg/Object` 等）。

### 显式依赖与语法简洁

RIDL 文件必须显式声明所依赖的消息类型，编译器不得假设任何类型默认可见。语法遵循最小化原则：仅支持 namespace 声明、import 声明、接口定义与类型引用；不支持数据结构定义、泛型、表达式或控制语句。

### 类型系统

### Canonical 类型：ROS IDL

Robonix 以 ROS message definitions 为唯一 canonical 类型系统；RIDL 不允许定义新的持久数据结构，所有结构均来自 `.msg` 文件。当前实现中，RIDL 生成的代码全部直接使用 ROS 的 `.msg` 类型与 ROS2 提供的序列化/反序列化与通信栈；若未来需要对接其他协议，可在 **不改变 RIDL 类型体系** 的前提下，按需增加额外 binding。

### 类型引用与导入

类型引用采用 ROS 风格：`<package>/msg/<Type>`（如 `geometry_msgs/msg/Twist`、`nav_msgs/msg/Odometry`、`sensor_msgs/msg/Image`、`robonix_msgs/msg/Object`）。RIDL 支持单类型导入 `import geometry_msgs/msg/Twist` 与目录导入 `import geometry_msgs/msg/*`（导入该目录下全部 `.msg` 类型）。编译器根据引用查找对应 `.msg` 文件（例如 `robonix_msgs/msg/Object` 对应 `robonix_msgs/msg/Object.msg`），并应支持通过搜索路径解析类型。

## RIDL 语言规范

### RIDL 词法、语法与 Namespace 规范

### 文件与词法

- **文件扩展名**：`.ridl`。ridlc 只接受扩展名为 `.ridl` 的输入文件。
- **编码**：UTF-8。
- **空白**：空格、制表符、换行。连续空白与换行在语法上等价于单一空格（除分隔 token 外）。
- **注释**：`//` 至行末为行注释；`/*` `*/` 为块注释。注释不参与语法解析。
- **标识符**：字母、数字、下划线组成，首字符不得为数字。namespace 路径段名还允许 `/`（段内不得有空格）。类型引用中的 `<package>`、`<Type>` 同标识符规则，但 `<package>` 可含多段，段间用 `/` 分隔。
- **关键字**：`namespace`、`import`、`stream`、`command`、`query`、`event`、`input`、`output`、`result`、`request`、`response`、`payload`、`safety`、`version`、`RO`、`RW`。关键字不可作标识符。
- **字面量**：safety 块内类型使用内置名 `float`、`double`、`int32`、`int64`、`uint32`、`uint64`、`bool`、`string`，或引用 ROS 类型 `<package>/msg/<Type>`。

### 语法规则（产生式）

以下用 BNF 风格描述；`*` 表示零次或多次，`?` 表示可选，`|` 表示或。

```
file             := namespace_decl import_decl* interface_def*
namespace_decl   := "namespace" namespace_path "\n"
namespace_path   := segment ("/" segment)*
segment          := identifier

import_decl     := "import" type_ref "\n"
                   | "import" type_ref "/" "*" "\n"
type_ref        := msg_package "/" "msg" "/" type_name
msg_package     := identifier
type_name       := identifier

interface_def  := stream_def | command_def | query_def | event_def

stream_def     := "stream" identifier "{" stream_body "}"
stream_body    := output_list version_decl?
output_list    := ("output" identifier type_ref ";")+

command_def    := "command" identifier "{" command_body "}"
command_body   := input_decl output_decl? result_decl? version_decl? safety_block?
input_decl     := "input" identifier type_ref ";"
output_decl    := "output" identifier type_ref ";"
result_decl    := "result" identifier type_ref ";"
version_decl   := "version" version_string ";"
version_string := digit+ "." digit+ "." digit+
safety_block   := "safety" "{" safety_item+ "}"
safety_item    := identifier safety_type "RO" ";" | identifier safety_type "RW" ";"
safety_type    := "float" | "double" | "int32" | "int64" | "uint32" | "uint64" | "bool" | "string" | type_ref

query_def      := "query" identifier "{" query_body "}"
query_body     := request_decl response_decl version_decl?
request_decl   := "request" identifier type_ref ";"
response_decl  := "response" identifier type_ref ";"

event_def      := "event" identifier "{" event_body "}"
event_body     := payload_decl version_decl?
payload_decl   := "payload" identifier type_ref ";"
```

- `version_decl` 可出现在 command、query、stream、event 的 body 中；若出现，格式为 `version` 后接语义版本字符串（`major.minor.patch`）与结束符。

- 同一文件中 `namespace` 仅允许出现一次；同一 `namespace` 下 `stream`/`command`/`query`/`event` 的 identifier 不得重复。
- 语句以**换行**或**分号**结束；同一行多条语句需用分号分隔。本 RFC 示例中为便于阅读多采用换行结束。
- `safety` 块内每个 identifier 为安全参数名，不得重复；`RO`/`RW` 必须为大写。
- 可选字段：command 的 `output`、`result`、`safety` 可省略；若省略 `safety`，则该 command 无安全参数，manifest 中不必提供该 command 的 safety 取值。

### Namespace 定义与规范

- **定义**：namespace 是 RIDL 接口的命名空间标识，由一条 `namespace` 声明给出，值为 **namespace_path**（若干 **segment** 用 `/` 连接）。注意：此处 namespace 与下文“Robonix 软件包（package）”不同——namespace 仅用于 RIDL 接口组织，package 指可部署的软件包目录结构。
- **命名规范**：
  - 第一段建议为 `robonix`，表示 Robonix 官方或兼容命名空间；第三方可使用自己的前缀（如组织名）。
  - 第二段建议表示能力类别：`prm`（原语）、`srv`（服务）、`skill`（技能；与 prm、srv 一致，每个 skill 在 `robonix/skill/<name>` 下定义独立 RIDL，如 `robonix/skill/pick_place`）、`msg` 保留给 ROS 类型包名（避免与 RIDL namespace 混淆）。
  - 后续段为具体子模块，如 `robonix/prm/base`、`robonix/srv/planning`、`robonix/skill/pick_place`。
- **保留名**：第一段不得为 `std`、`ros`、`geometry_msgs` 等已被 ROS 或系统占用的类型包名；RIDL namespace 与 ROS 类型包（`<package>/msg/<Type>` 中的 package）分属不同命名体系，但为避免歧义，RIDL namespace 第一段不得与常见 ROS 包名冲突。
- **目录布局约定**：每个 `.ridl` 文件必须包含且仅包含一条 `namespace` 声明。**HAL 的 RIDL 一般存放在平台或规范仓库**（类比 Android 中 HAL 的 AIDL 在 AOSP 中，由平台稳定提供）；**Service、Skill 的 RIDL 可由开发者自行编写**（类比 Android 中 app/server 自己写 AIDL），可随实现包提供、或放在规范/私有仓库，通过 ridlc 的 `-I` 与 manifest 的 `ridl_ref` 引用。实现包通过 manifest 的 `ridl_ref` 引用本能力实现的接口；ridlc 的输入来自平台/规范仓库或构建时指定的路径；生成代码的输出路径按 namespace_path 组织。
- **版本**：接口可在 command/query/stream/event 后通过 `version` 声明语义版本（格式 `major.minor.patch`）；若省略则无显式版本。版本信息可被 codegen 或 runtime 使用，不改变 RIDL 语法解析。

### 与类型引用的区分

- **RIDL namespace_path**：多段，用 `/` 分隔，用于组织接口（如 `robonix/srv/planning`）。
- **类型引用**：`<package>/msg/<Type>`，其中 `<package>` 为 ROS 消息包名（通常单段，如 `geometry_msgs`），`<Type>` 为消息类型名。RIDL 不定义 .msg 内容，仅引用；类型解析依赖 `-I` 与 .msg 文件存在性。

### 通信模式与语法

### Stream

Stream 表示单向、连续数据流（如相机、传感器）。每条流需要显式命名并指定类型，以便区分多路输出与 binding 生成。

```c
stream camera_rgb {
    output frame sensor_msgs/msg/Image
}
```

| 字段 | 含义 |
|------|------|
| output \<name\> \<Type\> | 输出流名称及消息类型，可多条 |

### Command

Command 表示带安全与所有权语义的控制命令（超时、watchdog、时间戳由 binding/runtime 保证）。输入为命令载荷，可选地包含持续反馈与最终结果。Command 必须有**安全保障**：RIDL 为每个 command 预留**安全与约束声明**位置，开发者声明实例时必须在 manifest 中填写对应的限制与参数；RO/RW 由 RIDL 定义，代码生成会据此自动产生校验与可调接口。

**安全与约束在 RIDL 中的声明**

在 command 内可声明 `safety` 块，列出本命令的安全相关参数：名称、类型（或引用 .msg）、以及 **RO**（只读，仅能在声明实例时在 manifest 中设值）或 **RW**（可读写，运行时可调整；codegen 通常为之生成 query 形式接口，供授权方查询或修改）。例如：控制限制（速度上下界、加速度限制）、超时、使能开关等。未在 RIDL 中声明的安全项，实例不必填写；凡在 RIDL 中声明的，实例声明时必须提供对应值（RO 为固定配置，RW 可提供默认值，后续通过生成代码暴露的 query 调整）。

```c
command move {
    input cmd geometry_msgs/msg/Twist
    output feedback nav_msgs/msg/Odometry   // 可选
    result status robonix_msgs/msg/CommandResult
    safety {
        max_linear_vel   float  RW   // 运行时可调，codegen 生成 query
        max_angular_vel   float  RW
        timeout_ms       int32  RO   // 仅 manifest 设一次
        enable_guard     bool   RO
    }
}
```

| 字段 | 含义 |
|------|------|
| input \<name\> \<Type\> | 命令输入名称及类型 |
| output \<name\> \<Type\> | 执行过程反馈（可选） |
| result \<name\> \<Type\> | 执行结果类型 |
| safety { ... } | 安全/约束参数：名称、类型、RO 或 RW；实例声明时必填，codegen 生成校验与（对 RW）可调接口 |

### Skill 与 HAL、Service 的一致性

Skill 与 HAL、Service 采用同一套 RIDL 与 manifest 约定：**每个 skill 在 skill namespace 下定义自己的 RIDL**（如 `robonix/skill/pick_place`、`robonix/skill/navigate`），包含该技能所需的 stream、command、query、event，不再共用“统一 skill RIDL”或单一 `invoke` command。实现包在 manifest 的 `ridl_ref` 中引用该 skill 的 namespace（如 `robonix/skill/pick_place`）；runtime 通过 capability `id` 与 `ridl_ref` 识别并发现能力，与 HAL、Service 一致。

### Query

Query 表示请求–响应接口（服务、LLM 等），多为低频。

```c
query plan_path {
    request req robonix_planning_msgs/msg/PlanPathRequest
    response path nav_msgs/msg/Path
}
```

| 字段 | 含义 |
|------|------|
| request \<name\> \<Type\> | 请求参数名称及类型 |
| response \<name\> \<Type\> | 响应类型 |

### Event

Event 表示 OS “中断线”式的异步信号：边沿触发、不承载连续数据流，语义是“某一刻发生了某事”，用于唤醒或触发响应（例如任务完成、异常、硬件就绪）。与 stream 的区别在于，stream 是持续、可多帧的流式数据，event 是离散的“发生了一次”的通知；与 command 的区别在于，event 无请求–响应或所有权语义，只是单向信号。payload 可携带少量上下文（如事件类型、时间戳、简单状态码），供接收方决定是否进一步 query 或拉取 stream。

```c
event task_event {
    payload ev robonix_msgs/msg/TaskEvent
}
```

| 字段 | 含义 |
|------|------|
| payload \<name\> \<Type\> | 事件数据名称及类型 |

### 示例

以下为符合本规范词法与语法的 RIDL 示例。

Command：

```c
namespace robonix/prm/base

import geometry_msgs/msg/Twist
import nav_msgs/msg/Odometry
import robonix_msgs/msg/CommandResult

command move {
    input cmd geometry_msgs/msg/Twist
    output feedback nav_msgs/msg/Odometry
    result status robonix_msgs/msg/CommandResult
    safety {
        max_linear_vel   float  RW
        max_angular_vel  float  RW
        timeout_ms       int32  RO
        enable_guard     bool   RO
    }
}
```

Stream：

```c
namespace robonix/prm/localization

import nav_msgs/msg/Odometry

stream odom {
    output pose nav_msgs/msg/Odometry
}
```

Query：

```c
namespace robonix/srv/planning

import robonix_planning_msgs/msg/PlanPathRequest
import nav_msgs/msg/Path

query plan_path {
    request req robonix_planning_msgs/msg/PlanPathRequest
    response path nav_msgs/msg/Path
}
```

## ridlc 与 Binding 约定

### 编译模型 与 ridlc 规范

RIDL 编译器 `ridlc` 负责解析 RIDL 文件与消息类型依赖、构建接口 AST，并生成面向 ROS2 的默认 binding 代码：stream/command/event 映射到 ROS topic / action 等，query 映射到 ROS service 或等价机制。以下为命令行、输入输出与行为的完整约定。

### 命令行用法

```
ridlc [选项] <.ridl 文件...>
```

### 选项（必需与可选）

| 选项 | 短选项 | 类型 | 必需 | 说明 |
|------|--------|------|------|------|
| `--include` | `-I` | path | 是（至少一次） | 类型搜索路径：用于解析 RIDL 中 `import` 及类型引用时查找 `.msg` 文件。可多次指定；顺序为搜索顺序。 |
| `--out` | `-o` | path | 是 | 生成代码的输出目录。ridlc 在该目录下按 RIDL namespace 路径创建子目录并写入生成文件。 |
| `--workdir` | `-w` | path | 否 | 工作目录：用于解析相对路径的 .ridl 及相对 `-I` 路径。默认当前工作目录。 |
| `--lang` |  | string | 否 | 生成目标语言：`cpp`（默认）、`python`。决定生成代码的语言。 |
| `--version` | `-V` |  | 否 | 打印 ridlc 版本并退出。 |
| `--help` | `-h` |  | 否 | 打印用法并退出。 |

未指定 `--out` 或至少一次 `-I` 时，ridlc 必须报错并返回非零退出码。

### 输入

- 位置参数：一个或多个 `.ridl` 文件路径（相对 `--workdir` 或绝对）。ridlc 解析所有给定文件，合并为同一 AST（按 namespace 去重与合并），再生成代码。
- 类型解析：RIDL 中出现的所有类型引用（`<package>/msg/<Type>`）必须在 `-I` 指定的搜索路径下找到对应 `<package>/msg/<Type>.msg`，否则报错并列出缺失类型。

### 输出

- 在 `--out` 指定的输出目录下，按每个 RIDL 文件中出现的 **namespace** 路径创建子目录（将 namespace 的 `/` 转为 OS 路径分隔符），例如 namespace `robonix/prm/base` 对应子目录 `robonix/prm/base/`。
- 每个 stream/command/event/query 接口生成对应源文件与头文件（或目标语言等价物）；具体文件名与内容由实现约定，但必须包含：stream/command/event 的 ROS 语义封装（pub/sub、action 等）、query 的 ROS service 封装，以及若 command 带 `safety` 则校验逻辑与 RW 项的 get/set 接口（以 query 形式或等价形式）。
- 生成的代码不包含任何硬编码的 topic 名或 service 名等 channel 标识；这些名称可以由 runtime 或部署配置注入，也可以按项目约定静态配置，本规范不强制动态分配或跨中间件切换。

### 行为

1. 解析阶段：按顺序解析每个 .ridl 文件，收集 namespace、import、stream/command/event/query 及 safety 块；类型引用通过 `-I` 解析 .msg。
2. 校验阶段：同一 namespace 内接口名不重名；safety 块内参数名不重复；RO/RW 仅允许为 `RO` 或 `RW`；类型必须存在且为 .msg 定义。
3. 生成阶段：按“Binding 与默认传输约定”一节，stream/command/event/query 生成 ROS 语义封装；若 command 含 safety，生成校验与 RW 可调接口。
4. 退出码：成功为 0；解析错误、类型缺失、校验失败或写入失败为非 0。

### Binding 与默认 ROS 映射约定

当前实现中，RIDL 仅面向 **ROS2** 进行代码生成与运行时绑定；即所有通信模式最终都落在 ROS2 提供的 topic / service / action 等原生机制之上。若未来扩展到其他中间件，将通过新增 binding 的方式实现，本 RFC 不做规范。

Robonix 约定一套**默认映射**，便于 ridlc 生成可编译、可运行的 ROS 节点代码：

- **stream**：映射为 ROS topic（Publisher/Subscription），用于高频流式数据。
- **command**：映射为 ROS action（推荐）或 service 组合，用于带反馈与结果的控制命令。
- **event**：映射为 ROS topic 或等价事件通道（如 latched topic），用于中断式通知。
- **query**：映射为 ROS service，用于请求–响应接口。

因此，对一个由若干 stream、command、event、query 构成的节点，ridlc 根据 RIDL 文件中的 import 以及各 stream/command/event/query 定义，生成**一份 ROS 节点代码**：包含基于 ROS 的 pub/sub、service 与 action 封装；通信细节（topic 名、service 名等）可由 runtime 或部署配置注入，也可以按项目约定静态配置。本规范不要求在运行时跨中间件动态切换，只要求接口语义与类型由 RIDL 统一描述。

| RIDL 模式 | 默认 ROS 映射 | 说明 |
|-----------|---------------|------|
| stream | topic | 流式数据 |
| command | action / service | 控制命令与反馈 |
| event | topic / 事件通道 | 中断线式事件信号 |
| query | service | 请求–响应 |

### robonix-server 行为

对于基于上面的代码生成器生成的代码实现的任何一个 robonix 进程节点（可能暴露若干 HAL、service 等服务接口），