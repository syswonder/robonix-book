# 命名空间与能力约定


Robonix 使用**能力约定（Contract）**定义标准接口，使用**能力（Capability）**表示某个提供方（Provider）对该接口的具体实现。能力约定说明数据结构和交互方式；能力再绑定提供方、传输方式（Transport）和访问地址（Endpoint）。

## 能力约定 ID 是接口名，不是设备 ID

以下 ID 表示“相机 RGB 数据”这一标准接口：

```text
robonix/primitive/camera/rgb
```

它不表示前置相机、机械臂相机或某个进程。多个提供方可以同时实现同一能力约定：

```text
front_camera   + robonix/primitive/camera/rgb + ROS2
arm_camera     + robonix/primitive/camera/rgb + ROS2
rear_camera    + robonix/primitive/camera/rgb + gRPC
```

消费者先选择提供方 ID，再选择能力约定和传输方式。Atlas 以 `(provider_id, contract_id, transport)` 唯一定位一条运行时能力。

## 命名空间

提供方注册时声明主 `namespace`。它用于分类、查询和配置诊断，不是授权或隔离边界。

| 能力约定前缀 | 内容 |
|---|---|
| `robonix/primitive/*` | 硬件与设备原语，例如底盘、相机、激光雷达和机械臂 |
| `robonix/service/*` | 可复用算法与服务，例如建图、导航和语音 |
| `robonix/skill/*` | 面向任务的可复用技能 |
| `robonix/system/*` | Robonix 系统接口，例如执行器、规划器、场景服务和本体服务 |

这四类是能力约定命名空间。Atlas 的提供方类别仍只有原语、服务和技能；`system` 不是第四种提供方类别。

普通能力约定应位于提供方主命名空间下。若不一致，Robonix API 和 Atlas 会记录警告，并将能力标记为 `namespace_mismatch`，但不会阻止注册、启动或调用。确实需要跨命名空间复用的能力约定可以在描述文件中设置：

```toml
cross_namespace = true
```

## 能力约定描述文件

标准描述文件位于 Robonix 源码树的 `capabilities/`。例如：

```toml
# capabilities/primitive/chassis/move.v1.toml
[contract]
id = "robonix/primitive/chassis/move"
version = "1"
kind = "primitive"
idl = "chassis/srv/ExecuteMoveCommand.srv"
description = "Low-level bounded chassis motion command without global path planning."

[mode]
type = "rpc"
```

`idl` 相对于 `capabilities/lib/` 解析。因此上例对应：

```text
capabilities/lib/chassis/srv/ExecuteMoveCommand.srv
```

描述文件的关键字段为：

- `id`：稳定的标准接口名。
- `version`：能力约定版本。
- `kind`：预期的提供方类别。
- `idl`：`.msg` 或 `.srv` 载荷定义。
- `description`：所有提供方实例共享的接口说明。
- `cross_namespace`：可选；允许跨主命名空间实现而不产生诊断。
- `[mode].type`：交互形态。

支持的模式：

| 模式 | IDL | 含义 |
|---|---|---|
| `rpc` | `.srv` | 一元请求与响应 |
| `rpc_server_stream` | `.srv` | 一次请求，服务端流式响应 |
| `rpc_client_stream` | `.srv` | 客户端流式请求，一次响应 |
| `rpc_bidirectional_stream` | `.srv` | 双向流 |
| `topic_out` | `.msg` | 提供方发布消息 |
| `topic_in` | `.msg` | 提供方接收消息 |

能力约定描述文件不绑定传输方式。同一能力约定可以由不同提供方通过 ROS 2、gRPC 或 MCP 实现。

## 软件包内能力约定

官方能力约定尚未覆盖的实验接口可以随软件包提供：

```text
my_skill/
├── capabilities/
│   ├── inspect.v1.toml
│   └── lib/
│       └── inspect/
│           └── srv/
│               └── Inspect.srv
├── package_manifest.yaml
└── scripts/
```

软件包清单可用 `path:` 明确关联软件包内的描述文件：

```yaml
capabilities:
  - name: robonix/skill/inspection/inspect
    path: capabilities/inspect.v1.toml
```

`rbnx codegen` 与 Atlas 都会合并 Robonix 全局 `capabilities/` 和软件包自带的 `capabilities/`。同 ID 的软件包内描述文件覆盖全局描述文件，适合实验；进入公共接口后应迁移到 Robonix 主仓库并更新使用方。

## 代码生成产物

在软件包根目录运行：

```bash
rbnx codegen --mcp --ros2
```

默认产物为：

```text
<package>/
└── rbnx-build/
    ├── proto-staging/                  # 临时 proto 输入
    └── codegen/
        ├── proto_gen/                  # Python protobuf/gRPC stubs
        ├── robonix_mcp_types/          # --mcp 生成的 MCP 类型
        └── ros2_idl/                   # --ros2 生成的 ROS 2 overlay 源码
```

`rbnx start` 会把软件包根目录、`proto_gen/` 和 `robonix_mcp_types/` 加入 `PYTHONPATH`。如果存在 `<package>/rbnx-build/ws/install/setup.bash`，启动前还会加载该 colcon 覆盖层。

`ros2_idl/` 只是 colcon 工作区源码。使用 ROS 2 类型前，软件包的 `build.sh` 仍需在目标 ROS 2 环境中执行 `colcon build`。`rbnx start` 只会自动加载 `<package>/rbnx-build/ws/install/setup.bash`；如果包将代码生成的接口构建到其他覆盖层，必须在自己的 `start` 命令中先加载目标平台的系统 ROS 2，再加载该覆盖层。

## 能力绑定与通道

提供方启动时调用 `DeclareCapability`，将能力约定绑定到传输方式和候选访问地址。消费者使用前调用：

```text
ConnectCapability(consumer_id, provider_id, contract_id, transport)
```

Atlas 返回最终访问地址和传输参数，并记录通道。消费者随后直接连接提供方。`Query` 只返回发现元数据，不公开访问地址，因此不能用 `Query` 绕过通道建立新连接。

使用以下命令检查结果：

```bash
rbnx contracts
rbnx caps -v
rbnx channels
```

能力约定解决接口兼容问题；提供方 ID 解决实例选择问题；命名空间解决分类与诊断问题。三者不要混用。
