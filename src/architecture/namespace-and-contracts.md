# 命名空间与能力约定

[toc]

Robonix 使用 **contract** 定义标准接口，使用 **Capability** 表示某个 provider 对该接口的具体实现。contract 说明数据形状和交互方式；Capability 再绑定 provider、transport 和 endpoint。

## Contract ID 是接口名，不是设备 ID

以下 ID 表示“相机 RGB 数据”这一标准接口：

```text
robonix/primitive/camera/rgb
```

它不表示前置相机、机械臂相机或某个进程。多个 provider 可以同时实现同一 contract：

```text
front_camera   + robonix/primitive/camera/rgb + ROS2
arm_camera     + robonix/primitive/camera/rgb + ROS2
rear_camera    + robonix/primitive/camera/rgb + gRPC
```

消费者先选择 provider ID，再选择 contract 和 transport。Atlas 以 `(provider_id, contract_id, transport)` 唯一定位一条运行时 Capability。

## 命名空间

provider 注册时声明主 `namespace`。它用于分类、查询和配置诊断，不是授权或隔离边界。

| Contract 前缀 | 内容 |
|---|---|
| `robonix/primitive/*` | 硬件与设备原语，例如 chassis、camera、lidar、arm |
| `robonix/service/*` | 可复用算法与服务，例如 mapping、navigation、speech |
| `robonix/skill/*` | 面向任务的可复用技能 |
| `robonix/system/*` | Robonix 系统接口，例如 Executor、Pilot、Scene、Soma |

这四类是 contract 命名空间。Atlas 的 provider kind 仍只有 Primitive、Service 和 Skill；`system` 不是第四种 provider kind。

普通 contract 应位于 provider 主 namespace 下。若不一致，robonix-api 和 Atlas 会记录 warning，并将 Capability 标记为 `namespace_mismatch`，但不会阻止注册、启动或调用。确实需要跨命名空间复用的 contract 可以在 descriptor 中设置：

```toml
cross_namespace = true
```

## Contract descriptor

标准 descriptor 位于 Robonix 源码树的 `capabilities/`。例如：

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

descriptor 的关键字段为：

- `id`：稳定的标准接口名。
- `version`：contract 版本。
- `kind`：预期的 provider kind。
- `idl`：`.msg` 或 `.srv` 载荷定义。
- `description`：所有 provider 实例共享的接口说明。
- `cross_namespace`：可选；允许跨主 namespace 实现而不产生诊断。
- `[mode].type`：交互形态。

支持的 mode：

| Mode | IDL | 含义 |
|---|---|---|
| `rpc` | `.srv` | 一元请求与响应 |
| `rpc_server_stream` | `.srv` | 一次请求，服务端流式响应 |
| `rpc_client_stream` | `.srv` | 客户端流式请求，一次响应 |
| `rpc_bidirectional_stream` | `.srv` | 双向流 |
| `topic_out` | `.msg` | provider 发布消息 |
| `topic_in` | `.msg` | provider 接收消息 |

contract descriptor 不绑定传输。同一 contract 可以由不同 provider 通过 ROS 2、gRPC 或 MCP 实现。

## Package-local contract

官方 contract 尚未覆盖的实验接口可以随 package 提供：

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

package manifest 可用 `path:` 明确关联 package-local descriptor：

```yaml
capabilities:
  - name: robonix/skill/inspection/inspect
    path: capabilities/inspect.v1.toml
```

`rbnx codegen` 与 Atlas 都会合并 Robonix 全局 `capabilities/` 和 package 自带的 `capabilities/`。同 ID 的 package-local descriptor 覆盖全局 descriptor，适合实验；进入公共接口后应迁移到 Robonix 主仓库并更新使用方。

## Codegen 输出

在 package 根目录运行：

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

`rbnx start` 会把 package 根、`proto_gen/` 和 `robonix_mcp_types/` 加入 `PYTHONPATH`。如果存在 `<package>/rbnx-build/ws/install/setup.bash`，启动前还会 source 该 colcon overlay。

`ros2_idl/` 只是 colcon workspace 源码。使用 ROS 2 类型前，package 的 `build.sh` 仍需在目标 ROS 2 环境中执行 `colcon build`，并在运行时 source 对应的 `install/setup.bash`。

## Capability 绑定与通道

provider 启动时调用 `DeclareCapability`，将 contract 绑定到 transport 和候选 endpoint。消费者使用前调用：

```text
ConnectCapability(consumer_id, provider_id, contract_id, transport)
```

Atlas 返回最终 endpoint 和传输参数，并记录 channel。消费者随后直接连接 provider。`Query` 只返回发现元数据，不公开 endpoint，因此不能用 Query 绕过 channel 建立新连接。

使用以下命令检查结果：

```bash
rbnx contracts
rbnx caps -v
rbnx channels
```

contract 解决接口兼容问题；provider ID 解决实例选择问题；namespace 解决分类与诊断问题。三者不要混用。
