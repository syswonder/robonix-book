# 快速开始

本节带领你从源码编译开始，理解 Robonix 的基本术语与通信模型，启动 robonix-server，并了解 ridlc 生成代码、自定义 package 与 `rbnx` 构建/运行的大致流程。更细的 package 目录、manifest 说明与多段完整示例见 [Package 开发指南](../chapter3-developer-guide/package-development.md)。

## 环境

- Ubuntu 22.04 + ROS 2 Humble；Rust；Python 3.10+
- `ros-humble-example-interfaces`、`ros-humble-test-msgs`；可选 `rmw_zenoh_cpp`

---

## 1. 源码与编译

### 1.1 克隆与依赖

```bash
git clone https://github.com/syswonder/robonix
cd robonix && git submodule update --init --recursive
cd rust
sudo apt install -y ros-humble-example-interfaces ros-humble-test-msgs ros-humble-rmw-zenoh-cpp python3-grpcio python3-protobuf
```

### 1.2 Makefile（推荐）

```bash
cd rust
make build          # ridlc、cli、server
make install        # ~/.cargo/bin
make clean
```

- Release：`make release` 或 `BUILD_MODE=release make install`
- 单项：`make build-ridlc` / `build-cli` / `build-server`；`make help`

首次 `make build` 会跑 ridlc、colcon `robonix_interfaces_ros2`。若需 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

**setuptools/packaging**：若 colcon 报 `canonicalize_version(... strip_trailing_zero)`，执行  
`pip3 install --user --upgrade 'packaging>=22.0' 'setuptools>=72'`（`start_server` 的 `setup_zenoh_rclrs_env` 会尝试处理）。

**不用 Make**：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
cargo build --manifest-path robonix-server/Cargo.toml --bin robonix-server --bin callquery
```

### 1.3 术语与模型

下面几个概念在后续章节会反复出现：

- **RIDL**：描述“有哪些接口、载荷长什么样、属于哪种通信语义”的契约；**不**在文件里写死 channel 名称，channel 由运行时分配（见 [RFC001 §7](../rfc/RFC001-RIDL.md)）。
- **channel**：某个 interface 在运行时的具体端点（对应 ROS 2 的 topic、service 或 action 名等），由 robonix-server 的 meta API 分配或解析。
- **原语（抽象硬件原语）**：Robonix 的**硬件抽象层**，在 RIDL 中主要体现为 `robonix/prm/*` 等命名空间下的接口（相机、底盘、机械臂等）；每种设备暴露哪些接口由厂商按能力实现子集。详见 [抽象硬件原语](../chapter3-developer-guide/primitives/index.md)。
- **通信语义**：RIDL 用 **stream**、**command**、**query** 描述单个接口的通信形态（传输无关的语义）；当前参考实现里常分别落到 ROS 2 的 topic、action、service，但**不要把这三者叫作「原语」**——它们是接口的通信类别，与「抽象到哪类硬件」是两个维度。
- **node**：向 meta 注册的逻辑实体，每个运行中的进程实例对应 manifest 里的一项 `nodes[]`。**interface** 则由 RIDL 定义；生成代码会提供 **`create_*_client` / `create_*_server` 等创建函数**（有些文档里也叫「工厂函数」，意思相同：传入 `runtime_client` 等参数，返回可用的 client/server 实例）。

不仅是 skill，任何运行在系统之上的**用户态进程**只要约定好 RIDL，都可以用同一套方式与 robonix-server、PRM、其他 package 通信。

| 示例 | 通信语义 | 说明 |
|------|----------|------|
| `stream_demo` | stream | 位姿流：`stream_server` 发布、`stream_client` 订阅，经 `robonix/prm/base/pose_cov`（底盘**原语** `pose_cov`） |
| `query_demo` | query | 语义查询：`semantic_server` 提供、`semantic_client` 调用，经 `robonix/system/map/semantic_query` |
| `skill_demo` | command | greet：`skill_server`/`skill_client`，经 package-local `skill_demo/skill/greet` |
| `python_ping_client` | query（client） | 调用 server 内置 ping，经 `robonix/system/debug/ping` |

各类硬件抽象（相机、底盘、机械臂等）的接口清单见 [抽象硬件原语](../chapter3-developer-guide/primitives/index.md)。

---

## 2. 启动 robonix-server

```bash
cd rust
./start_server
```

`start_server` 会先清理占用端口的旧进程，再执行 `setup_zenoh_rclrs_env`（source ROS、设置 RMW、必要时 colcon build 并 source overlay），最后启动 `robonix-server`。常用环境变量包括 `ROBONIX_META_GRPC_ADDR`（监听地址）、`ROBONIX_META_GRPC_ENDPOINT`（客户端连接地址）、`RMW_IMPLEMENTATION` 与 `RUST_LOG`。

确认启动成功时，日志中应出现 `meta-runtime: registered node 'robonix-server'` 以及 `ping query runtime ready`。你也可以执行 `./examples/callquery robonix-server robonix/system/debug/ping '"test"'`，期望返回 `pong:"test"`。

---

## 3. ridlc 与生成代码

大多数情况下你**不需要**手动运行 ridlc：构建 `robonix-server` 时，`build.rs` 会调用 ridlc 处理 `robonix-interfaces/ridl/` 下的定义并生成 Rust/Python 等代码。只有在维护自建接口树或做工具链实验时，才需要像下面这样单独调用 ridlc（需至少一个 `-I` 搜索路径）：

```bash
cargo run --manifest-path ridlc/Cargo.toml -- -I <include> -o <out> -i <ridl_dir_or_file> --lang python
```

| RIDL namespace | Python 包 |
|----------------|-----------|
| `robonix/system/debug` | `robonix.system.debug` |
| `robonix/prm/base` | `robonix.prm.base` |
| `robonix/system/map` | `robonix.system.map` |
| `robonix/system/skill` | `robonix.system.skill` |

`robonix/a/b/c` → `robonix.a.b.c`。

| 通信语义 | 生成文件 | 典型创建函数（`create_*`） |
|----------|----------|------------------------------|
| query | `{name}_query.py` | `create_{name}_client` / `create_{name}_server` |
| stream | `{name}_stream.py` | `create_{name}_publisher` / `create_{name}_subscriber` |
| command | `{name}_command.py` | `create_{name}_client` / `create_{name}_server` |

上表中的 **创建函数** 即 ridlc 按接口名生成、你在业务代码里直接调用的入口。更完整的生成规则与 Python 补全示例见 [ridlc 开发手册](../chapter3-developer-guide/ridlc.md)。

---

## 4. Package 与业务

自定义业务通常以 **package** 形式交付：根目录放置 `robonix_manifest.yaml` 描述包名与节点入口，源码里通过生成代码连接 meta 并注册/解析 channel。完整目录约定、manifest 字段表、以及 query/command/stream 的多段示例见 [Package 开发指南](../chapter3-developer-guide/package-development.md)。若你实现的是相机、机械臂或地图等对外能力，可再对照 [硬件/服务厂商接入指南](../chapter3-developer-guide/vendor-integration.md) 的按需子集原则。

生成代码已经处理了“如何向 robonix-server 注册、如何绑定 ROS 类型”等样板逻辑；你需要做的是在约定位置写入业务：**query** 使用 `server.start(handler)`；**command** 使用 `server.execute = fn`；**stream** 在发布侧调用 `publish(msg)`，在订阅侧 `subscriber.start(callback)`。具体签名与注意点见 [ridlc 手册 §5](../chapter3-developer-guide/ridlc.md#5-用户逻辑补全python)。

建议按下面顺序落地一个最小 package：

1. 在 RIDL 中定位要实现的接口，记下命名空间与接口名，从而确定 Python 模块与对应的 `create_*` 创建函数。
2. 新建 package 根目录：包含 `robonix_manifest.yaml`，以及名称与 `package.name` 一致的 Python 子包；`setup.py`/`setup.cfg` 由 `rbnx build` 生成，一般无需手写。
3. 填写 manifest 中的 `package` 元数据与 `nodes`（每项给出唯一 `id` 与 `entry: 模块:函数`）。
4. 在 entry 指向的模块里创建 gRPC channel、构造 `RobonixRuntimeStub`，再调用相应的 `create_*_server` 或 `create_*_client` 完成注册或调用。
5. 使用 `rbnx build -p <path>` 构建，确认 robonix-server 已运行后，再用 `rbnx start -p <package> -n <node_id>` 启动指定节点（`start` 会阻塞到进程结束）。

**最小 ping client** 的目录与代码可参考仓库内 `rust/examples/python_ping_client/`：构建后通过 `rbnx start` 拉起；若 manifest 中声明了多个 node，则需显式传入 `-n` 选择要启动的节点。

| 类型 | RIDL | 补全 |
|------|------|------|
| 机械臂 command | `.../arm/joint_trajectory` | execute |
| 语义 query | `.../map/semantic_query` | handler → objects |
| Skill command | package-local greet | execute |
| stream | `.../base/pose_cov` | publish / callback |

---

## 5. rbnx

```bash
rbnx build -p <path>              # 增量
rbnx build -p <path> --clean
rbnx build -g <name>              # 已 install 的包
rbnx start -p <package> -n <node_id>
```

`rbnx install --github <repo>` / `--path` → `~/.robonix/packages`；`list` / `info`。

示例：

```bash
cd rust
rbnx build -p python_ping_client
rbnx start -p python_ping_client   # 单 node 时常可省略 -n；多 node 须 -n
```

---

## 6. 路径与校验

| 路径 | 说明 |
|------|------|
| `rust/robonix-interfaces/ridl/` | RIDL |
| `rust/robonix-server/src/generated/` | Rust 生成 |
| `rust/robonix-server/target/rclrs_interfaces_ws/python_pkg/` | Python |
| `.../install/` | colcon install |

Zenoh：`echo $RMW_IMPLEMENTATION`。  
一键：`callquery ... ping` 或 `rbnx start -p python_ping_client`。
