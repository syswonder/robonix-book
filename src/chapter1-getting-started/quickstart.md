# 快速开始

<!-- toc -->

本节说明：获取源码与编译、Robonix 术语与通信模型、如何启动 robonix-server、如何使用 ridlc 与生成代码、如何创建 package 并开发业务逻辑、如何使用 rbnx 编译/运行/停止 package。

## 环境要求

- Ubuntu 22.04 + ROS 2 Humble
- Rust (rustup)
- Python 3.10+
- `ros-humble-example-interfaces`、`ros-humble-test-msgs`（rclrs 依赖）
- `rmw_zenoh_cpp`（可选，默认使用 Zenoh 作为 RMW）

---

## 1. 获取源码与编译

### 1.1 获取源码

```bash
git clone https://github.com/syswonder/robonix
cd robonix
git submodule update --init --recursive
cd rust
sudo apt install -y ros-humble-example-interfaces ros-humble-test-msgs ros-humble-rmw-zenoh-cpp python3-grpcio python3-protobuf
```

### 1.2 编译与安装

推荐使用顶层 Makefile 一次性编译或安装 ridlc、rbnx、robonix-server：

```bash
cd rust

# 编译（默认 debug；会依次构建 ridlc、robonix-cli、robonix-server）
make build

# 安装到 ~/.cargo/bin（需已 PATH）
make install

# 清理各项目 target
make clean
```

- Release 编译：`make release` 再 `make build`，或安装时 `BUILD_MODE=release make install`。
- 单独编译某一项：`make build-ridlc`、`make build-cli`、`make build-server`；单独安装：`make install-ridlc`、`make install-cli`、`make install-server`。
- 查看所有目标：`make help`。

首次执行 `make build`（或直接构建 robonix-server）时会通过 ridlc 解析 RIDL、生成 Rust/Python/ROS 接口，并触发 colcon 构建 `robonix_interfaces_ros2`。若需 ROS 环境，请先：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

若 colcon 构建报错 `canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'`，是 setuptools 与 packaging 版本不兼容，请先执行：`pip3 install --user --upgrade 'packaging>=22.0' 'setuptools>=72'`（或使用 `sudo pip3 install --upgrade packaging setuptools`）。`./start_server` 使用的 `setup_zenoh_rclrs_env` 会在构建前自动尝试升级，并取消 `PYTHONNOUSERSITE` 以便使用用户已升级的 setuptools。

不用 Make 时，可手动编译 robonix-server 与 callquery（rbnx 需在 robonix-cli 下单独构建）：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
cargo build --manifest-path robonix-server/Cargo.toml --bin robonix-server --bin callquery
```

### 1.3 Robonix 术语与通信模型

- RIDL（Robonix Interface Definition Language）：接口定义语言，描述“谁提供什么、请求/响应类型”。不定义 channel 名或部署，channel 由运行时分配。
- channel（通道）：某一 interface 在运行时的具体通信端点（query/command 对应 ROS 2 service/action 名，stream 对应 topic 名）。由 robonix-server 在节点注册或 client 解析时分配；RIDL 与 manifest 中不写 channel 名。详见 [RFC001 §7](../rfc/RFC001-RIDL.md)。
- 通信原语：
  - stream：单向流，对应 ROS 2 topic（如位姿流）。
  - command：带结果的控制命令，对应 ROS 2 action（如运动指令、skill 执行）。
  - query：请求-响应，对应 ROS 2 service（如 ping、语义地图查询）。
- 节点（node）：向 robonix-server 注册的逻辑实体。每个节点可基于 RIDL 生成的 stub 提供若干 interface：
  - 被调用方：实现某 interface 的 server，通常为常驻进程（如 robonix-server 内的 ping 服务、语义地图 query server、机械臂 PRM、skill 节点）。
  - 调用方：使用某 interface 的 client，不要求常驻（可一次性调用后退出）。
- interface：由 RIDL 定义；生成代码提供 `create_*_client` / `create_*_server`（或 stream 的 publisher/subscriber、command 的 client/server），业务逻辑在 stub 中补全。

**通用性**：不仅是 skill，运行在系统之上的**任意进程**（不属于标准服务、原语实现、skill 的“用户级”应用）都可以用同一套框架相互通信，只需约定好 RIDL 接口即可。类比 Android：系统服务、HAL、用户 app 均可通过 AIDL 通信；Robonix 中，robonix-server、prm 厂商、skill、以及任意自定义 package 都通过 RIDL 接口通信。本仓库示例对应关系：

| 示例 | 类型 | 说明 |
|------|------|------|
| `stream_demo` | stream | 位姿流：stream_server 发布、stream_client 订阅，两进程通过 `robonix/prm/base/pose_cov` 通信 |
| `query_demo` | query | 语义查询：semantic_server 提供、semantic_client 调用，通过 `robonix/system/map/semantic_query` 通信 |
| `skill_demo` | command | greet 命令：skill_server 提供、skill_client 调用，通过 package-local `skill_demo/skill/greet` 通信 |
| `python_ping_client` | query client | 调用 robonix-server 内置 ping，通过 `robonix/system/debug/ping` 通信 |

抽象硬件（相机、底盘、机械臂等）的接口形态见 [抽象硬件原语](../chapter3-developer-guide/primitives/index.md)。

---

## 2. 启动 robonix-server

robonix-server 提供 gRPC meta API（默认 `0.0.0.0:50051`）：节点注册、Query/Stream/Command channel 的分配与解析，并内置 ping query 服务。

```bash
cd rust
./start_server
```

- `start_server` 会：清理占用端口的旧进程 -> 执行 `setup_zenoh_rclrs_env`（source ROS、设置 RMW、colcon build、source overlay）→ 启动 `robonix-server`。
- 环境变量（可选）：`ROBONIX_META_GRPC_ADDR`（监听地址）、`ROBONIX_META_GRPC_ENDPOINT`（客户端连接地址，默认 `127.0.0.1:50051`）、`RMW_IMPLEMENTATION=rmw_zenoh_cpp`、`RUST_LOG`。

确认运行：日志中出现 `meta-runtime: registered node 'robonix-server'`、`ping query runtime ready`；可用 `./examples/callquery robonix-server robonix/system/debug/ping '"test"'` 验证返回 `pong:"test"`。

---

## 3. 使用 ridlc 与生成代码

### 3.1 ridlc 怎么用

- 日常：一般不需要单独跑 ridlc。构建 `robonix-server` 时，`build.rs` 会调用 ridlc 库，处理 `robonix-interfaces/ridl/` 下的 `.ridl`，生成 Rust 与 Python 代码。
- 单独使用（如自建接口树）：  
  `cargo run --manifest-path ridlc/Cargo.toml -- -I <include> -o <out> -i <ridl_dir_or_file> --lang python`  
  需至少一个 `-I` 类型搜索路径，`-o` 为生成输出目录。

### 3.2 RIDL 命名空间 -> Python 模块与函数名

| RIDL 命名空间 | Python 模块路径 |
|---------------|-----------------|
| `robonix/system/debug` | `robonix.system.debug` |
| `robonix/prm/base` | `robonix.prm.base` |
| `robonix/system/map` | `robonix.system.map` |
| `robonix/system/skill` | `robonix.system.skill` |

规则：`robonix/a/b/c` -> `robonix.a.b.c`（斜杠变点号）。

### 3.3 接口类型 -> 生成代码速查

| 原语 | 生成文件名 | Python 主要函数 |
|------|------------|-----------------|
| query | `{name}_query.py` | `create_{name}_client`, `create_{name}_server` |
| stream | `{name}_stream.py` | `create_{name}_publisher`, `create_{name}_subscriber` |
| command | `{name}_command.py` | `create_{name}_client`, `create_{name}_server` |

示例：`query ping` → `robonix.system.debug.ping_query` 模块，`create_ping_client(...)` / `create_ping_server(...)`。详细规范见 [ridlc 开发手册](../chapter3-developer-guide/ridlc.md)。

---

## 4. 创建 package 并开发业务逻辑

本节给出步骤总览与一个最小示例；完整目录结构、manifest 逐字段说明、三种典型 package（机械臂 PRM、语义地图 query server、skill server）的代码骨架与对照表，见 [Package 开发指南](../chapter3-developer-guide/package-development.md)。

**业务逻辑补全要点**：生成代码负责 channel 注册与 ROS 绑定；你只需在指定位置写业务。Query server 用 `server.start(handler)` 传入 handler；Command server 用 `server.execute = fn` 赋值 execute；Stream 发布方在定时器或循环中 `publish(msg)`，订阅方用 `subscriber.start(callback)` 传入回调。详见 [ridlc 开发手册 §5](../chapter3-developer-guide/ridlc.md#5-用户逻辑补全python)。相机/机械臂/地图等厂商接入流程见 [硬件/服务厂商接入指南](../chapter3-developer-guide/vendor-integration.md)。

### 4.1 步骤总览

1. 确定要实现的接口：在 RIDL 里找到对应 query/command/stream（如 `robonix/system/debug/ping`、`robonix/prm/base/pose_cov`），得到生成模块与函数名（如 query 的 `create_ping_server`/`create_ping_client`，stream 的 `create_pose_cov_publisher`/`create_pose_cov_subscriber`）。
2. 创建目录：在任意位置新建 package 根目录，包含 `robonix_manifest.yaml` 以及与包名同名的 Python 子包（如 `python_ping_client/python_ping_client/`）。`setup.py`、`setup.cfg` 由 `rbnx build` 自动生成；`package.xml` 可选（有则使用，可添加自定义 ROS2 依赖；无则自动生成）。`rbnx -p` 可传该目录的路径或 package 名（按约定查找，见下文）。
3. 写 manifest：`package`（id、name、version、vendor、description、license）+ `nodes`（每项一个 node：`id` 建议 `com.syswonder.xxx`，`entry` 为 `模块:函数`，如 `python_ping_client.call_ping:main`）。
4. 写业务代码：在 entry 指向的模块里连接 meta API（gRPC），用生成代码的 `create_*_server` 或 `create_*_client`，在 handler/execute 或 call 处实现逻辑。
5. 用 rbnx 构建与运行：`rbnx build -p <path>` 或 `rbnx build -g <name>` → `rbnx start -p <package> -n <node_id>`（需先启动 robonix-server）；start 会阻塞直到该 package 进程退出，无需 stop。

### 4.2 最小示例：Python ping 客户端

- 目录（示例）：本仓库中的 `rust/examples/python_ping_client/` 仅作参考；你可把 package 放在任意路径，用 `rbnx build -p /path/to/your_package` 指定。
- Manifest 要点：`nodes[].id` 为 node id（本例为 client，可不被调用）；`entry: python_ping_client.call_ping:main` 表示启动时执行 `python_ping_client.call_ping` 的 `main`。
- 业务逻辑：`python_ping_client/call_ping.py` 的 `main()` 中连接 `ROBONIX_META_GRPC_ENDPOINT`，`create_ping_client(runtime_client, requester_id, target)`，构造 `SystemDebugPing.Request()`，`client.call(req)`，打印响应。
- 运行：在 package 根目录或 `rust` 下执行 `rbnx build -p <路径或包名>`，再 `rbnx start -p <路径或包名>`（示例中为 `python_ping_client`，执行一次 ping 后进程退出）。

### 4.3 三类典型 package 速查

| 类型 | RIDL 接口 | 业务逻辑补全位置 |
|------|-----------|------------------|
| 机械臂 PRM（command server） | `robonix/prm/arm/joint_trajectory` | `server.execute = fn`，fn 内收 trajectory、控机械臂、返回 Result |
| 语义地图（query server） | `robonix/system/map/semantic_query` | `server.start(handler)`，handler 内根据 request.filter 查地图、填 response.objects |
| Skill 节点（command server） | `skill_demo/skill/greet`（package-local） | `server.execute = fn`，fn 内处理 typed request、可选 `goal_handle.publish_feedback()`、返回 result |
| 位姿/传感器流（stream） | `robonix/prm/base/pose_cov` 等 | 发布方：定时器/循环中 `publish(msg)`；订阅方：`subscriber.start(callback)`，callback 内处理 msg |

共性：生成代码负责注册/解析 channel 和 ROS 绑定；你只在上述补全位置写业务。node id 必须与 manifest 中 `nodes[].id` 一致，且建议全局唯一（如 `com.syswonder.prm_arm`）。完整补全说明见 [ridlc §5](../chapter3-developer-guide/ridlc.md#5-用户逻辑补全python)。

更详细的目录树、manifest 逐字段、完整代码示例与 colcon 配置，见 [Package 开发指南](../chapter3-developer-guide/package-development.md)。

---

## 5. 使用 rbnx 编译/运行/停止 package

无单独 deployment：直接用 rbnx 对 package 做 build/start/stop。

```bash
rbnx build -p <path>       # 构建本地路径（如 examples/skill_demo），默认增量
rbnx build -p <path> --clean   # 清空 rbnx-build 后全量重建
rbnx build -g <name>       # 构建系统已安装包（rbnx install 安装的）
rbnx start -p <package> -n <node_id>   # 启动并阻塞直到进程退出（Ctrl+C 结束）
```

包管理：`rbnx install --github <repo>` 或 `rbnx install --path <dir>` 安装到 ~/.robonix/packages；`rbnx list` 列出已安装包；`rbnx info <name>` 查看包详情。

- `-p` 可为 package 根目录的绝对或相对路径；也可为 package 名，此时 rbnx 在 `examples/`、当前目录、`rust/examples/` 或 ~/.robonix/packages 下查找。启动时需指定 node：`rbnx start -p <package> -n <node_id>`。
- 启动前需已构建且 robonix-server 已运行（client 需连接 meta gRPC）；server 型 package 启动后会向 robonix-server 注册。

示例：

```bash
cd rust
rbnx build -p python_ping_client
rbnx start -p python_ping_client   # 阻塞直到 ping 执行完并退出
```

---

## 6. 常用路径与验证

| 路径 | 说明 |
|------|------|
| `rust/robonix-interfaces/ridl/` | RIDL 接口定义 |
| `rust/robonix-server/src/generated/` | 生成的 Rust 绑定 |
| `rust/robonix-server/target/rclrs_interfaces_ws/python_pkg/` | 生成的 Python 包 |
| `rust/robonix-server/target/rclrs_interfaces_ws/install/` | colcon 安装空间 |

验证 Zenoh：`echo $RMW_IMPLEMENTATION` 应为 `rmw_zenoh_cpp`；`ps aux | grep zenoh` 可见 Zenoh 相关进程。

一键验证：先启动 robonix-server，再 `./examples/callquery robonix-server robonix/system/debug/ping '"hello"'` 得到 `pong:"hello"`；或 `rbnx start -p python_ping_client` 后使用该 package 的客户端行为验证。
