# 快速上手

[toc]

5 分钟跑起一个完整的 Tiago 仿真 + VLM 对话 demo。

## 1. 前置

需要：

- Linux x86_64 + Rust stable + Python ≥ 3.10
- Docker + Compose v2（仿真容器）
- `uv`（Python 包构建/运行环境管理，`python3 -m pip install --user uv`）
- 一个 OpenAI 兼容的 VLM API key（Qwen / GPT-4o / Gemini / Claude via 兼容网关 都行）

推荐：NVIDIA GPU + `nvidia-container-toolkit`（Webots 3D 渲染）；只跑对话不跑仿真可跳过 Docker。

## 2. 构建

```bash
git clone --recursive https://github.com/syswonder/robonix
cd robonix
make install
```

`make install` 会：
- 编译并把 `rbnx`、`robonix-atlas`、`robonix-pilot`、`robonix-executor`、`robonix-liaison`、`robonix-codegen` 装到 `~/.cargo/bin/`
- 自动登记当前 clone 为 robonix 源码根目录，让其他位置的包做 codegen 时能找到 contracts/IDL（见 [Build 与 Codegen](../integration-guide/build-and-codegen.md)）

> Python 依赖不由 `package_manifest.yaml` 声明。`package_manifest.yaml` 描述的是包的 `build` / `start` 命令、能力约定和包依赖；Python 依赖由各包自己的构建脚本、`pyproject.toml` / `uv.lock` 或镜像构建过程安装。`rbnx start` 只负责在启动前把本包 codegen 产物加入运行环境，例如 `rbnx-build/codegen/proto_gen`。

默认 ROS 2 RMW 是 Zenoh：

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

这不是把系统绑死在 Zenoh。Webots 容器、mapping、nav2、explore、scene、camera/lidar/chassis primitives 都读取同一个 `RMW_IMPLEMENTATION`；默认是 `rmw_zenoh_cpp`，需要对比时可以切回 Fast DDS：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

默认选 Zenoh RMW 的原因：Webots demo / CI 是单机多容器 ROS graph，TF、RGB-D、lidar、map、Nav2、scene 都是高频 topic。Fast DDS 在这个拓扑里主要问题是发现和跨容器通信不稳定，且 DDS discovery/state 开销偏重。Zenoh RMW 的运行模型不同：ROS 2 API 不变，底层走 Zenoh；默认使用本机 `rmw_zenohd` router daemon 负责 discovery 和 routed traffic，节点间数据仍可 peer-to-peer。Webots sim 容器在 `RMW_IMPLEMENTATION=rmw_zenoh_cpp` 时会自动启动 router，普通 quickstart 不需要手动起 daemon。

依据：

- [`rmw_zenoh` design](https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md)：说明 `rmw_zenoh_cpp` 如何映射 ROS 2 RMW API，并依赖本机 Zenoh router 做 discovery / host-to-host 通信。
- Chovet et al., ["Performance Comparison of ROS2 Middlewares for Multi-robot Mesh Networks in Planetary Exploration"](https://link.springer.com/article/10.1007/s10846-024-02211-2)：Table 4 报告 Zenoh 相对其他 RMW 在动态 mesh 实验中 reachability 提升 146.93% / 58.17%，单消息 data overhead 降低 47.82% / 25.93%，CPU usage 降低 41.27% / 39.76%；代价是 RAM usage 增加。
- Liang et al., ["A Performance Study on the Throughput and Latency of Zenoh, MQTT, Kafka, and DDS"](https://arxiv.org/abs/2303.09419)：以 throughput 和 latency 比较 Zenoh、DDS 等协议，实验结果显示 Zenoh 在该测试设置下优于 DDS。

## 3. 配 VLM

`rbnx boot` 通过环境变量读取 VLM endpoint（manifest 里以 `${VLM_*}` 形式引用）：

```bash
# OpenAI（或任意 OpenAI 兼容网关）
export VLM_API_KEY=sk-xxx
export VLM_BASE_URL=https://api.openai.com/v1
export VLM_MODEL=gpt-5.5

# Qwen（阿里 DashScope 提供 OpenAI 兼容网关）
export VLM_API_KEY=sk-xxx
export VLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
export VLM_MODEL=qwen3-vl-plus
```

把这几行写进 `~/.bashrc` / `~/.zshrc` 或部署目录的 `.env` 都可以。

## 4. 跑起来

整个栈分两个终端，仿真和 Robonix 系统服务/驱动各占一个：

```bash
# T1：仿真容器（Webots + ROS 2 + 3 个 driver，docker compose 栈，Ctrl-C 停）
# sim/start.sh 末尾会自动启动 rviz2，所以不需要手动开。
export DISPLAY=:0
bash examples/webots/sim/start.sh

# 可选：仅 CI/headless 调试时打开浏览器 stream；普通 quickstart 默认用上面的 Webots GUI。
# export ROBONIX_SIM_STREAM=1
# export WEBOTS_HEADLESS_MODE=auto
# bash examples/webots/sim/start.sh

# T2：Robonix 系统服务 + 系统/服务/技能包
export VLM_BASE_URL=https://api.openai.com/v1
export VLM_API_KEY=sk-...
export VLM_MODEL=gpt-5.5
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
cd examples/webots
rbnx build
rbnx boot
```

T1 不是 Robonix 包——它就是个 docker compose 栈。Robonix 不管它的生命周期。

T2 的 `rbnx boot` 读 `examples/webots/robonix_manifest.yaml`，按声明顺序拉起所有组件。当前 webots 部署一共 15 个：

- `system` 块（5）：`atlas` + `scene` + `executor` + `pilot` + `liaison`
- `primitive` 块（4）：`tiago_chassis` + `tiago_camera` + `tiago_lidar` + `audio_driver`
- `service` 块（5）：`mapping`（rtabmap 2D + RGBD fusion）+ `nav2`（ROS 2 Nav2 导航）+ `memory`（向量记忆检索）+ `speech` + `voiceprint`
- `skill` 块（1）：`explore`（frontier 自主探索）

driver 进程跑在仿真容器里（`docker exec`），系统服务（scene、mapping）跑在它们各自的容器里加入同一个 ROS 2 graph；底层通信由 `RMW_IMPLEMENTATION` 选择的 RMW transport 承载（默认 Zenoh，也可切回 Fast DDS）。host 上不需要 ROS 2 环境。

`rbnx boot` 报告全部组件 up 后即可进入下一步。具体启动时序见 [系统部署与启动流程](../architecture/deployment-and-startup.md)。

> **scene 第一次构建较慢**：`rbnx build` 会按 manifest 构建 scene 容器，首次会拉 torch/cu124 wheel、concept-graphs 源码，并预下 YOLO-World + MobileSAM 权重（约 8–12 分钟，取决于网络和缓存）。如果只想单独预热 scene，可以在仓库根目录运行 `rbnx build -p system/scene`，或在 `examples/webots` 下运行 `rbnx build -p ../../system/scene`。
>
> 之后打开 <http://localhost:50107/> 看 scene 的 3 栏实时面板：左 = 2D occupancy + 物体定位，中 = 3D 点云 + bbox + Tiago 本体，右 = 相机 RGB + 深度直播流。

> **cache 注意**：`rbnx boot` 第一次会把 `mapping`、`nav2`、`explore` 这些 `url:` 远端包克隆到 `examples/webots/rbnx-boot/cache/`，之后复用这份 checkout。`rbnx boot` / `rbnx build` 会提示本地 cache 是否落后于上游；需要同步时不要手动删 cache，直接运行：
>
> ```bash
> rbnx update                  # 更新当前部署里的所有远端包，会询问确认
> rbnx update -p <package dir>  # 只更新某个包
> ```

## 5. 跟机器人对话

第三个终端：

```bash
rbnx caps          # 列出所有注册的能力提供者 + 其每条 capability
rbnx tools         # LLM 看到的工具列表（MCP transport 子集）
rbnx chat          # 直连 pilot 的 ratatui TUI
```

`rbnx chat` 里输入问题即可。典型一轮：

```
You:   what can you see?
Pilot: I'll capture a current RGB camera snapshot to see what's in view.
       > [r0] camera_snapshot({})
Pilot: The camera shows a potted plant near a beige wall …
```

按 **Esc** 中断当前推理（`AbortSession`）。退出 chat：`Ctrl+C`。

### 自主探索并构建语义地图

`explore` 是自带的 frontier 探索 skill，pilot 会把它当 MCP 工具调用：

```bash
# 一次性 prompt，事件流打到 stdout，state 终止时退出
rbnx ask "请启动 explore 完整探索当前房间，完成后总结地图覆盖和发现的物体"
```

跑起来之后：
- `rbnx-boot/logs/service_explore.log` 每跳 frontier 打一行 `driving to frontier (x,y) size=...`，每个 sweep 步打 `sweep at (x,y) yaw=...°`
- `mapping_rbnx` 的 occupancy grid 会随机器人走过填空白
- `scene` 的 3 栏 web UI 实时反映：物体进 registry、点云累计、地图扩张

默认 Tiago/Webots 场景完成一次完整探索（约 6 个 frontier hops）约需 3–4 分钟。`explore` 是长任务工具，Pilot / Executor 会根据工具返回的 `run_id` 和状态事件跟踪执行，不需要在 prompt 里要求模型每 5 秒手动轮询。若用一次性 shell 命令希望限制最长等待时间，可以在外层加 `timeout 240 rbnx ask "..."`；也可以改用 `rbnx chat` 交互式运行。

## CI Webots 怎么工作

CI 不复用服务器上已有的 Webots，也不复用其他 PR 的 sim 容器。每次 Webots integration run 都会在 self-hosted GPU runner 上按当前 checkout 独立完成：

1. 初始化 submodule 和远端包 cache，构建 Webots 部署镜像/组件。
2. 启动一套新的 Webots Tiago 仿真容器。CI/headless 模式才打开 Webots stream；普通 quickstart 默认是本地 GUI。
3. 启动 deterministic fake VLM。它只替代 LLM 规划随机性，返回固定 RTDL tree；Webots、ROS topics、TF、camera/lidar、mapping、Nav2、scene、explore、executor/pilot 都走真实代码路径。
4. `rbnx boot` 当前 checkout，等待组件注册和 ACTIVE，再跑 interface checks、mapping occupancy grid readiness、scenario suite。
5. 生成 HTML report，内嵌 scenario JSONL、provider logs、sim/ROS logs、环境元数据和可选 LLM 分析；push、PR、manual dispatch、`@robonix-ci test` 都生成同样结构的 report，并发布到 Pages 历史目录。

清栈：

```bash
bash examples/webots/sim/stop.sh   # 一键 kill 容器内 driver + rbnx boot + docker compose down
```

## 高级调试：只起子集

这不是 quickstart 主流程，只用于开发者调试单个包或接入外部已运行的系统服务。

```bash
# 跳过 system 块：要求 atlas / executor / pilot / liaison 等已经由外部进程启动
rbnx boot --skip-system

# 单独起一个包：要求 atlas 已运行；config 与 robonix_manifest.yaml 里的 config 块同形
rbnx start -p ./primitives/tiago_chassis --config tiago_chassis.local.yaml
rbnx start -p ./primitives/tiago_chassis --set can_port=/dev/can0 --set max_speed=0.4
```

`rbnx start` 会运行该包的 `start` 命令；如果包注册了 `*/driver` capability，并且传入了 `--config` / `--set`，配置会作为 `Driver(CMD_INIT, config_json)` 注入 provider。

## 运行时自省

栈跑起来之后：

```bash
rbnx caps                    # 所有注册的能力提供者及其 capability
rbnx tools                   # agent 可见的 MCP 工具
rbnx describe --provider <provider_id>   # 某个能力提供者的 CAPABILITY.md 全文
rbnx channels                # 当前活跃的 consumer→provider 通道
rbnx inspect                 # 完整 runtime 快照（JSON）
```

## 下一步

- [系统组件](../architecture/components.md)——12 个系统组件的职责与实现状态
- [接入指南](../integration-guide/index.md)——把自己的硬件或算法接入 Robonix
- [Build 与 Codegen](../integration-guide/build-and-codegen.md)——包作者必读（`rbnx setup`、`rbnx codegen`、自定义 contract）
- [接口目录](../interface-catalog/index.md)——`primitive/*` 原语与 `service/*` 服务的能力约定定义

## 常见问题

**Webots 没显示 GUI**：本地桌面确认 `echo $DISPLAY` 非空，运行 `xhost +local:docker`。`ROBONIX_SIM_STREAM=1` 只用于 CI/headless 调试；普通用户 quickstart 不需要打开 stream。

**Webots 卡顿**：确认 `nvidia-smi` 可用且装了 `nvidia-container-toolkit`；否则跑在纯 CPU 软光栅上性能会显著下降。

**MCP 工具暂时不可见（`rbnx tools` 空）**：T1 仿真 + T2 `rbnx boot` 全部就绪需 ~10 s，等一会儿；如果一直空，看 `rbnx-boot/logs/<name>.log`。

**工具调用被 schema 拒绝**：优先看对应 provider log 和 `rbnx tools` 输出。provider 应使用 codegen 产物和 `@<provider>.mcp(...)` / `@mcp_contract(...)` 暴露工具，schema 由能力约定 IDL 生成；不要手写一份与 IDL 不一致的 JSON schema。详见[开发者指南 §14.9](../developer-guide.md#149-servicemcp--servicegrpc)。
