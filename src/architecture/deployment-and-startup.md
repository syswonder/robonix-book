# 系统部署与启动流程

本页讲清楚一次完整的 Robonix 部署在终端里发生了什么——从 `rbnx boot` 第一行 log 到 `rbnx chat` 收到第一个工具调用之间的所有事件。读完应该能：自己写一份 deploy manifest、看懂 `rbnx-boot/logs/` 里的输出、定位"组件起不来"或"LLM 看不到工具"这类问题在哪个阶段。

## 两层 manifest

| 文件 | 谁读 | 范围 |
|---|---|---|
| `<deployment>/robonix_manifest.yaml` | `rbnx boot` | 一次部署：列系统服务的配置、哪些设备、服务，对应的代码包路径，实例名…… |
| `<package>/package_manifest.yaml`    | `rbnx start` | 单个包：build 命令、start 命令、提供哪些 capability、依赖哪些其他包 |


## Webots Tiago 例子的两个终端

`examples/webots/` 是仓库内置的端到端样例——驱动在仿真容器里跑，Robonix 系统服务和 Pilot 在主机上跑。部署 layout 见 [快速上手](../getting-started/quickstart.md)。整个栈分两个终端启动：

```bash
# T1：仿真环境（Ctrl-C 停）
bash examples/webots/sim/start.sh

# T2：Robonix（atlas + executor + pilot + 4 个 driver + nav2）
cd examples/webots
export VLM_BASE_URL=https://api.openai.com/v1
export VLM_API_KEY=sk-...
export VLM_MODEL=gpt-5.4-mini
rbnx boot
```

> 仿真容器（Webots + ROS 2）不是 Robonix 包，它就是个 docker compose 栈。Robonix 不管它的生命周期；T1 终端 Ctrl-C 即可停。
>
> driver 进程（chassis、camera、lidar、nav2）跑在仿真容器**里面**——`rbnx boot` 通过 `docker exec robonix_tiago_sim ...` 把 Python driver 起在容器进程空间，让它们与 Webots 共享同一份 DDS graph。host 上不需要 ROS 2 环境。

整个栈起来后开第三个终端：

```bash
rbnx caps        # 列出所有注册的 capability 和它们的 interface
rbnx tools       # LLM 看到的工具列表（MCP transport 子集）
rbnx chat        # ratatui TUI，直连 Pilot
```

清栈：`bash examples/webots/sim/stop.sh`——脚本会一并 kill 容器内 driver 进程、`rbnx boot` 子进程组、并 `docker compose down` 仿真栈。

## `rbnx boot` 生命周期

`rbnx boot` 主流程在 `rust/crates/robonix-cli/src/cmd/deploy.rs`，七步：

1. **解析 manifest**：读 `robonix_manifest.yaml`，展开 `${VAR}` 环境变量，校验声明的 package 存在、capability 引用合法。
2. **初始化日志目录**：默认 `<manifest-dir>/rbnx-boot/logs/`，每个组件一个 `<name>.log` 文件。可用 `--log-dir` 改路径。
3. **起 system 块**：按 `system:` 下的字段顺序起 atlas → executor → pilot → liaison → memory → ……。
4. **轮询 atlas 就绪**：调 `QueryCapabilities("")` 直到返回非错（atlas 完全起来需 ~200 ms）。
5. **逐个起 primitive / service / skill**：按 manifest 声明顺序，一条一条 spawn，**每条等它在 atlas 里 `RegisterCapability` 上来才进入下一条**。
6. **driver init dance**：如果新注册的 cap 声明了 `*/driver` 接口（如 `robonix/primitive/lidar/driver`），调一次 `LifecycleDriver.Driver(CMD_INIT, config_json)` 完成硬件初始化。
7. **守候**：sit-on-Ctrl-C/SIGTERM 循环，收到信号后向所有子进程发 SIGTERM、等回收，再退出。

每个子进程的 stdout / stderr 重定向到 `<log-dir>/<name>.log`，前台终端只看 `[deploy]` 自己的状态行。组件 panic 或 register 超时时 `rbnx boot` 会打印失败摘要并指向对应 log 文件。


时间线大致如下（host = 主 Robonix 终端，sim = 仿真容器）：

```
T+0     T1: bash sim/start.sh           # docker compose up，Webots GUI 弹出
T+10s   sim:  Webots + ROS 2 + Nav2 全部 up，DDS graph 准备好
T+15s   T2: rbnx boot                 # 读 robonix_manifest.yaml
T+15s   host: spawn robonix-atlas       # listen 50051
T+16s   host: atlas RegisterCapability "robonix/system/atlas"   # self-register
T+16s   host: spawn robonix-executor    # connect to atlas，RegisterCapability
T+17s   host: spawn robonix-pilot       # discover vlm/memory caps from atlas
T+18s   host: docker exec sim python chassis_driver/driver.py
T+19s   sim:  chassis driver RegisterCapability + DeclareInterface (state, move) MCP
T+19s   host: deploy 收到 register 通知 → 进入下一条 primitive
T+20s   ...重复 camera / lidar / nav2...
T+24s   host: ✓ 7 component(s) up
T+25s   T3: rbnx caps                  # 看到全部能力
T+25s   T3: rbnx chat                  # 直连 pilot SubmitTask
T+26s   user: "what can you see?"      # → pilot → vlm → tool_calls
T+27s   pilot: read_file CAPABILITY.md（懒加载） → camera_snapshot →
                 executor → docker exec MCP HTTP → driver → image
```

第一次部署慢主要在仿真容器拉镜像 + Webots 启动。后续 deploy 在已有容器上 docker exec，从 `rbnx boot` 到全部 ready 通常 5–8 秒。