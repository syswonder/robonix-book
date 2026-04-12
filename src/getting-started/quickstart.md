# 快速上手

5 分钟跑起一个完整的 Tiago 仿真 + VLM 对话 demo。

## 1. 前置

需要：

- Linux x86_64 + Rust stable + Python ≥ 3.10
- Docker + Compose v2（仿真容器）
- 一个 OpenAI 兼容的 VLM API key（Qwen / GPT-4o / Gemini 都行）

推荐：NVIDIA GPU + `nvidia-container-toolkit`（Webots 3D 渲染）；若只跑对话不跑仿真可跳过 Docker。

## 2. 构建

```bash
git clone --recursive https://github.com/syswonder/robonix
cd robonix/rust
make install
pip install -r examples/requirements.txt
```

`make install` 会：
- 编译并把 `rbnx`、`robonix-atlas`、`robonix-pilot`、`robonix-executor`、`robonix-liaison`、`robonix-codegen` 装到 `~/.cargo/bin/`
- 自动登记当前 clone 为 robonix 源码根目录，让包的 codegen 无论在哪都能找到 contracts/IDL（见[Build 与 Codegen](../integration-guide/build-and-codegen.md)）

## 3. 配 VLM

```bash
cp examples/.env.example examples/.env
$EDITOR examples/.env     # 任选一个 OpenAI 兼容的 endpoint
```

当前 `vlm_service` 只对接 OpenAI 兼容接口（`/v1/chat/completions`）。下面两个示例都适用：

```bash
# OpenAI（或其他 OpenAI 兼容商）
VLM_API_KEY=sk-xxx
VLM_BASE_URL=https://api.openai.com/v1
VLM_MODEL=gpt-4o

# Qwen（阿里 DashScope 提供 OpenAI 兼容网关）
VLM_API_KEY=sk-xxx
VLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
VLM_MODEL=qwen3-vl-plus
```

## 4. 跑起来

```bash
./examples/run.sh
```

脚本会启动 atlas → vlm_service → tiago_sim_stack（docker，含 Webots + Nav2）→ memsearch_service → executor → pilot → liaison，并自动做各包的 codegen。首次启动因为要拉 docker 镜像 + 编译会慢一些；后续秒起。

## 5. 跟机器人对话

**在 Webots / rviz2 完全起来之后**另开一个终端连 Pilot——tiago 容器里 Webots + Nav2
启动约需 40 秒，期间机器人相关的 MCP 接口还没有注册完，提前连会看不到相关工具：

```bash
rbnx chat
```

输入问题即可。典型一轮：

```
You:   what can you see?
Pilot: I'll capture a current RGB camera snapshot to see what's in view.
       > [r0] camera_snapshot({})
Pilot: The camera shows a potted plant near a beige wall …
```

按 **Esc** 中断当前推理（`AbortSession`）。退出：`Ctrl+C`。

## 只跑子集

```bash
START_SIM_STACK=0 ./examples/run.sh    # 不启动 Webots 仿真，只 VLM + agent
START_AGENT=0     ./examples/run.sh    # 只启控制面 + 仿真（调试桥接时用）
```

## 去看看里面发生了什么

栈跑起来之后：

```bash
rbnx nodes      # 所有注册节点
rbnx tools      # agent 可见的工具清单
rbnx graph -o topology.png    # 系统拓扑图
rbnx inspect    # 完整 runtime 快照（JSON）
```

## 下一步

- [系统全景](../architecture/overview.md)——控制面 / 数据面、一次请求的完整链路
- [接入指南](../integration-guide/index.md)——把自己的硬件或算法接入 Robonix
- [Build 与 Codegen](../integration-guide/build-and-codegen.md)——包作者必读（`rbnx setup`、`rbnx codegen`、自定义 contract）
- [接口目录](../interface-catalog/index.md)——`prm/*` 原语与 `srv/*` 服务的契约定义

## 常见问题

**Webots 没显示 GUI**：确认 `echo $DISPLAY` 非空，运行 `xhost +local:docker`。

**Webots 卡顿**：确认 `nvidia-smi` 可用且装了 `nvidia-container-toolkit`；否则跑在纯 CPU 软光栅上会很慢。

**MCP 工具暂时不可见（`rbnx tools` 空）**：容器里 Webots + Nav2 首次启动需 ~40 s，等一会儿。

**VLM 报错但 pilot 没崩**：符合预期——错误会以普通消息出现在 chat 里，session 不死，直接发下一条即可。
