# 端口与端点速查

robonix 大部分内部 gRPC 接口都是 **动态端口**（package 启动时 `bind 0`，把分配到的端口写进 atlas，消费者通过 `ConnectCapability` 拿到 endpoint 再拨）。下面列的是**静态、对外可见**的端口 —— 你写客户端、调试、或者要从浏览器直接打开 UI 时需要知道这些。

## 系统层（全部由 `rbnx boot` 起，listen 在 `system:` block 里改）

| 服务 | 默认 listen | 协议 | 用途 |
| --- | --- | --- | --- |
| `atlas` | `127.0.0.1:50051` | gRPC | 能力注册中心。所有其他东西都连这个；改 listen 要同步改各家的 `--atlas` |
| `executor` | `127.0.0.1:50061` | gRPC | Plan 派发器；内置 `read_file` / `write_file` / `search_memory` 等 builtin MCP 工具 |
| `pilot` | `127.0.0.1:50071` | gRPC | VLM 推理 + 任务分解；`SystemPilot.SubmitTask` |
| `liaison` | `127.0.0.1:50081` | gRPC | **用户入口**：`rbnx chat` / mobile / API 都连这；`SystemLiaison.{SubmitTask,StartVoiceSession}` |

改任意一个：在 `examples/webots/robonix_manifest.yaml` 的 `system:` 块对应 entry 加 `listen: <host>:<port>`。

## 服务层（动态 + 一个静态 web UI）

| 服务 | 动态 gRPC | 静态 HTTP / Web UI |
| --- | --- | --- |
| `scene` | OS 分配，atlas 查 | `http://0.0.0.0:50107/`（Live：物体表 + 机器人位姿 + 关系图）<br>覆盖：`SCENE_WEB_PORT` 环境变量或 `system: scene: { web_port: ... }`，`0` 关闭 |
| `memory` | OS 分配 | — |
| `speech` | OS 分配（`SPEECH_PORT` 可指定） | — |
| `soma` | OS 分配 | — |

scene/memory/speech 的 MCP HTTP 端口是 OS 分配的——pilot 通过 atlas 自动发现，**不要硬编码**。需要的话查 `rbnx caps --verbose`。

## 原语层（全部动态，按 cap_id 查）

| 包 | 注册的 cap | 接口 |
| --- | --- | --- |
| `tiago_chassis` | `com.robonix.primitive.tiago_chassis` | gRPC `chassis/move` + ROS2 `chassis/{odom,twist_in,pose}` |
| `tiago_camera` | `com.robonix.primitive.tiago_camera` | gRPC `camera/snapshot,depth_snapshot` + ROS2 `camera/{rgb,depth,extrinsics}` |
| `tiago_lidar` | `com.robonix.primitive.tiago_lidar` | gRPC `lidar/snapshot` + ROS2 `lidar/lidar` |
| `audio_driver` (Linux ALSA) | `com.robonix.primitive.audio` | gRPC `audio/{mic,speaker}` |
| `audio_macos_bridge` | 同上 — 替换 `audio_driver` 跑 mic/speaker 物理在 macOS | gRPC `audio/{mic,speaker}`，内部 WS proxy |

## audio_macos_bridge — macOS 端（外部进程，不属于 rbnx boot）

跑在 macOS 上的 `mac_server/server*.py` 暴露：

| 端口 | 协议 | 路径 / 用途 |
| --- | --- | --- |
| `60000` | WS | `/mic` 服务端流 PCM；`/speaker` 客户端流 PCM；`/health` 健康检查；`/devices`、`/vu`、`/log`、`/set_device`（仅 web 版） |
| `60001` | HTTP | `server_web.py` 的调试 UI；浏览器打开 `http://localhost:60001/` 选设备、看 VU meter、滚日志 |

`60001` 默认只 bind `127.0.0.1`，要别的机器访问改 `--ui-host 0.0.0.0`（注意没鉴权）。

`60000` 默认 bind `0.0.0.0`，让 LAN 内 robonix Linux 端能 dial。如果 macOS 在 tailscale 里，rbnx-cli 这边的 manifest 写 tailscale IP（如 `100.78.x.x`）。

## rbnx-cli 自身

`rbnx chat` / `rbnx ask` 不监听端口，它们做客户端：

1. 找 atlas（`--server` 默认 `127.0.0.1:50051`）
2. `QueryCapabilities("robonix/system/liaison", grpc)` → `ConnectCapability` → 拿到 liaison 的 endpoint（`127.0.0.1:50081`）
3. 直连 liaison

跨机器：`rbnx chat --server <debian-ip>:50051` 就能从笔电连到远端的 robonix 大脑。

## 修改 listen 的优先级

每个 system 服务都按这个优先级解析监听地址：

1. CLI flag (`--listen`)
2. `system: <name>: { listen: ... }` from `robonix_manifest.yaml`
3. 各服务自己的硬编码默认值

`rbnx boot` 在 spawn 前会用 `TcpStream::connect_timeout` 探一下端口，被占了就 `[FAIL]` 早退（不会静默 shadow 一个老 daemon）。
