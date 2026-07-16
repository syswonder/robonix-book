# 多架构与多运行环境部署


Robonix 的机器人部署清单可以为每个软件包实例选择不同的软件包清单。软件包作者据此提供 x86_64、Jetson arm64、本机或容器等构建与启动实现。

这不是一张由 Robonix 强制定义的固定目标矩阵。一个软件包只支持它实际提供并测试过的清单文件。

## 选择软件包清单

默认情况下，`rbnx build` 和 `rbnx start` 读取：

```text
<package>/package_manifest.yaml
```

部署项可用 `manifest:` 选择同一软件包中的另一个文件：

```yaml
system:
  scene:
    manifest: package_manifest.jetson-native.yaml
    camera_frame: front_camera_rgb_optical_frame
    camera_provider_id: front_camera
```

`rbnx build -f robonix_manifest.yaml` 使用该文件的 `build:`；`rbnx boot` 内部执行 `rbnx start --manifest <file>`，使用同一文件的 `start:` 和 `stop:`。文件不存在时直接失败，不会静默回退。

常见文件名是约定，不是 CLI 枚举：

| 文件名示例 | 通常表示 |
|---|---|
| `package_manifest.yaml` | 软件包默认目标 |
| `package_manifest.jetson-native.yaml` | Jetson 宿主机原生运行 |
| `package_manifest.jetson-docker.yaml` | Jetson 的 L4T/NVIDIA 容器 |

软件包也可以定义其它名称。部署仓库必须只引用该软件包实际存在的文件。

## 一个目标清单应包含什么

不同目标清单应保持相同的软件包身份与公开能力，只改变构建和运行实现：

```yaml
manifestVersion: 1

package:
  name: com.robonix.system.scene
  version: 0.1.0
  vendor: robonix
  description: Scene — semantic + geometric live map (Jetson + native host-ROS2 target).
  license: MulanPSL-2.0

build: RBNX_BUILD_TARGET=jetson-native bash scripts/build.sh
start: ROBONIX_SCENE_FORCE=native bash scripts/start.sh

capabilities:
  - name: robonix/system/scene/list_objects
  - name: robonix/system/scene/get_robot_context
  - name: robonix/system/scene/goal_near
  - name: robonix/system/scene/goal_room
  - name: robonix/system/scene/get_scene_graph
  - name: robonix/system/scene/get_object_context
  - name: robonix/system/scene/list_relations
```

这段内容直接摘自当前 Scene 的 Jetson 本机清单。`RBNX_BUILD_TARGET`、`ROBONIX_SCENE_FORCE` 是 Scene 脚本自己的实现约定；`rbnx` 只执行清单中的 shell 命令，不解释这些变量。

## 本机与容器的边界

### 本机软件包

本机软件包直接使用宿主机的系统库、ROS 2、设备文件和 GPU 运行时。它的目标清单应在 `build:` 中验证必要依赖。使用 Robonix ROS 2 类型时，构建脚本应先加载目标平台的系统 ROS 2，再生成并用 colcon 构建 `rbnx codegen --ros2` 产物。运行时需要加载系统 ROS 2 和已安装的叠加工作区；`rbnx start` 只会自动加载 `<package>/rbnx-build/ws/install/setup.bash`，使用其他输出路径的软件包必须在自己的 `start:` 命令中加载对应叠加层。

适合本机运行的情况包括：

- Jetson 已预装与 JetPack 匹配的 CUDA、TensorRT、PyTorch 等系统软件包。
- 硬件 SDK 或 ROS 驱动已由软件包自带构建脚本安装或编译。
- 需要直接访问 SocketCAN、USB、串口或宿主机 ROS 2 graph。

不要假定另一台 Jetson 与当前机器具有相同 JetPack、CUDA 或 Python wheel。每个软件包的 README 和目标清单是前置条件的权威来源。

### 容器软件包

容器软件包由 `build:` 构建镜像，由 `start:` 创建容器。软件包脚本必须显式处理：

- CPU 架构与基础镜像。
- GPU 运行时和设备映射。
- ROS 2 环境与 RMW 实现。
- Atlas 的可达地址。
- 配置、模型缓存和持久化数据的挂载。
- 信号处理和容器清理。

`rbnx` 不会自动把 ROS 2、CUDA、设备或环境变量注入任意容器；这些属于软件包的启动实现。

## 当前仓库中可验证的例子

| 软件包 | 默认清单 | 额外清单 |
|---|---|---|
| 场景服务 | `package_manifest.yaml`，x86 Docker | `package_manifest.jetson-native.yaml`、`package_manifest.jetson-docker.yaml` |

当前 Robonix 源码快照中，只有场景服务同时提供上述三个清单。Webots 部署清单通过 `url:` 引用建图和导航外部软件包，但它们的目标清单由各自上游仓库定义，不能从本源码树推断。记忆、语音、某个原语或第三方技能也不会因此自动支持相同目标。接入时应先检查实际软件包根目录：

```bash
find /path/to/package -maxdepth 1 -name 'package_manifest*.yaml' -print
```

## 本体专用配置属于部署仓库

目标清单决定“如何构建和运行软件包”；部署项的 `config:` 决定“这个机器人实例如何配置”。两者不要混合。

机器人专用参数通常由部署仓库管理。仅当软件包明确约定从部署目录解析相对路径时，建图和导航参数可以按下面的形式组织：

```text
robot-deploy/
├── robonix_manifest.yaml
└── config/
    ├── rtabmap_params.yaml
    └── nav2_params.yaml
```

```yaml
service:
  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    config:
      params_file: config/rtabmap_params.yaml
      sensor_providers:
        lidar2d: front_lidar
        rgb: front_camera
        depth: front_camera
        odom: base_chassis

  - name: nav2
    url: https://github.com/syswonder/service-navigation-rbnx
    config:
      params_file: config/nav2_params.yaml
      provider_ids:
        map: mapping
        odom: base_chassis
        scan: front_lidar
```

`rbnx` 把 `params_file` 等软件包配置当作不透明值，不会代替软件包解析任意文件路径。只有声明 `*/driver` 能力约定的提供方会通过 `Driver(CMD_INIT, config_json)` 收到这份配置，并应在 `on_init` 中明确相对路径以部署目录、软件包目录还是其他目录为基准。没有 driver 的软件包不会从这条路径收到 `config:`，必须在自己的启动实现中定义配置入口。上游外部包的 profile 兼容性和路径约定应以该软件包当前实现为准。

## ROS 2、RMW 与 Zenoh

原语和服务继续编写标准 ROS 2 发布者、订阅者、服务或动作代码，不需要调用 Zenoh 专用接口。实际传输由 ROS 2 的 RMW 层选择。

同一 ROS 2 通信图中的进程必须使用一致的实现，例如：

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
rbnx boot -f robonix_manifest.yaml
```

`rbnx boot` 启动的本机子进程继承该环境。容器软件包的 `start.sh` 必须把 `RMW_IMPLEMENTATION` 转发进容器。

使用 `rmw_zenoh_cpp` 时，还需要一个所有参与者都能到达的 `rmw_zenohd` 路由器。Webots 示例在仿真容器中启动路由器。真实机器人可以在宿主机或网络中的固定节点启动，但同一通信图不应由每个软件包各自盲目启动一份路由器。

在使用 ROS 官方二进制仓库的目标机上，安装与当前 ROS 发行版一致的软件包，并确认路由器入口存在：

```bash
export ROS_DISTRO=humble  # 改为目标机实际安装的 ROS 2 发行版
source "/opt/ros/$ROS_DISTRO/setup.bash"
sudo apt update
sudo apt install -y "ros-$ROS_DISTRO-rmw-zenoh-cpp"
test -x "/opt/ros/$ROS_DISTRO/lib/rmw_zenoh_cpp/rmw_zenohd"
```

由机器人 deployment 的启动脚本统一拉起路由器。下面是最小宿主机流程：保存 PID、等待默认 TCP `7447` 就绪，并把日志留在本次部署目录。

```bash
set -euo pipefail
mkdir -p rbnx-boot/logs
ZENOH_ROUTER_BIN="/opt/ros/$ROS_DISTRO/lib/rmw_zenoh_cpp/rmw_zenohd"
ZENOH_ROUTER_PID_FILE="rbnx-boot/rmw_zenohd.pid"
ZENOH_ROUTER_EXPECTED="$(readlink -f "$ZENOH_ROUTER_BIN")"

stop_started_router() {
  test -f "$ZENOH_ROUTER_PID_FILE" || return 0
  pid="$(cat "$ZENOH_ROUTER_PID_FILE")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    actual="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"
    test "$actual" = "$ZENOH_ROUTER_EXPECTED" || {
      echo "refusing to stop PID $pid: executable is $actual" >&2
      return 1
    }
    kill -TERM "$pid"
    for _ in $(seq 1 20); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$ZENOH_ROUTER_PID_FILE"
}

trap stop_started_router EXIT INT TERM
"$ZENOH_ROUTER_BIN" >rbnx-boot/logs/rmw_zenohd.log 2>&1 &
echo "$!" >"$ZENOH_ROUTER_PID_FILE"

ready=0
for _ in $(seq 1 20); do
  if python3 - <<'PY'
import socket
with socket.create_connection(("127.0.0.1", 7447), timeout=0.2):
    pass
PY
  then
    ready=1
    break
  fi
  sleep 0.25
done
kill -0 "$(cat "$ZENOH_ROUTER_PID_FILE")"
test "$ready" -eq 1 || {
  tail -n 80 rbnx-boot/logs/rmw_zenohd.log
  exit 1
}
trap - EXIT INT TERM
```

关闭 deployment 时重新核对 PID 对应的可执行文件，只终止这一个路由器；PID 文件过期或被复用时不会误杀其他进程：

```bash
ZENOH_ROUTER_BIN="/opt/ros/$ROS_DISTRO/lib/rmw_zenoh_cpp/rmw_zenohd"
ZENOH_ROUTER_PID_FILE="rbnx-boot/rmw_zenohd.pid"
if test -f "$ZENOH_ROUTER_PID_FILE"; then
  pid="$(cat "$ZENOH_ROUTER_PID_FILE")"
  expected="$(readlink -f "$ZENOH_ROUTER_BIN")"
  actual="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"
  if kill -0 "$pid" 2>/dev/null; then
    test "$actual" = "$expected" || {
      echo "refusing to stop PID $pid: executable is $actual" >&2
      exit 1
    }
    kill -TERM "$pid"
    for _ in $(seq 1 20); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$ZENOH_ROUTER_PID_FILE"
fi
```

如果 `apt` 仓库没有目标 ROS 发行版的 `rmw_zenoh_cpp`，应按该发行版的[官方构建说明](https://github.com/ros2/rmw_zenoh)构建并安装到独立工作区，再在启动所有 ROS 2 进程前加载该工作区的 `install/setup.bash`。不要混用为另一 ROS 发行版构建的二进制。

部分软件包支持以下可选变量，用于为容器生成 Zenoh 会话文件：

```text
ROBONIX_ZENOH_ROUTER
ROBONIX_ZENOH_MODE
ROBONIX_ZENOH_LISTEN
```

它们不是第二个 RMW 选择器。`RMW_IMPLEMENTATION` 选择 ROS 2 中间件；上述变量只描述 `rmw_zenoh_cpp` 如何连接路由器。软件包入口脚本最终将生成的文件写入 `ZENOH_SESSION_CONFIG_URI`。

路由器位于宿主机或另一台固定节点、而 ROS 2 package 在容器中运行时，可在 deployment 顶层统一提供会话参数：

```yaml
env:
  RMW_IMPLEMENTATION: rmw_zenoh_cpp
  ROBONIX_ZENOH_MODE: client
  ROBONIX_ZENOH_ROUTER: tcp/192.168.1.10:7447
```

只有明确支持这些变量的软件包入口脚本才会据此生成会话文件；其他容器必须在自己的 `start:` 实现中把等价配置写入容器并设置 `ZENOH_SESSION_CONFIG_URI`。先从容器内确认路由器地址和 TCP `7447` 可达，再检查 ROS 2 topic/service 发现。

## 验证目标选择

```bash
# 构建部署选中的全部软件包清单
rbnx build -f robonix_manifest.yaml

# 启动并检查提供方状态
rbnx boot -f robonix_manifest.yaml
rbnx caps -v

# 检查目标软件包的构建和启动日志
rg -n "RBNX_BUILD_TARGET|mode=|ERROR|FAIL" rbnx-boot/logs
```

验收时至少确认：实际架构正确、软件包使用了预期清单、ROS 2 参与者能互相发现，以及设备、GPU 和持久化目录在运行环境中可访问。Atlas 生命周期状态应按类别检查：已完成启动的常驻原语和服务应为 `ACTIVE`；技能在启动后可以保持 `INACTIVE`，首次调用并由 Executor 激活后再检查 `ACTIVE`。提供方暴露健康接口时还应调用该接口；没有健康接口时，以业务接口和软件包日志验证运行状态。
