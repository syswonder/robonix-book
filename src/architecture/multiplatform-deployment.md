# 多架构与多运行环境部署

[toc]

Robonix 的 deployment manifest 可以为每个 package 实例选择不同的 package manifest。package 作者据此提供 x86_64、Jetson arm64、native 或 container 等构建与启动实现。

这不是一张由 Robonix 强制定义的固定 target 矩阵。一个 package 只支持它实际提供并测试过的 manifest 文件。

## 选择 package manifest

默认情况下，`rbnx build` 和 `rbnx start` 读取：

```text
<package>/package_manifest.yaml
```

部署项可用 `manifest:` 选择同一 package 中的另一个文件：

```yaml
service:
  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config:
      params_file: config/rtabmap_params.yaml
      sensor_providers:
        lidar3d: roof_lidar
        odom: base_chassis
```

`rbnx build -f robonix_manifest.yaml` 使用该文件的 `build:`；`rbnx boot` 内部执行 `rbnx start --manifest <file>`，使用同一文件的 `start:` 和 `stop:`。文件不存在时直接失败，不会静默回退。

常见文件名是约定，不是 CLI 枚举：

| 文件名示例 | 通常表示 |
|---|---|
| `package_manifest.yaml` | package 默认目标 |
| `package_manifest.jetson-native.yaml` | Jetson 宿主机原生运行 |
| `package_manifest.jetson-docker.yaml` | Jetson 的 L4T/NVIDIA 容器 |

package 也可以定义其它名称。部署仓库必须只引用该 package 实际存在的文件。

## 一个目标 manifest 应包含什么

不同目标 manifest 应保持相同的 package 身份与 Capability surface，只改变构建和运行实现：

```yaml
manifestVersion: 1

package:
  name: robonix.service.navigation
  version: 0.1.0
  description: Nav2 service wrapper.
  license: Apache-2.0

build: RBNX_BUILD_TARGET=jetson-native bash scripts/build.sh
start: ROBONIX_NAV2_FORCE=native bash scripts/start.sh
stop: ROBONIX_NAV2_FORCE=native bash scripts/stop.sh

capabilities:
  - name: robonix/service/navigation/driver
  - name: robonix/service/navigation/navigate
  - name: robonix/service/navigation/navigate/status
  - name: robonix/service/navigation/navigate/cancel
```

`RBNX_BUILD_TARGET`、`ROBONIX_NAV2_FORCE` 等变量是 package 脚本自己的实现约定。`rbnx` 只执行 manifest 中的 shell 命令，不解释这些变量。

## Native 与 container 的边界

### Native package

Native package 直接使用宿主机的系统库、ROS 2、设备文件和 GPU runtime。它的目标 manifest 应在 `build:` 中验证必要依赖，在 `start:` 中 source 正确的 ROS 2 或 colcon overlay。

适合 native 的情况包括：

- Jetson 已预装与 JetPack 匹配的 CUDA、TensorRT、PyTorch 等系统 package。
- 硬件 SDK 或 ROS driver 已由 package 自带构建脚本安装或编译。
- 需要直接访问 SocketCAN、USB、串口或宿主机 ROS 2 graph。

不要假定另一台 Jetson 与当前机器具有相同 JetPack、CUDA 或 Python wheel。每个 package 的 README 和目标 manifest 是前置条件的权威来源。

### Container package

Container package 由 `build:` 构建镜像，由 `start:` 创建容器。package 脚本必须显式处理：

- CPU 架构与基础镜像。
- GPU runtime 和设备映射。
- ROS 2 环境与 RMW 实现。
- Atlas endpoint 的可达地址。
- 配置、模型缓存和持久化数据的 mount。
- 信号处理和容器清理。

`rbnx` 不会自动把 ROS 2、CUDA、设备或环境变量注入任意容器；这些属于 package 的启动实现。

## 当前仓库中可验证的例子

| Package | 默认 manifest | 额外 manifest |
|---|---|---|
| Scene | `package_manifest.yaml`，x86 Docker | `package_manifest.jetson-native.yaml`、`package_manifest.jetson-docker.yaml` |
| Mapping | `package_manifest.yaml`，x86 Docker | `package_manifest.jetson-native.yaml`、`package_manifest.jetson-docker.yaml` |
| Navigation | `package_manifest.yaml`，x86 Docker | `package_manifest.jetson-native.yaml`、`package_manifest.jetson-docker.yaml` |

这张表只描述当前文件。不能据此推断 Memory、Speech、某个 Primitive 或第三方 Skill 也支持相同目标。接入时应先检查 package 根目录：

```bash
find /path/to/package -maxdepth 1 -name 'package_manifest*.yaml' -print
```

## Robot-specific 配置属于部署仓库

目标 manifest 决定“如何构建和运行 package”；部署项的 `config:` 决定“这个机器人实例如何配置”。两者不要混合。

例如 Mapping 和 Navigation 的机器人参数文件应位于机器人部署仓库：

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
    manifest: package_manifest.jetson-native.yaml
    config:
      params_file: config/rtabmap_params.yaml
      sensor_providers:
        lidar3d: roof_lidar
        rgb: front_camera
        depth: front_camera
        odom: base_chassis

  - name: nav2
    url: https://github.com/syswonder/service-navigation-rbnx
    manifest: package_manifest.jetson-native.yaml
    config:
      params_file: config/nav2_params.yaml
      provider_ids:
        map: mapping
        odom: base_chassis
        scan: roof_lidar
```

`params_file` 相对于 robot deployment manifest 所在目录解析。上游仓库中的 `*.template.yaml` 只用于复制参考，不在运行时自动加载。Mapping 的旧 `rtabmap_profile`、Navigation 的旧 `params_profile` 仍为迁移兼容入口；当前实现会给出迁移 warning，新部署不要使用。

## ROS 2、RMW 与 Zenoh

Primitive 和 Service 继续编写标准 ROS 2 publisher、subscriber、service 或 action 代码，不需要调用 Zenoh 专用 API。实际传输由 ROS 2 的 RMW 层选择。

同一 ROS 2 graph 中的进程必须使用一致的实现，例如：

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
rbnx boot -f robonix_manifest.yaml
```

`rbnx boot` 启动的 native 子进程继承该环境。container package 的 `start.sh` 必须把 `RMW_IMPLEMENTATION` 转发进容器。

使用 `rmw_zenoh_cpp` 时，还需要一个所有参与者都能到达的 `rmw_zenohd` router。Webots 示例在仿真容器中启动 router。真实机器人可以在宿主机或网络中的固定节点启动，但同一 graph 不应由每个 package 各自盲目启动一份 router。

部分 package 支持以下可选变量，用于为容器生成 Zenoh session 文件：

```text
ROBONIX_ZENOH_ROUTER
ROBONIX_ZENOH_MODE
ROBONIX_ZENOH_LISTEN
```

它们不是第二个 RMW 选择器。`RMW_IMPLEMENTATION` 选择 ROS 2 middleware；上述变量只描述 `rmw_zenoh_cpp` 如何连接 router。package entrypoint 最终将生成的文件写入 `ZENOH_SESSION_CONFIG_URI`。

## 验证目标选择

```bash
# 构建部署选中的全部 package manifest
rbnx build -f robonix_manifest.yaml

# 启动并检查 provider 状态
rbnx boot -f robonix_manifest.yaml
rbnx caps -v

# 检查目标 package 的构建和启动日志
rg -n "RBNX_BUILD_TARGET|mode=|ERROR|FAIL" rbnx-boot/logs
```

验收时至少确认：实际架构正确、package 使用了预期 manifest、ROS 2 participant 能互相发现、Atlas 中 provider 为 `ACTIVE`，以及设备/GPU/持久化目录在运行环境中可访问。
