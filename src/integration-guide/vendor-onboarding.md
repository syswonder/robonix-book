# 机器人本体接入指南

[toc]

本章面向负责一台真实机器人或仿真本体的集成工程师。完成后，你将得到一个可独立克隆、构建和启动的 **robot deployment 仓库**：仓库内包含整机描述、硬件实例、机器人专属配置，以及对社区 package 的引用。

<div class="procedure-meta">
  <div><strong>对齐分支</strong><code>dev-next</code></div>
  <div><strong>参考实现</strong>AgileX Ranger Mini v3</div>
  <div><strong>交付目标</strong>从空目录到可验收部署</div>
</div>

> **版本说明**　本文命令和字段与 Robonix `dev-next` 对齐。该分支用于整机集成验证，接口仍可能调整；需要较稳定开发基线时优先使用 `dev`，`main` 仅保存重大版本定档。开始接入前先记录所用 Robonix commit，避免不同机器实际运行不同接口。

## 交付物

一个完整的本体接入至少包含下列内容：

```text
robot-<vendor>-<model>/
├── robonix_manifest.yaml          # 整机启动入口
├── soma.yaml                      # 本体、部件和 provider 关系
├── urdf/
│   └── <robot>.urdf               # 一棵完整的整机坐标树
├── config/
│   ├── rtabmap_params.yaml        # 本体自有的建图参数
│   ├── nav2_params.yaml           # 本体自有的导航参数
│   └── navigate.xml               # 可选：自定义 Nav2 BT
├── assets/
│   └── robot.jpg                  # Catalog 使用的机器人预览图
├── build.sh                       # 可选：整机便捷构建入口
└── start.sh                       # 可选：CAN、RMW router 等宿主准备后再 boot
```

硬件驱动通常放在独立 Primitive 仓库中，由 `robonix_manifest.yaml` 的 `url` 引用；只有暂不复用的部署私有代码才放在本体仓库。可参考当前可运行的 [`robot-agilex-ranger_mini_v3`](https://github.com/syswonder/robot-agilex-ranger_mini_v3) 和用于学习包结构的 [`template-rbnx`](https://github.com/syswonder/template-rbnx)。

## 1. 建立可追溯的开发环境

先按[快速开始](../getting-started/quickstart.md)安装 Rust、`uv`、Docker 和 Robonix。接入工作必须在所有目标机上使用同一 Robonix revision：

```bash
git clone --branch dev-next --recurse-submodules \
  https://github.com/syswonder/robonix.git
cd robonix
make install
rbnx setup "$PWD"
rbnx --version
git rev-parse HEAD
```

`rbnx setup` 记录 Robonix 源码根目录，使部署中的 `${ROBONIX_SOURCE_PATH}`、标准 contract 和 codegen 输入可以从任意工作目录解析。把最后一条命令的 commit 写进联调记录。

在仓库外创建部署骨架：

```bash
cd ..
rbnx init robot-acme-rover
cd robot-acme-rover
```

`rbnx init` 会创建 `robonix_manifest.yaml` 以及空的 `primitives/`、`services/`、`skills/` 目录。随后补充 `soma.yaml`、`urdf/`、`config/` 和 `assets/robot.jpg`。

<div class="expected-result">
  <strong>完成标志</strong>
  <p><code>robonix_manifest.yaml</code> 的 <code>catalog</code> 已填写 name、version、description、license、tags 和 maintainers；团队能指出当前使用的 Robonix commit。</p>
</div>

## 2. 盘点本体和运行目标

在写 manifest 前先确定四组事实，并把结果保存在部署仓库 README：

| 项目 | 必须确认的内容 |
|---|---|
| 计算平台 | x86_64 或 aarch64；native 或 Docker；操作系统、ROS 2、JetPack/CUDA 版本 |
| 硬件 | 底盘、机械臂、夹爪、相机、雷达、IMU、麦克风和扬声器的型号与连接方式 |
| 坐标系 | `base_link`、`odom`、所有传感器 optical/frame link，以及唯一的 TF 发布者 |
| 安全边界 | 急停、速度/关节限制、CAN/串口权限、看门狗和无控制输入时的停止行为 |

真实机器人第一次运动测试前，应能用硬件急停独立停止底盘和机械臂。底盘先架空或限制在清空区域，机械臂先关闭运动能力或使用安全姿态。文档验收不能代替设备厂商的安全要求。

## 3. 编写完整 URDF 与 Soma 描述

### 3.1 一台机器人只维护一棵完整 URDF

`soma.yaml` 的 `urdf.path` 指向整机 URDF。底盘、传感器和机械臂之间的固定变换都必须出现在这棵树中；Mapping、Navigation、Scene 和 robot-description Primitive 依赖同一坐标关系。不要分别维护互相重叠的“底盘 URDF”和“相机 URDF”并同时发布 TF。

如果仿真器或厂商程序已经发布 `/robot_description`、`/tf` 和 `/tf_static`，不要再启动第二个 TF 发布者。没有现成发布者时，在部署中引用通用 [`primitive-robot-description-rbnx`](https://github.com/syswonder/primitive-robot-description-rbnx)；它从 Soma 读取 URDF并发布描述与坐标变换。

### 3.2 创建 `soma.yaml`

Soma 描述的是机器人本体与部件，不负责环境物体或健康判定。最小结构如下；完整字段说明以 [`system/soma/README.md`](https://github.com/syswonder/robonix/blob/dev-next/system/soma/README.md) 为准。

```yaml
urdf:
  path: ./urdf/acme_rover.urdf
  root_link: base_link
  model_name: acme_rover

robot:
  id: acme_rover_01
  display_name: "ACME Rover"
  family: mobile_robot
  root_part: base
  dimensions: { length_m: 0.72, width_m: 0.50, height_m: 0.82 }
  footprint:
    base_frame: base_link
    points: [[0.36, 0.25], [0.36, -0.25], [-0.36, -0.25], [-0.36, 0.25]]
  mass_kg: 32.0
  exports:
    - provider_id: nav2
      capabilities:
        - { path: robonix/service/navigation/navigate, description: "Navigate to a 2D goal." }
  components:
    - id: base
      type: mobile_base
      urdf_link: base_link
      exports:
        - provider_id: base_chassis
          capabilities:
            - { path: robonix/primitive/chassis/move, description: "Command chassis motion." }
            - { path: robonix/primitive/chassis/odom, description: "Read chassis odometry." }
    - id: front_camera
      type: rgbd_camera
      urdf_link: front_camera_link
      exports:
        - provider_id: front_camera
          capabilities:
            - { path: robonix/primitive/camera/rgb, description: "Stream RGB images." }
            - { path: robonix/primitive/camera/depth, description: "Stream aligned depth images." }

description:
  summary: "Mobile robot with an RGB-D camera."
  can_do: ["drive", "navigate", "capture RGB-D images"]
  cannot_do: ["manipulate objects"]
```

`provider_id` 必须等于部署清单中对应 package instance 的 `name`。`components` 是面向系统和模型的逻辑部件树；实际几何变换仍由 URDF 决定，不需要把 URDF 每个 joint 都重复一遍。

带机械臂和夹爪时，机械臂 provider 应暴露标准 `robonix/primitive/arm/joint_states`。夹爪的开启位置可以在其 component 上标定：

```yaml
- id: arm
  type: manipulator
  urdf_link: arm_base_link
  exports:
    - provider_id: arm_controller
      capabilities:
        - { path: robonix/primitive/arm/joint_states, description: "Read arm and gripper joints." }
  components:
    - id: gripper
      type: parallel_jaw_gripper
      state:
        joint_name: gripper
        open_position_m: 0.080
        open_tolerance_m: 0.003
      exports: []
```

`joint_name` 必须与 JointState 反馈一致；`open_position_m` 和容差应在空夹爪完全张开时实测。Soma据此报告运动和夹爪状态，不依赖抓取 Skill。

在部署清单中绑定这份文件：

```yaml
system:
  soma:
    listen: 127.0.0.1:50091
    robot_yaml: ${ROBONIX_DEPLOY_DIR}/soma.yaml
```

`${ROBONIX_DEPLOY_DIR}` 由 `rbnx boot` 指向部署清单所在目录，避免依赖操作者当前工作目录。

## 4. 为硬件实现标准 Primitive

### 4.1 先选择标准接口

不要先从厂商 SDK 的函数名设计 Robonix 接口。先在[接口目录](../interface-catalog/index.md)选择本体需要实现的标准 contract，再把 SDK 适配到该 contract。常见对应关系如下：

| 硬件 | 接口目录 | 常用输入/输出 |
|---|---|---|
| 移动底盘 | [Chassis](../interface-catalog/primitive/chassis.md) | move、twist input、odometry |
| RGB/RGB-D 相机 | [Camera](../interface-catalog/primitive/camera.md) | RGB、depth、snapshot、intrinsics/extrinsics |
| 2D/3D 雷达 | [Lidar](../interface-catalog/primitive/lidar.md) | LaserScan、PointCloud2、snapshot |
| IMU | [IMU](../interface-catalog/primitive/imu.md) | angular velocity、linear acceleration、orientation |
| 机械臂/夹爪 | [Arm](../interface-catalog/primitive/arm.md) | joint command/state、gripper command/state |
| 音频设备 | [Audio](../interface-catalog/primitive/audio.md) | mic、speaker、device selection |
| URDF/TF | [Robot description](../interface-catalog/primitive/robot-description.md) | robot description、TF |
| 设备健康 | [Health](../interface-catalog/primitive/health.md) | provider health sample |

`package_manifest.yaml` 的 `capabilities` 只声明这个 package 实现的标准 contract。接口字段以[自动生成的 Contract/IDL 参考](../reference/index.md)为准。

### 4.2 创建或复用 Primitive package

Catalog 中已有匹配 package 时直接引用，不复制源码。没有时，从 `template-rbnx/primitives/mock_chassis` 或命令生成骨架：

```bash
rbnx package-new acme_chassis --type primitive
```

新 package 的根目录至少包含：

```text
acme_chassis/
├── package_manifest.yaml
├── config.spec
├── CAPABILITY.md             # 需要向 Pilot 提供使用说明时添加
├── scripts/
│   ├── build.sh
│   └── start.sh
└── acme_chassis/
    ├── __init__.py
    └── main.py
```

`package_manifest.yaml` 中的发布元数据、构建入口、启动入口和标准能力必须与实现一致：

```yaml
manifestVersion: 1
package:
  name: robonix.primitive.acme.rover.chassis
  version: 0.1.0
  description: ACME Rover chassis driver.
  tags: [primitive, chassis, acme, rover]
  maintainers:
    - Your Name <you@example.com>
  license: Apache-2.0

build: bash scripts/build.sh
start: bash scripts/start.sh

capabilities:
  - name: robonix/primitive/chassis/driver
  - name: robonix/primitive/chassis/move
  - name: robonix/primitive/chassis/twist_in
  - name: robonix/primitive/chassis/odom
```

`config.spec` 是部署参数的人类/模型可读说明，不是由 Robonix 运行时解析的 schema。它必须解释每个公开配置的类型、单位、默认值、允许范围、硬件影响和示例；provider 代码仍负责实际校验。社区维护的硬件 package 应提供该文件，避免参数只隐藏在源代码里。

```yaml
config:
  # string, default: can0.
  # SocketCAN interface used by the chassis SDK. The interface must already
  # exist and be UP before CMD_INIT.
  can_port: can0

  # float, seconds, default: 30.0; must be > 0.
  # Maximum wait for the first valid hardware feedback frame during CMD_INIT.
  sentinel_timeout_s: 30.0
```

`CAPABILITY.md` 用于说明模型何时、如何使用 provider。存在时，Robonix API 会在注册时读取内容并交给 Atlas；Pilot 通过 provider id 按需读取，而不是猜本地路径。没有额外使用语义的简单数据流 Primitive 可以不写。

### 4.3 厂商 SDK 与 ROS 2

Primitive 可以直接调用 C/C++/Python SDK，也可以包装已有 ROS 2 driver。包应拥有它启动的厂商进程，并在 `CMD_SHUTDOWN` 中可靠停止；不要要求部署者另外手工启动一份同名 driver。

如果 package 使用 ROS 2：

- 所有相互通信的进程必须使用一致的 `RMW_IMPLEMENTATION`；
- Primitive 实现标准 ROS topic/service contract 即可，不需要调用 Zenoh 专用 API；
- `rbnx boot` 会把部署清单的 `env:` 传给子进程，但 Docker 的 `start.sh` 仍需显式把相关环境传入容器；
- 整个 ROS graph 只启动一个预期的 `rmw_zenohd`，router 的所有权由整机部署的 `start.sh` 或仿真容器确定。

真实机器人常用配置为：

```yaml
env:
  ROS_DOMAIN_ID: "0"
  RMW_IMPLEMENTATION: rmw_zenoh_cpp
```

只需要标准 ROS 变量；不要再为同一选择维护第二个 Robonix 私有 RMW 变量。Zenoh 安装、router 与容器网络见[多平台部署](../architecture/multiplatform-deployment.md)。

### 4.4 验证单个 package

在 package 根目录的父目录执行：

```bash
rbnx validate ./acme_chassis
rbnx build -p ./acme_chassis
rbnx start -p ./acme_chassis --endpoint 127.0.0.1:50051 \
  --set can_port=can0
```

`rbnx validate` 验证 manifest 结构；`build` 执行 package 的构建入口；`start` 需要已经运行的 Atlas。开发细节见[开发者手册](../developer-guide.md)。

## 5. 组装 `robonix_manifest.yaml`

每个列表条目代表一个运行实例。`name` 同时是期望注册的 provider id 和日志标签；`url`/`path` 选择 package；`manifest` 选择该 package 为目标平台提供的构建/启动变体；`config` 原样传给 provider 的 `CMD_INIT`。

```yaml
manifestVersion: 1
name: robonix-acme-rover

catalog:
  name: robonix.robot.acme.rover
  version: 0.1.0
  description: Robonix deployment for ACME Rover.
  license: Apache-2.0
  tags: [robot, deploy, acme, rover]
  maintainers:
    - Your Name <you@example.com>

env:
  ROS_DOMAIN_ID: "0"
  RMW_IMPLEMENTATION: rmw_zenoh_cpp

system:
  atlas:
    listen: 0.0.0.0:50051
  soma:
    listen: 127.0.0.1:50091
    robot_yaml: ${ROBONIX_DEPLOY_DIR}/soma.yaml
  vitals:
    listen: 0.0.0.0:50093
  executor:
    listen: 0.0.0.0:50061
  pilot:
    listen: 127.0.0.1:50071
    vlm:
      upstream: ${VLM_BASE_URL}
      api_key: ${VLM_API_KEY}
      model: ${VLM_MODEL}
  liaison:
    listen: 0.0.0.0:50081
  scene:
    camera_provider_id: front_camera

primitive:
  - name: robot_description
    url: https://github.com/syswonder/primitive-robot-description-rbnx
    branch: main
    manifest: package_manifest.native.yaml

  - name: base_chassis
    url: https://github.com/acme/primitive-acme-rover-chassis-rbnx
    branch: main
    config:
      can_port: can0

  - name: front_camera
    url: https://github.com/acme/primitive-acme-rgbd-camera-rbnx
    branch: main
    config: {}
```

对外连接只开放实际需要的端口，并限制在可信局域网或 Tailscale。Pilot 通常保持 loopback；Client 连接 Atlas、Executor、Liaison、Vitals 和音频 bridge，因此相应服务按部署网络边界监听。不要把 API key 写进 manifest；在启动 shell 中导出环境变量。

同一 package 可以同时提供 `package_manifest.yaml`、`package_manifest.jetson-native.yaml`、`package_manifest.jetson-docker.yaml` 等变体。整机条目的 `manifest:` 明确选择其中一个；不存在的文件会直接报错，不会静默回退。各变体共享标准 contract，但可以使用不同的系统依赖、构建脚本和启动方式。

## 6. 配置 Mapping、Navigation 与 Scene

算法 package 提供实现和配置模板，**机器人参数归本体部署仓库所有**。不要把新机器人 profile 加入 Mapping/Navigation 上游代码。

### 6.1 Mapping

从 [`service-map-rbnx/config/rtabmap_params.template.yaml`](https://github.com/syswonder/service-map-rbnx/blob/main/config/rtabmap_params.template.yaml) 复制一份到本体仓库 `config/rtabmap_params.yaml`，然后基于传感器位置、频率、噪声、机器人 footprint 和计算平台调参。上游 template 只供复制，运行时不会自动加载。

```yaml
service:
  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config:
      params_file: config/rtabmap_params.yaml
      occupancy_sources: [lidar]
      sensor_providers:
        lidar3d: roof_lidar
        odom: base_chassis
        rgb: front_camera
        depth: front_camera
```

`sensor_providers` 的值是 Atlas provider id，不是 topic 名。支持角色为 `lidar2d`、`lidar3d`、`rgb`、`depth`、`imu` 和 `odom`；RGB-D 必须同时指定 `rgb` 与 `depth`。`rtabmap_params` 可以在 manifest 中对文件做少量最终覆盖，但完整基线仍应保存在部署仓库。字段以 Mapping 仓库根目录的 `config.spec` 为准。

旧 `rtabmap_profile` 和 `sensors` 仍可用于迁移并产生 warning；新部署不得依赖它们。

### 6.2 Navigation

从 Navigation 上游的模板复制完整 Nav2 配置到本体仓库 `config/nav2_params.yaml`，将 footprint、速度/加速度、costmap、controller、goal checker 和行为树参数与真实底盘对齐：

```yaml
  - name: nav2
    url: https://github.com/syswonder/service-navigation-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config:
      params_file: config/nav2_params.yaml
      bt_xml_file: config/navigate.xml
      provider_ids:
        map: mapping
        odom: base_chassis
        scan: front_lidar
```

3D 雷达需要投影时使用 `provider_ids.scan_cloud` 并配置 `scan_projection`；原生 2D LaserScan 使用 `scan`。旧 `params_profile` 只用于兼容并产生 warning。完整字段和安全 guard 以 [`service-navigation-rbnx/config.spec`](https://github.com/syswonder/service-navigation-rbnx/blob/main/config.spec) 为准；Nav2 参数语义参考 [Nav2 配置指南](https://docs.nav2.org/configuration/index.html)。

### 6.3 Scene

机器人有多个相机时，在 `system.scene.camera_provider_id` 选择 Scene 使用的相机。用于 RGB-D Scene 的 provider 必须同时暴露标准 `camera/rgb` 和 `camera/depth` 能力。

Scene 的机器人 pose 来自 Atlas 解析到的 `robonix/service/map/pose`，即经过 SLAM 修正的 map-frame 位姿，不直接读取 chassis odom。RGB-D 几何还需要相机 provider 的 `camera/intrinsics` 与 `camera/extrinsics`；相机 Primitive 应从标定和完整 URDF/TF 树提供真实外参。Scene 仅为旧部署保留 tf2 fallback，新的本体不能依赖该回退。

因此新本体必须同时满足：Mapping 绑定了正确的 odom/传感器 provider，Mapping 正常发布 `map/pose`，相机 provider 同时声明 RGB、depth、intrinsics、extrinsics，URDF 可以连接所有 frame，系统只有一套权威 TF。

## 7. 按风险分阶段验收

不要第一次就把全部驱动和运动服务一起启动。每一阶段通过后再加入下一组 package。

### 7.1 静态检查与构建

```bash
rbnx build -f robonix_manifest.yaml
```

构建必须无失败。Jetson native package 使用宿主已安装且与 JetPack 匹配的 CUDA、PyTorch、TorchVision/TorchAudio 等系统组件；有些开发套件出厂已带这些组件。先核对 JetPack/L4T 与实际 Python wheel，再参考 [Jetson AI Lab package index](https://pypi.jetson-ai-lab.io/)；不要照抄另一块板子的 CUDA index。包自身携带的厂商 SDK/ROS driver 由 package 构建脚本处理。

### 7.2 无运动启动

暂时移除或禁用底盘/机械臂运动 package，只启动 Atlas、Soma、Vitals、robot-description 和只读传感器：

```bash
rbnx boot -f robonix_manifest.yaml
rbnx caps -v
```

检查每个 provider 为 `ACTIVE`、provider id 与 manifest `name` 一致、URDF/TF 连通、传感器 frame/timestamp 正确。失败时查看：

```bash
rbnx logs -t <provider_id>
```

### 7.3 单设备运动

先在硬件安全条件下验证底盘低速前进/停止、机械臂单关节/夹爪开合和急停；再验证 Navigation。任何 provider 在输入超时、进程退出或 `CMD_SHUTDOWN` 时都必须停止其硬件输出。

### 7.4 Mapping、Navigation 与交互

依次验证：

1. 静止时点云/scan 与墙面重合，旋转时 TF 和定位不漂移；
2. 建图、保存、重启、加载同一 map id 后 occupancy 与 Scene 标记恢复；
3. 已知区域内规划、避障、到点停止和 cancel；
4. Client 文本任务、相机理解、语音输入/输出与 stop；
5. Vitals 能区分本体状态、provider 状态和故障。

地图文件和联调环境属于测试数据。验收后只删除本次创建的临时 map id，不要清理操作者已有地图目录。

<div class="expected-result">
  <strong>整机完成标志</strong>
  <p>一台新机器克隆 deployment 仓库、设置文档列出的环境变量并运行 build/boot 后，可以复现相同 provider 列表、Soma/TF、传感器、地图、导航与 Client 链路；不依赖未记录的绝对路径或手工后台进程。</p>
</div>

## 8. 发布和持续维护

Primitive、Service、Skill 各自提交到独立社区仓库；robot deployment 仓库只组装它们并保存机器人专属配置。提交格式和 Catalog 流程见 [Package Catalog 发布流程](package-catalog.md)。

发布前至少确认：

- package manifest 与 robot catalog metadata 完整，license 为有效 SPDX 标识；
- 每个公开 `config` 字段在根目录 `config.spec` 中有解释；
- README 写明系统依赖、硬件连接、构建/启动、权限和安全限制；
- package 在它声明支持的 target 上通过 `rbnx validate`、build、start、shutdown；
- deployment 的 `assets/robot.jpg` 已压缩到网页适合尺寸；
- 维护者、默认分支与 Catalog 记录一致。

新设备接入后的参数变化优先提交到对应 robot deployment；可复用的驱动修复提交到 Primitive 仓库；标准接口变更才进入 Robonix 主仓库。这样其他本体可以升级 package，而不会继承某台机器的私有参数。
