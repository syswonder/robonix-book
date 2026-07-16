# 机器人本体接入指南


本章面向负责一台真实机器人或仿真本体的集成工程师。完成后，你将得到一个可独立克隆、构建和启动的**机器人部署仓库（Robot Deployment）**：仓库内包含整机描述、硬件实例、机器人专属配置，以及对社区软件包的引用。

<div class="procedure-meta">
  <div><strong>源码仓库</strong><code>syswonder/robonix</code></div>
  <div><strong>参考部署</strong>AgileX Ranger Mini v3</div>
  <div><strong>交付目标</strong>从空目录到可验收部署</div>
</div>

## 交付物

一个完整的本体接入至少包含下列内容：

```text
robot-<vendor>-<model>/
├── robonix_manifest.yaml          # 整机启动入口
├── soma.yaml                      # 本体、部件和提供方关系
├── urdf/
│   └── <robot>.urdf               # 一棵完整的整机坐标树
├── config/
│   ├── rtabmap_params.yaml        # 本体自有的建图参数
│   ├── nav2_params.yaml           # 本体自有的导航参数
│   └── navigate.xml               # 可选：自定义 Nav2 BT
├── assets/
│   └── robot.jpg                  # 软件包目录使用的机器人预览图
├── build.sh                       # 可选：整机便捷构建入口
└── start.sh                       # 可选：CAN、RMW router 等宿主准备后再 boot
```

硬件驱动通常放在独立原语仓库中，由 `robonix_manifest.yaml` 的 `url` 引用；只有暂不复用的部署私有代码才放在本体仓库。可参考 [`robot-agilex-ranger_mini_v3`](https://github.com/syswonder/robot-agilex-ranger_mini_v3) 的部署编排，以及用于学习软件包结构的 [`template-rbnx`](https://github.com/syswonder/template-rbnx/tree/60dc85834c2714022b1821e6fce6c629c0314699)。采用任何参考部署前，都要核对其整机 URDF 是否包含当前机器人实际安装的全部部件；缺少的坐标关系必须先补齐，不能依赖另一份分离 URDF。

AgileX 仓库依赖包装脚本设置部署目录和宿主环境，构建与启动应使用：

```bash
bash build.sh
bash start.sh
```

不要绕过包装脚本直接执行 `rbnx build -f` 或 `rbnx boot -f`。

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

`rbnx setup` 记录 Robonix 源码根目录，使部署中的 `${ROBONIX_SOURCE_PATH}`、标准能力约定和代码生成输入可以从任意工作目录解析。把最后一条命令输出的提交号写进联调记录。

在仓库外创建部署骨架：

```bash
cd ..
rbnx init robot-acme-rover
cd robot-acme-rover
```

`rbnx init` 当前只创建部署目录、`robonix_manifest.yaml` 和 `.gitignore`。随后建立本体需要的目录：

```bash
mkdir -p primitives services skills config urdf assets
```

再补充 `soma.yaml`、完整 URDF、机器人专属配置和 `assets/robot.jpg`。

**完成标志：** `robonix_manifest.yaml` 的 `catalog` 已填写 `name`、`version`、`description`、`license`、`tags` 和 `maintainers`；团队能指出当前使用的 Robonix commit。

## 2. 盘点本体和运行目标

在写部署清单前先确定四组事实，并把结果保存在部署仓库 README：

| 项目 | 必须确认的内容 |
|---|---|
| 计算平台 | x86_64 或 aarch64；native 或 Docker；操作系统、ROS 2、JetPack/CUDA 版本 |
| 硬件 | 底盘、机械臂、夹爪、相机、雷达、IMU、麦克风和扬声器的型号与连接方式 |
| 坐标系 | `base_link`、`odom`、所有传感器光学坐标系和设备坐标系，以及唯一的 TF 发布方 |
| 安全边界 | 急停、速度/关节限制、CAN/串口权限、看门狗和无控制输入时的停止行为 |

:::danger 首次运动测试

真实机器人第一次运动测试前，必须确认硬件急停能够独立停止底盘和机械臂。底盘应先架空或限制在清空区域，机械臂应先关闭运动能力或使用安全姿态。文档验收不能代替设备厂商的安全要求。

:::

## 3. 编写完整 URDF 与 Soma 描述

### 3.1 一台机器人只维护一棵完整 URDF

`soma.yaml` 的 `urdf.path` 指向整机 URDF。底盘、传感器和机械臂之间的固定变换都必须出现在这棵树中；建图、导航、Scene 和机器人描述原语依赖同一坐标关系。不要分别维护互相重叠的“底盘 URDF”和“相机 URDF”并同时发布 TF。

如果仿真器或厂商程序已经发布 `/robot_description`、`/tf` 和 `/tf_static`，不要再启动第二个 TF 发布方。没有现成发布方时，在部署中引用通用 [`primitive-robot-description-rbnx`](https://github.com/syswonder/primitive-robot-description-rbnx)；它从 Soma 读取 URDF，并发布描述与坐标变换。

AgileX Ranger Mini v3 仓库当前只能作为部署连线的部分参考，不能用来证明底盘与机械臂已经合并为一棵完整 URDF；新机器人仍须按本节完成整机坐标树。

### 3.2 创建 `soma.yaml`

Soma 描述的是机器人本体与部件，不负责环境物体或健康判定。最小结构如下；完整字段说明以当前源码的 [`system/soma/README.md`](https://github.com/syswonder/robonix/blob/436204a0aa5301dcf682a38ad29523023ad5a3a5/system/soma/README.md) 为准。

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

`provider_id` 必须等于部署清单中对应软件包运行实例的 `name`。`components` 是面向系统和模型的逻辑部件树；实际几何变换仍由 URDF 决定，不需要把 URDF 的每个关节都重复一遍。

带机械臂和夹爪时，机械臂提供方应暴露标准 `robonix/primitive/arm/joint_states`。夹爪的开启位置可以在其部件上标定：

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

`joint_name` 必须与 JointState 反馈一致；`open_position_m` 和容差应在空夹爪完全张开时实测。Soma 据此报告运动和夹爪状态，不依赖抓取技能。

在部署清单中绑定这份文件：

```yaml
system:
  soma:
    listen: 127.0.0.1:50091
    robot_yaml: soma.yaml
```

`robot_yaml` 的相对路径以部署清单所在目录为基准。若同目录存在 `soma.yaml`，也可以省略 `robot_yaml`，启动器会自动注入该文件。

## 4. 为硬件实现标准原语

### 4.1 先选择标准接口

不要先从厂商 SDK 的函数名设计 Robonix 接口。先在[接口目录](../interface-catalog/index.md)选择本体需要实现的标准能力约定，再把 SDK 适配到该能力约定。常见对应关系如下：

| 硬件 | 接口目录 | 常用输入/输出 |
|---|---|---|
| 移动底盘 | [底盘](../interface-catalog/primitive/chassis.md) | `move`、`twist_in`、`odom` |
| RGB/RGB-D 相机 | [相机](../interface-catalog/primitive/camera.md) | 彩色图、深度图、快照、内参；外参优先来自完整 URDF 发布的 TF |
| 二维/三维激光雷达 | [激光雷达](../interface-catalog/primitive/lidar.md) | `LaserScan`、`PointCloud2`、`snapshot` |
| 惯性测量单元（IMU） | [惯性测量单元](../interface-catalog/primitive/imu.md) | 角速度、线加速度、姿态 |
| 机械臂/夹爪 | [机械臂](../interface-catalog/primitive/arm.md) | 关节命令与状态、夹爪命令与状态 |
| 音频设备 | [音频](../interface-catalog/primitive/audio.md) | 麦克风、扬声器、设备选择 |
| URDF/TF | [机器人描述](../interface-catalog/primitive/robot-description.md) | `robot_description`、TF |
| 设备健康 | [设备健康](../interface-catalog/primitive/health.md) | 提供方健康样本 |

`package_manifest.yaml` 的 `capabilities` 只声明这个软件包实现的标准能力约定。接口字段以[自动生成的能力约定与 IDL 参考](../reference/index.md)为准。

### 4.2 创建或复用原语软件包

软件包目录中已有匹配软件包时直接引用，不复制源码。没有时，从 `template-rbnx/primitives/mock_chassis` 或命令生成骨架：

```bash
rbnx package-new acme_chassis --type primitive
```

新软件包的根目录至少包含：

```text
acme_chassis/
├── .gitignore
├── package_manifest.yaml
├── capabilities/
│   └── .gitkeep
├── scripts/
│   ├── build.sh
│   └── start.sh
└── acme_chassis/
    ├── __init__.py
    └── main.py
```

这是 `rbnx package-new` 实际生成的目录。生成的 `main.py` 默认把提供方 ID 写死为 `acme_chassis`；可复用软件包必须改为读取部署实例名，并为单个软件包调试保留默认值：

```python
import os

from robonix_api import Primitive

instance_name = os.environ.get("RBNX_INSTANCE_NAME", "acme_chassis")
provider = Primitive(
    id=instance_name,
    namespace="robonix/primitive/chassis",
)
```

`rbnx boot` 会把部署条目的 `name` 写入 `RBNX_INSTANCE_NAME`，因此后文的 `base_chassis` 会注册成同名提供方。开始实现后再添加 `config.spec`；只有提供方需要向 Pilot 说明调用顺序、约束或长任务语义时才添加可选的 `CAPABILITY.md`。`capabilities/` 用于放置尚未进入 Robonix 标准接口的软件包内能力约定；如果实现的都是标准接口，可以保持为空。

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

`config.spec` 是部署参数的人类和模型可读说明，不是由 Robonix 运行时解析的模式定义。它必须解释每个公开配置的类型、单位、默认值、允许范围、硬件影响和示例；提供方代码仍负责实际校验。社区维护的硬件软件包应提供该文件，避免参数只隐藏在源代码里。

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

`CAPABILITY.md` 用于说明模型何时、如何使用提供方。存在时，Robonix API 会在注册时读取内容并交给 Atlas；Pilot 通过提供方 ID 按需读取，而不是猜本地路径。没有额外使用语义的简单数据流原语可以不写。

### 4.3 厂商 SDK 与 ROS 2

原语可以直接调用 C、C++ 或 Python SDK，也可以包装已有 ROS 2 驱动。软件包应拥有它启动的厂商进程，并在 `CMD_SHUTDOWN` 中可靠停止；不要要求部署者另外手工启动一份同名驱动。

如果软件包使用 ROS 2：

- 所有相互通信的进程必须使用一致的 `RMW_IMPLEMENTATION`；
- 原语实现标准 ROS 2 话题或服务能力约定即可，不需要调用 Zenoh 专用 API；
- `rbnx boot` 会把部署清单的 `env:` 传给子进程，但 Docker 的 `start.sh` 仍需显式把相关环境传入容器；
- 整个 ROS graph 只启动一个预期的 `rmw_zenohd`，router 的所有权由整机部署的 `start.sh` 或仿真容器确定。

真实机器人常用配置为：

```yaml
env:
  ROS_DOMAIN_ID: "0"
  RMW_IMPLEMENTATION: rmw_zenoh_cpp
```

只需要标准 ROS 变量；不要再为同一选择维护第二个 Robonix 私有 RMW 变量。Zenoh 安装、路由器与容器网络见[多平台部署](../architecture/multiplatform-deployment.md)。

### 4.4 验证单个软件包

在软件包根目录的父目录执行：

```bash
rbnx validate ./acme_chassis
rbnx build -p ./acme_chassis
rbnx start -p ./acme_chassis --endpoint 127.0.0.1:50051 \
  --set can_port=can0
```

`rbnx validate` 验证软件包清单结构；`build` 执行软件包的构建入口；`start` 需要已经运行的 Atlas。开发细节见[开发者手册](../developer-guide.md)。

## 5. 组装 `robonix_manifest.yaml`

每个列表条目代表一个运行实例。`name` 同时是期望注册的提供方 ID 和日志标签，并通过 `RBNX_INSTANCE_NAME` 传给软件包；提供方必须按 4.2 节所示使用该实例名。`url` 或 `path` 选择软件包；`manifest` 选择该软件包为目标平台提供的构建和启动变体；`config` 原样传给提供方的 `CMD_INIT`。

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
    robot_yaml: soma.yaml
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
    manifest: package_manifest.jetson-native.yaml

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

对外连接只开放实际需要的端口，并限制在可信局域网或 Tailscale。客户端先连接 Atlas，再由能力目录发现 Liaison、Executor 和音频桥接提供方；无需为客户端硬编码这些组件的端口。Pilot 通常保持环回监听。不要把 API 密钥写进清单；在启动 shell 中导出环境变量。

同一软件包可以同时提供 `package_manifest.yaml`、`package_manifest.jetson-native.yaml`、`package_manifest.jetson-docker.yaml` 等变体。整机条目的 `manifest:` 明确选择其中一个；不存在的文件会直接报错，不会静默回退。各变体必须共享软件包身份、版本和标准能力约定，只允许系统依赖、构建脚本和启动方式不同。在新部署中选用变体前，用 `rbnx validate <package-dir>` 校验默认清单；再让部署条目的 `manifest:` 逐一选择各变体，执行 `rbnx build -f robonix_manifest.yaml` 并核对身份、版本和能力列表。

## 6. 配置建图、导航与场景服务

算法软件包提供实现和配置模板，**机器人参数归本体部署仓库所有**。不要把新机器人配置组合加入 Mapping 或 Navigation 上游代码。

### 6.1 建图

从 [`service-map-rbnx/config/rtabmap_params.template.yaml`](https://github.com/syswonder/service-map-rbnx/blob/main/config/rtabmap_params.template.yaml) 复制一份到本体仓库 `config/rtabmap_params.yaml`，然后基于传感器位置、频率、噪声、机器人 footprint 和计算平台调参。上游 template 只供复制，运行时不会自动加载。

```yaml
service:
  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    branch: main
    manifest: package_manifest.yaml
    config:
      params_file: config/rtabmap_params.yaml
      occupancy_sources: [lidar]
      sensor_providers:
        lidar3d: roof_lidar
        odom: base_chassis
        rgb: front_camera
        depth: front_camera
```

`sensor_providers` 的值是 Atlas 提供方 ID，不是话题名。支持角色为 `lidar2d`、`lidar3d`、`rgb`、`depth`、`imu` 和 `odom`；RGB-D 必须同时指定 `rgb` 与 `depth`。`rtabmap_params` 可以在部署清单中对文件做少量最终覆盖，但完整基线仍应保存在部署仓库。字段以 Mapping 仓库根目录的 `config.spec` 为准。

旧 `rtabmap_profile` 和 `sensors` 仍可用于迁移并产生警告；新部署不得依赖它们。

### 6.2 导航

从 [`service-navigation-rbnx/config/nav2_params.example.yml`](https://github.com/syswonder/service-navigation-rbnx/blob/main/config/nav2_params.example.yml) 复制一份到本体仓库 `config/nav2_params.yaml`，再将 footprint、速度与加速度、代价地图、规划器、控制器、目标检查器和行为树参数与真实底盘对齐。`params_file` 的相对路径以 `robonix_manifest.yaml` 所在目录为基准：

```yaml
  - name: nav2
    url: https://github.com/syswonder/service-navigation-rbnx
    branch: main
    manifest: package_manifest.yaml
    config:
      params_file: config/nav2_params.yaml
      provider_ids:
        map: mapping
        odom: base_chassis
        scan: front_lidar
```

原生二维激光雷达使用 `provider_ids.scan`。只有三维点云需要 `provider_ids.scan_cloud` 和显式 `scan_projection`；投影高度、目标坐标系、量程和本体过滤边界必须依据雷达安装位姿与 Soma 本体描述设置。自定义行为树保存在部署仓库，并通过 `bt_xml_file` 引用；Nav2 参数文件中对应的行为树路径必须写为 `__ROBONIX_BT_XML__`，Navigation 才会把该占位符替换成部署文件的绝对路径。仅设置 `bt_xml_file` 而不放置占位符不会切换行为树。全部字段、默认值和弃用项以 Navigation 仓库根目录的 [`config.spec`](https://github.com/syswonder/service-navigation-rbnx/blob/main/config.spec) 为准；Nav2 参数语义参考 [Nav2 配置指南](https://docs.nav2.org/configuration/index.html)。

旧 `params_profile` 仍可用于迁移并产生警告；新部署不得依赖它。

### 6.3 场景服务

当前 `rbnx boot` 只识别 `system.scene.manifest`；`camera_provider_id`、`web_port` 等字段不会从 `robonix_manifest.yaml` 送达 Scene。短期内，把 Scene 配置保存在部署仓库，例如：

```yaml
# config/scene.yaml
camera_provider_id: front_camera
web_port: 50107
```

再由机器人仓库的启动包装脚本导出绝对路径后启动：

```bash
DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
export RBNX_CONFIG_FILE="$DEPLOY_DIR/config/scene.yaml"
exec rbnx boot -f "$DEPLOY_DIR/robonix_manifest.yaml" "$@"
```

用于 RGB-D Scene 的提供方必须同时暴露标准 `camera/rgb` 和 `camera/depth` 能力。Scene 当前通过上述包装脚本与 `RBNX_CONFIG_FILE` 读取运行参数；不要把 `camera_provider_id`、`web_port` 等字段放进 `system.scene` 并假定它们会被传入。

Scene 优先从 Atlas 解析 `robonix/service/map/pose`，没有时再尝试 `robonix/service/map/odom`，最后使用 TF 获取机器人位姿。RGB-D 世界坐标投影当前优先组合地图位姿与 `robonix/primitive/camera/extrinsics`，该组合不完整时再从 TF 查询相机位姿。接入新本体时必须保证 URDF 能连接 `base_link` 与所选 RGB、深度光学坐标系，并由同一相机提供方声明显式外参能力，确保父坐标系、子坐标系与图像消息一致。[Issue #156](https://github.com/syswonder/robonix/issues/156) 跟踪改为 TF 优先并删除旧外参接口的迁移。

因此新本体必须同时满足：Mapping 绑定正确的里程计与传感器提供方并发布 `map/pose`；相机提供方声明 RGB、深度、内参与当前实现所需的相机外参；完整 URDF/TF 树连接所有坐标系。不要实现旧的 `robonix/system/soma/sensor_extrinsics`；Soma 保存完整本体模型，坐标变换由机器人描述原语发布。

## 7. 按风险分阶段验收

不要第一次就把全部驱动和运动服务一起启动。每一阶段通过后再加入下一组软件包。

### 7.1 静态检查与构建

```bash
rbnx build -f robonix_manifest.yaml
```

该命令适用于不需要额外宿主准备的通用部署。AgileX Ranger Mini v3 应在其仓库根目录执行 `bash build.sh`。

构建必须无失败。Jetson 原生软件包使用宿主已安装且与 JetPack 匹配的 CUDA、PyTorch、TorchVision、TorchAudio 等系统组件；有些开发套件出厂已带这些组件。先核对 JetPack/L4T 与实际 Python wheel，再参考 [Jetson AI Lab 软件包索引](https://pypi.jetson-ai-lab.io/)；不要照抄另一块板子的 CUDA 索引。软件包自身携带的厂商 SDK 或 ROS 驱动由软件包构建脚本处理。

### 7.2 无运动启动

暂时移除或禁用底盘和机械臂运动软件包，只启动 Atlas、Soma、Vitals、机器人描述原语和只读传感器。通用部署在终端 A 执行：

```bash
rbnx boot -f robonix_manifest.yaml
```

`rbnx boot` 会保持在前台并等待 Ctrl-C 或关闭信号。AgileX Ranger Mini v3 在终端 A 改为执行 `bash start.sh`，由包装脚本设置 `${ROBONIX_DEPLOY_DIR}` 并完成宿主准备。

保持终端 A 运行，在终端 B 检查提供方状态：

```bash
rbnx caps -v
```

检查原语和服务提供方为 `ACTIVE`；技能在首次调用前通常保持 `INACTIVE`。同时检查提供方 ID 与清单中的 `name` 一致、URDF/TF 连通、传感器坐标系和时间戳正确。失败时查看：

```bash
provider_id=base_chassis
rbnx logs -t "$provider_id"
```

### 7.3 单设备运动

先在硬件安全条件下验证底盘低速前进和停止、机械臂单关节和夹爪开合以及急停；再验证导航。任何提供方在输入超时、进程退出或 `CMD_SHUTDOWN` 时都必须停止其硬件输出。

### 7.4 建图、导航与交互

依次验证：

1. 静止时点云/scan 与墙面重合，旋转时 TF 和定位不漂移；
2. 建图、保存、重启、加载同一 map id 后 occupancy 与 Scene 标记恢复；
3. 已知区域内规划、避障、到点停止和 cancel；
4. Client 文本任务、相机理解、语音输入/输出与 stop；
5. Vitals 能区分本体状态、提供方状态和故障。

地图文件和联调环境属于测试数据。验收后只删除本次创建的临时 map id，不要清理操作者已有地图目录。

**整机完成标志：** 一台新机器克隆部署仓库、设置文档列出的环境变量并运行构建和启动后，可以复现相同的能力提供方列表、Soma/TF、传感器、地图、导航与客户端链路；不依赖未记录的绝对路径或手工后台进程。

清单中的 `branch: main` 只适合开发阶段跟踪上游。验收与发布时应改用软件包仓库提供的不可变发布标签，并把 `rbnx-boot/cache/` 中每个实际源码检出的提交哈希记录到验收材料；`rbnx` 当前不支持在 `branch:` 中直接填写任意提交哈希。这样新的克隆才能解析到与验收时相同的软件包内容。

## 8. 发布和持续维护

原语、服务和技能各自提交到独立社区仓库；机器人部署仓库只组装它们并保存机器人专属配置。提交格式和软件包目录流程见[软件包目录发布流程](package-catalog.md)。

发布前至少确认：

- 软件包清单与机器人目录元数据完整，许可证为有效 SPDX 标识；
- 每个公开 `config` 字段在根目录 `config.spec` 中有解释；
- README 写明系统依赖、硬件连接、构建/启动、权限和安全限制；
- 软件包在它声明支持的目标平台上通过 `rbnx validate`、构建、启动和关闭；
- deployment 的 `assets/robot.jpg` 已压缩到网页适合尺寸；
- 维护者、默认分支与软件包目录记录一致。

新设备接入后的参数变化优先提交到对应机器人部署仓库；可复用的驱动修复提交到原语仓库；标准接口变更才进入 Robonix 主仓库。这样其他本体可以升级软件包，而不会继承某台机器的私有参数。
