# 机器人本体接入指南


本章面向负责一台真实机器人或仿真本体的集成工程师。完成后，你将得到一个可独立克隆、构建和启动的**机器人部署仓库（Robot Deployment）**：仓库内包含整机描述、硬件实例、机器人专属配置，以及对社区软件包的引用。

<div class="procedure-meta">
  <div><strong>源码仓库</strong><code>syswonder/robonix</code></div>
  <div><strong>参考部署</strong><a href="https://syswonder.github.io/robonix-package-catalog/robots/robonix.robot.agilex.ranger_mini_v3/">AgileX Ranger Mini v3</a></div>
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
│   └── robot.jpg                  # 可选：机器人目录页面的本体预览图
├── build.sh                       # 可选：整机便捷构建入口
└── start.sh                       # 可选：CAN、RMW router 等宿主准备后再 boot
```

`assets/robot.jpg` 只供 Robonix 软件包目录的机器人列表展示，不参与构建、启动或模型推理。准备把机器人仓库提交到目录时再添加并压缩该图片；不发布目录条目时可以省略。

硬件驱动通常放在独立原语仓库中，由 `robonix_manifest.yaml` 的 `url` 引用；只有暂不复用的部署私有代码才放在本体仓库。先查看目录中的 [AgileX Ranger Mini v3 本体页面](https://syswonder.github.io/robonix-package-catalog/robots/robonix.robot.agilex.ranger_mini_v3/)了解已发布的硬件组成和软件包，再进入 [`robot-agilex-ranger_mini_v3`](https://github.com/syswonder/robot-agilex-ranger_mini_v3) 查看实际部署清单、配置和包装脚本。学习单个软件包结构时使用 [`template-rbnx`](https://github.com/syswonder/template-rbnx/tree/60dc85834c2714022b1821e6fce6c629c0314699)。采用任何参考部署前，都要核对其整机 URDF 是否包含当前机器人实际安装的全部部件；缺少的坐标关系必须先补齐，不能依赖另一份分离 URDF。

AgileX 仓库依赖包装脚本设置部署目录和宿主环境，构建与启动应使用：

```bash
bash build.sh
bash start.sh
```

不要绕过包装脚本直接执行 `rbnx build -f` 或 `rbnx boot -f`。

## 1. 建立可追溯的开发环境

先按[快速开始](../getting-started/quickstart.md)安装 Rust、`uv`、Docker 和 Robonix。先确定联调使用的源码提交，再在每台目标机上检出同一提交：

```bash
git clone --recurse-submodules https://github.com/syswonder/robonix.git
cd robonix
git checkout --detach 181d3eb974fd495a795ed120a0a4c6e6f342d179
git submodule update --init --recursive
make install
rbnx setup "$PWD"
rbnx --version
git rev-parse HEAD
```

`rbnx setup` 记录 Robonix 源码根目录，使部署中的 `${ROBONIX_SOURCE_PATH}`、标准能力约定和代码生成输入可以从任意工作目录解析。把 `git rev-parse HEAD` 的输出写进联调记录，并让所有目标机使用同一个已验证提交。

在仓库外创建部署骨架：

```bash
cd ..
rbnx init robot-acme-rover
cd robot-acme-rover
test -f robonix_manifest.yaml
test -f .gitignore
```

两条 `test` 命令都应以状态码 0 结束。`rbnx init` 当前只创建部署目录、`robonix_manifest.yaml` 和 `.gitignore`。随后建立本体需要的目录：

```bash
mkdir -p primitives services skills config urdf assets
git init
printf '# ACME Rover Robonix deployment\n' > README.md
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

:::danger[首次运动测试]

真实机器人第一次运动测试前，必须确认硬件急停能够独立停止底盘和机械臂。底盘应先架空或限制在清空区域，机械臂应先关闭运动能力或使用安全姿态。开始测试前，还必须按照设备厂商的安全手册检查急停、限位、安全区域和操作人员要求。

:::

## 3. 编写完整 URDF 与 Soma 描述

### 3.1 一台机器人只维护一棵完整 URDF

`soma.yaml` 的 `urdf.path` 指向整机 URDF。底盘、传感器和机械臂之间的固定变换都必须出现在这棵树中；建图、导航、Scene 和机器人描述原语依赖同一坐标关系。不要分别维护互相重叠的“底盘 URDF”和“相机 URDF”并同时发布 TF。

如果仿真器或厂商程序已经发布 `/robot_description`、`/tf` 和 `/tf_static`，不要再启动第二个 TF 发布方。没有现成发布方时，在部署中引用通用 [`primitive-robot-description-rbnx`](https://github.com/syswonder/primitive-robot-description-rbnx)；它从 Soma 读取 URDF，并发布描述与坐标变换。

AgileX Ranger Mini v3 仓库当前只能作为部署连线的部分参考，不能用来证明底盘与机械臂已经合并为一棵完整 URDF；新机器人仍须按本节完成整机坐标树。

### 3.2 创建 `soma.yaml`

Soma 描述机器人本体与部件，不负责环境物体，也不替 Vitals 做健康阈值判断。`soma.yaml` 是本体描述文件；部署清单里的 `system.soma` 和可选的 Soma `config` 文件是进程配置。不要把监听地址、Atlas 地址等进程参数写进本体文件，也不要把 `robot.components` 写进进程配置。Soma 的进程参数见主仓库的 [`system/soma/README.md`](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/system/soma/README.md)；本节的本体字段以实际读取它们的 [`store.rs`](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/system/soma/src/store.rs) 为准。

编写前先准备三份事实，而不是从示例里猜值：

1. 完整整机 URDF：提供真实 link、joint、父子关系和固定安装变换；
2. `robonix_manifest.yaml`：提供每个运行实例的 `name`；
3. 各实例的软件包清单或 `rbnx caps -v`：提供该实例实际声明的能力约定 ID。

然后按下面的顺序写：先写能够被 Soma 加载的最小文档，再补 footprint，接着把物理设备树投影为逻辑部件树，最后填写提供方、能力和描述信息。

最小可加载文件只有两项：

```yaml
urdf:
  path: ./urdf/acme_rover.urdf

robot:
  id: acme_rover_01
```

`urdf` 和 `robot` 都必须是 YAML 映射。缺少它们、类型错误、URDF 文件不可读或 `robot.id` 为空都会使 Soma 启动失败。当前实现只对少数字段做强制解析；其它内容会原样保留并通过 `get_yaml` 返回。下表明确区分“运行时校验”和“由接入方维护”。

#### URDF 与机器人根字段

| 字段 | 类型与是否必填 | 如何填写 | 当前运行时行为 |
|---|---|---|---|
| `urdf.path` | 字符串，必填 | 指向唯一的整机 URDF；相对路径以 `soma.yaml` 所在目录为基准 | Soma 读取文件文本；不可读时启动失败，但不会解析 URDF XML |
| `urdf.root_link` | 字符串，建议填写 | 填整机 URDF 的真实根 link，例如 `base_link` | 原样保留，不与 URDF 交叉校验 |
| `urdf.model_name` | 字符串，建议填写 | 填稳定的机器人型号标识 | 原样保留，无默认值和类型校验 |
| `robot.id` | 字符串，必填 | 填该机器人在当前部署中的稳定 ID；不要带首尾空格 | 仅检查去除空白后非空，但保存原字符串；不会替接入方检查全局唯一性 |
| `robot.display_name` | 字符串，可选 | 填 UI 和自然语言中使用的名称 | 原样保留 |
| `robot.family` | 字符串，可选 | 填本体类别，例如 `mobile_manipulator` | 原样保留 |
| `robot.root_part` | 字符串，可选 | 填逻辑部件树根节点的 `id` | 原样保留，不检查引用 |
| `robot.dimensions` | 映射，可选 | 记录静态外形尺寸；本指南统一使用米并以 `_m` 结尾 | 原样保留，不检查数值或单位 |
| `robot.mass_kg` | 数值，可选 | 填设备资料或实测的整机质量 | 原样保留 |
| `robot.passable_door_width_m` | 数值，可选 | 填接入方确认的最小安全通行宽度，不要直接等同于车宽 | 原样保留，不参与导航约束 |
| `robot.exports` | 列表，可选 | 放属于整机而非单一硬件部件的服务能力，例如导航 | 原样保留；字段结构见下表 |
| `robot.components` | 列表，可选 | 放逻辑部件树的第一层节点 | 供消费者读取，并用于识别部分运行状态；不验证 ID、URDF 或提供方引用 |
| `description` | 任意 YAML，可选 | 记录简短摘要、已验证能力和明确限制 | 原样保留，不注册成独立能力，也不做 schema 校验 |

#### 静态底盘 footprint

| 字段 | 类型与是否必填 | 如何填写 | 当前运行时行为 |
|---|---|---|---|
| `robot.footprint` | 映射，可选 | 移动机器人建议填写；非移动本体可以省略 | 省略不阻止启动，但调用 footprint 接口时返回前置条件未满足 |
| `robot.footprint.base_frame` | 非空字符串，footprint 存在时必填 | 使用轮廓坐标所属的真实底盘 frame，通常为 `base_link` | 空值使加载失败 |
| `robot.footprint.points` | 至少 3 个 `[x, y]` 顶点，footprint 存在时必填 | 从机器人俯视图按米填写静态底盘外轮廓；按顺时针或逆时针连续排列，不重复首点 | 拒绝少于 3 点、非二元坐标和非有限数值，并计算内切/外接半径 |

接入方必须用几何工具或 RViz2 确认多边形闭合、不自交且包含 `base_frame` 原点。当前 Soma 只检查顶点和到边距离，没有完整的点在多边形内校验；一个整体偏离原点的多边形仍可能被加载。不要把机械臂伸展时的动态包络写进静态底盘 footprint，运动安全范围应由控制器和安全策略另行约束。

#### 逻辑部件树与能力映射

`robot.components` 描述“这台机器人由哪些系统可理解的部件组成”，不是 URDF 的副本。只保留底盘、传感器、机械臂、末端执行器等有运行意义的节点，不要把每个 URDF joint 都复制进来。

```text
robot: acme_rover_01
├── base                         -> base_link
│   ├── front_lidar              -> front_lidar_link
│   ├── base_imu                 -> imu_link
│   └── front_camera             -> front_camera_link
└── arm                          -> arm_base_link / arm_mount_joint
    ├── gripper                  -> gripper_link / gripper_joint
    └── wrist_camera             -> wrist_camera_link
```

| 字段 | 类型与是否必填 | 如何填写 | 当前运行时行为 |
|---|---|---|---|
| `components[].id` | 字符串，作者应填写 | 在这台机器的逻辑树中取稳定、可读的名称 | 通用部件不会因缺失或重复而加载失败；被识别为夹爪但未写 `id` 时，内部状态 ID 默认为 `gripper` |
| `components[].type` | 字符串，作者应填写 | 填部件类别，例如 `mobile_base`、`rgbd_camera`、`manipulator` 或 `parallel_jaw_gripper` | 通常原样保留；只有 `parallel_jaw_gripper` 和 `end_effector` 会进入夹爪状态聚合 |
| `components[].urdf_link` | 字符串，可选 | 填该部件在完整 URDF 中对应的真实 link | 原样保留，不检查 link 是否存在 |
| `components[].urdf_joint` | 字符串，可选 | 填该部件或安装点对应的真实 joint | 原样保留，不检查 joint 是否存在 |
| `components[].components` | 列表，可选 | 递归填写直接安装或逻辑从属的子部件 | 用于构成逻辑树 |
| `exports[]` | 列表，可选 | 放在真正提供或汇总该能力的机器人/部件节点上；需要夹爪聚合时，夹爪自身或祖先必须提供关节状态 | 当前不检查提供方或能力是否存在；结构不完整时可能只被保留而不参与状态聚合 |
| `exports[].provider_id` | 字符串，每条有效导出必填 | 必须逐字等于 `robonix_manifest.yaml` 中对应实例的 `name` | 夹爪状态聚合会用它查找关节状态；缺失时该导出不能建立夹爪数据来源 |
| `exports[].capabilities` | 列表，每条有效导出必填 | 列出该提供方属于此部件的能力 | 当前不与 Atlas 自动交叉校验；缺失时不会从这条导出发现关节状态能力 |
| `capabilities[].path` | 字符串，每条能力必填 | 从接口目录和软件包清单复制完整能力约定 ID，例如 `robonix/primitive/chassis/odom` | 不要写 ROS 2 话题名或 SDK 函数名；缺失或不匹配不会触发通用加载错误 |
| `capabilities[].description` | 字符串，可选 | 描述这个实例的实际用途、安装位置或限制 | 原样保留 |

从物理设备树到 YAML 的对应关系是：URDF 决定几何和 TF；`components` 决定逻辑层级；部署实例 `name` 决定 `provider_id`；标准接口和软件包清单决定 `capabilities[].path`。这四类值不能互相替代。

#### 夹爪运行状态字段

只有需要 Soma 从关节状态推断夹爪开合时，才在夹爪部件添加 `state`：

| 字段 | 类型与默认值 | 如何填写 | 当前运行时行为 |
|---|---|---|---|
| `state.joint_name` | 字符串，默认 `gripper` | 填关节状态消息 `name[]` 中的精确名称；当前只读取这一条标量关节位置 | 找不到同名关节时状态为未知 |
| `state.open_position_m` | 数值，无默认值 | 空夹爪完全张开时，记录该提供方 `position[]` 对应元素的实测值；必须与提供方使用同一单位 | 缺失或类型错误不会使 YAML 启动失败，但该部件不会被识别为可聚合的夹爪；Soma 不做单位转换或有限性校验 |
| `state.open_tolerance_m` | 数值，默认 `0.002` | 使用与 `open_position_m` 相同的单位，填写能覆盖静态噪声但不会吞掉实际闭合量的非负有限容差 | 新鲜样本位于容差内时状态为 `open`；当前不会拒绝负数或非有限值，错误标定会使分类失真 |

夹爪部件自身或任一祖先节点还必须有一条 `exports`，把 `robonix/primitive/arm/joint_states` 绑定到正确的 `provider_id`。否则 Soma 无法知道应读取哪个机械臂实例。

以下是一份用于教学的完整移动操作机器人示例。它覆盖底盘、二维雷达、IMU、前置 RGB-D 相机、机械臂、夹爪和腕部相机；link、joint、尺寸、质量和标定值必须换成目标机器人的真实数据。

```yaml
urdf:
  path: ./urdf/acme_rover.urdf
  root_link: base_link
  model_name: acme_rover

robot:
  id: acme_rover_01
  display_name: "ACME Rover"
  family: mobile_manipulator
  root_part: base
  dimensions: { length_m: 0.72, width_m: 0.50, height_m: 1.24 }
  footprint:
    base_frame: base_link
    points: [[0.36, 0.25], [0.36, -0.25], [-0.36, -0.25], [-0.36, 0.25]]
  mass_kg: 41.5
  passable_door_width_m: 0.65
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
            - { path: robonix/primitive/chassis/twist_in, description: "Accept velocity commands." }
            - { path: robonix/primitive/chassis/odom, description: "Read chassis odometry." }
      components:
        - id: front_lidar
          type: lidar_2d
          urdf_link: front_lidar_link
          exports:
            - provider_id: front_lidar
              capabilities:
                - { path: robonix/primitive/lidar/lidar, description: "Stream the front laser scan." }
        - id: base_imu
          type: imu
          urdf_link: imu_link
          exports:
            - provider_id: base_imu
              capabilities:
                - { path: robonix/primitive/imu/imu, description: "Stream base inertial measurements." }
        - id: front_camera
          type: rgbd_camera
          urdf_link: front_camera_link
          exports:
            - provider_id: front_camera
              capabilities:
                - { path: robonix/primitive/camera/rgb, description: "Stream RGB images." }
                - { path: robonix/primitive/camera/depth, description: "Stream metric depth registered to RGB." }
                - { path: robonix/primitive/camera/intrinsics, description: "Read CameraInfo for the selected stream." }
                - { path: robonix/primitive/camera/snapshot, description: "Capture one RGB frame." }
    - id: arm
      type: manipulator
      urdf_link: arm_base_link
      urdf_joint: arm_mount_joint
      exports:
        - provider_id: arm_controller
          capabilities:
            - { path: robonix/primitive/arm/joint_states, description: "Read arm and gripper joints." }
            - { path: robonix/primitive/arm/joint_command, description: "Command arm joints." }
            - { path: robonix/primitive/arm/pos_command, description: "Command a Cartesian end-effector target; the provider performs IK." }
            - { path: robonix/primitive/arm/end_pose, description: "Read the end-effector pose." }
      components:
        - id: gripper
          type: parallel_jaw_gripper
          urdf_link: gripper_link
          urdf_joint: gripper_joint
          state:
            joint_name: gripper
            open_position_m: 0.080
            open_tolerance_m: 0.003
          exports: []
        - id: wrist_camera
          type: rgbd_camera
          urdf_link: wrist_camera_link
          exports:
            - provider_id: wrist_camera
              capabilities:
                - { path: robonix/primitive/camera/rgb, description: "Stream wrist RGB images." }
                - { path: robonix/primitive/camera/depth, description: "Stream wrist metric depth." }
                - { path: robonix/primitive/camera/intrinsics, description: "Read wrist camera intrinsics." }

description:
  summary: "Indoor mobile manipulator with a two-finger gripper."
  can_do: ["drive", "navigate", "observe RGB-D", "manipulate objects"]
  cannot_do: ["climb stairs", "operate outdoors in rain"]
```

示例中的夹爪在新鲜关节样本落入开启容差时报告 `open`，否则报告 `holding_or_partially_closed`；样本超过 2 秒或缺少对应 joint 时报告未知。若配置的提供方从未发出关节样本，快照中不会凭空创建该机械臂或夹爪状态。`likely_holding` 只是“没有处于标定开启位置”的启发式结果，不能证明夹爪内确实有物体。对于角度制、联动或多关节夹爪，应先由原语提供方选定并稳定输出一个可标定的代表关节；不能满足这一条件时，不要声明上述 `state` 推断。

在部署清单中绑定这份文件：

```yaml
system:
  soma:
    listen: 127.0.0.1:50091
    robot_yaml: soma.yaml
```

`robot_yaml` 的相对路径以部署清单所在目录为基准。若当前 `-f` 选择的清单同目录存在 `soma.yaml`，也可以省略 `robot_yaml`；包含原语或技能的部署会自动注入 Soma 和该本体描述文件。显式写出的 `system.soma.config` 是进程配置路径，不是本体描述文件。完整的注入、默认值和诊断规则见[系统部署与启动流程](../architecture/deployment-and-startup.md#soma-sidecar-与进程配置)。

先验证文件存在、URDF 可由专用工具解析，再构建部署。在 Ubuntu 上，`check_urdf` 由 `liburdfdom-tools` 提供：

```bash
test -f soma.yaml
test -f urdf/acme_rover.urdf
check_urdf urdf/acme_rover.urdf
rbnx build -f robonix_manifest.yaml
```

`check_urdf` 应以状态码 0 结束并打印解析出的 link 树；`rbnx build` 应成功完成部署清单实际选择的所有软件包。Soma 自己只读取 URDF 文本，不解析 XML，也不核对 `root_link`、部件 link 或 joint。

:::danger[先使用无运动清单]

第一次启动真实机器人时，先复制部署清单并移除底盘、机械臂和其他运动提供方的整个实例，只保留 Soma、机器人描述和只读传感器。当前部署格式没有通用 `enabled: false`；不要用未知字段假装禁用硬件。

:::

在无运动清单或仿真环境中执行启动验收：

```bash
rbnx boot -v -f robonix_manifest.yaml
rbnx caps -v
rbnx inspect
rbnx logs --list-tags
rbnx logs -t soma -l warn
rbnx logs -t soma -f
```

验收结果至少应满足：

1. `rbnx caps -v` 中的 `soma` 提供方为 `ACTIVE`，并声明 `get_yaml`、`get_urdf`、`footprint`、`get_health` 和 `health` 5 条本体接口。
2. `base_chassis`、`front_lidar`、`base_imu`、`front_camera`、`arm_controller` 和 `wrist_camera` 等实例 ID 与 `soma.yaml` 完全一致；只核对无运动清单中实际保留的实例。
3. 每个实例的能力路径和传输与其软件包清单一致，ROS 2 `arm/joint_states` 与 `chassis/odom` 提供方可被 Soma 发现。
4. `soma` 日志没有 YAML/URDF 读取错误、运行状态 reader 启动错误或意外的 stale/missing-joint 警告。

从其他工作目录检查已写入的故障现场时使用：

```bash
rbnx logs -d /path/to/deploy/rbnx-boot/logs -t soma --json
```

常见问题按下面的信号定位：

| 信号 | 原因与处理 |
|---|---|
| `missing robot_yaml: set --robot-yaml, ROBONIX_SOMA_ROBOT_YAML, or provide it in --config <yaml>` | 选中的部署清单旁没有 `soma.yaml`，也没有显式 `robot_yaml`；补文件或改为正确路径 |
| `read URDF ...` | `urdf.path` 相对的是 `soma.yaml` 目录，不是 shell 当前目录；修正路径并重新运行 `test -f` / `check_urdf` |
| footprint 解析失败 | 检查 `base_frame`、至少 3 个有限二元顶点和非退化边；另用几何工具或 RViz2 检查顶点顺序、自交和原点包含关系 |
| footprint RPC 返回 `FAILED_PRECONDITION` | `robot.footprint` 是可选启动字段，但调用该 RPC 前必须声明有效轮廓 |
| 夹爪为 `unknown_missing_joint` 或 `unknown_stale` | 核对 ancestor/self 的 joint-state `provider_id`、精确能力路径、`joint_name` 和 2 秒新鲜度 |
| YAML 中有部件但 Atlas 中没有提供方 | Soma 不做 `exports` 与部署清单的引用完整性检查；修正实例 `name`、包清单或启动失败 |

最后要明确当前边界：Soma 不验证部件 ID 唯一性，不交叉检查 URDF link/joint/root、提供方或能力路径，也不验证通用尺寸、质量和标定数值。它没有传感器外参接口，源码中的 description 约定也尚未由 Soma 注册或实现；相机外参应走相机能力或完整 URDF/TF。当前本体健康聚合也不会填充电源、安全和故障事实，一个 Soma 进程只服务一台机器人。接口和聚合细节见[本体服务](../interface-catalog/system/soma.md)。

## 4. 为硬件实现标准原语

### 4.1 先选择标准接口

不要先从厂商 SDK 的函数名设计 Robonix 接口。先在[接口目录](../interface-catalog/index.md)选择本体需要实现的标准能力约定，再把 SDK 适配到该能力约定。常见对应关系如下：

| 硬件 | 接口目录 | 常用输入/输出 |
|---|---|---|
| 移动底盘 | [底盘](../interface-catalog/primitive/chassis.md) | `move`、`twist_in`、`odom` |
| RGB/RGB-D 相机 | [相机](../interface-catalog/primitive/camera.md) | 彩色图、深度图、快照、内参和外参；外参必须与整机 URDF/TF 一致 |
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

从部署根目录执行命令后，`rbnx package-new` 先生成以下基础骨架：

```text
primitives/acme_chassis/
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

脚手架不会猜测软件包是否接受部署参数，也不会生成厂商资源的清理逻辑。本节的 ACME 底盘示例后续会公开 `can_port`、`sentinel_timeout_s` 等配置，并演示可选的停止清理入口，因此准备发布时还要补入：

```text
primitives/acme_chassis/
├── config.spec          # 本节配置字段的说明
└── scripts/
    └── stop.sh          # 仅在需要额外清理外部资源时提供
```

`config.spec` 和 `scripts/stop.sh` 不是 `rbnx package-new` 自动生成的文件。软件包没有公开配置时可以不提供 `config.spec`；没有受管进程组之外的守护进程、容器或硬件资源需要清理时，应同时省略 `stop.sh` 和清单中的 `stop:`。

生成的 `main.py` 默认把提供方 ID 写死为 `acme_chassis`。可复用软件包应将原有的提供方构造代码替换为下面的实现，并保留脚手架生成的 `Ok` 导入和 `on_init` 处理函数：

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

上面的片段只解决运行实例命名。完成原语时，还必须为清单中的 `move`、`twist_in` 和 `odom` 分别实现 gRPC 或 ROS 2 端点；清单本身不会自动注册这些能力。实现方式见[开发原语](../developer-guide.md#9-开发原语)。

`package_manifest.yaml` 中的发布元数据、构建入口、启动入口、可选的停止清理入口和标准能力必须与实现一致：

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
stop: bash scripts/stop.sh

capabilities:
  - name: robonix/primitive/chassis/move
  - name: robonix/primitive/chassis/twist_in
  - name: robonix/primitive/chassis/odom
```

这是一份新软件包清单。清单未列 Driver 时，框架会自动提供共享生命周期 Driver。

:::warning[后向兼容：已有命名空间 Driver]
已有接入仓库若使用 `<provider-namespace>/driver` 和本地 Driver TOML，应暂时保持原结构并按[兼容流程](packaging-spec.md#42-已有命名空间-driver-的兼容流程)验收。该方式计划迁移到共享 Driver；不要在保留旧条目的同时追加共享 Driver。
:::

`stop` 不是必填字段。`rbnx boot` 关闭受管软件包时会先请求 Driver 的 `CMD_SHUTDOWN`，再运行已配置的 `stop` hook，最后用进程组终止作为兜底；独立执行 `rbnx start` 不会调用 `stop`。保留本例的 `stop:` 时，`scripts/stop.sh` 必须可重复执行，并负责关闭由软件包创建、但不属于受管进程组的厂商守护进程、容器或其他外部资源。

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

使用三个终端验证。终端 0 启动 Atlas：

```bash
robonix-atlas --listen 127.0.0.1:50051 \
  --capabilities "$(rbnx path capabilities)"
```

上面的参数加载 Robonix 标准能力约定。若软件包还定义了自有约定，将 `$(pwd)/primitives/acme_chassis/capabilities` 作为逗号分隔的第二个目录追加到 `--capabilities`。

在部署根目录中，终端 A 执行：

```bash
rbnx validate ./primitives/acme_chassis
rbnx build -p ./primitives/acme_chassis
rbnx start -p ./primitives/acme_chassis --endpoint 127.0.0.1:50051 \
  --set can_port=can0
```

`rbnx start` 在前台持续运行。保持终端 A 不退出，在终端 B 验证注册结果：

```bash
rbnx caps -v
```

输出中应出现 `acme_chassis`，状态应为 `ACTIVE`，已实现的运行时能力应与清单中的能力约定对应。清单列出但代码未声明的能力不会出现在 `rbnx caps -v` 中，应视为实现未完成。检查后回到终端 A 按 Ctrl-C，确认提供方退出且硬件停止输出。`rbnx validate` 只检查清单的基础结构；发布前还需按第 8 节复核元数据、能力引用和 `config.spec`。

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
    config:
      camera_provider_id: front_camera
      web_port: 50107

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

  - name: front_lidar
    url: https://github.com/acme/primitive-acme-lidar-rbnx
    branch: main
    config: {}
```

对外连接只开放实际需要的端口，并限制在可信局域网或 Tailscale。客户端先连接 Atlas，再由能力目录发现 Liaison、Executor 和音频桥接提供方；无需为客户端硬编码这些组件的端口。Pilot 通常保持环回监听。不要把 API 密钥写进清单；在启动 shell 中导出环境变量。

同一软件包可以同时提供 `package_manifest.yaml`、`package_manifest.jetson-native.yaml`、`package_manifest.jetson-docker.yaml` 等变体。整机条目的 `manifest:` 明确选择其中一个；不存在的文件会直接报错，不会静默回退。各变体必须共享软件包身份、版本和标准能力约定，只允许系统依赖、构建脚本和启动方式不同。通用 x86 Docker 运行通常选默认清单，Jetson Docker 选 `package_manifest.jetson-docker.yaml`，只有 Jetson 原生运行才选 `package_manifest.jetson-native.yaml`。最终以软件包实际提供的清单为准。

当前 `rbnx validate <package-dir>` 只校验默认的 `package_manifest.yaml`，不会自动比较多个变体。发布前应分别打开每个变体，确认 `package.name`、`package.version` 和 `capabilities` 完全一致；随后在部署清单中逐一选择目标变体执行构建。每个部署实例必须且只能设置 `path` 或 `url`，`branch` 只与 `url` 一起使用。

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
        lidar2d: front_lidar
        odom: base_chassis
        rgb: front_camera
        depth: front_camera
```

`sensor_providers` 的值是 Atlas 提供方 ID，不是话题名。支持角色为 `lidar2d`、`lidar3d`、`rgb`、`depth`、`imu` 和 `odom`；RGB-D 必须同时指定 `rgb` 与 `depth`。`rtabmap_params` 可以在部署清单中对文件做少量最终覆盖，但完整基线仍应保存在部署仓库。字段以 Mapping 仓库根目录的 `config.spec` 为准。

参考实现为 [`service-map-rbnx`](https://github.com/syswonder/service-map-rbnx)。`algo` 省略时使用默认且推荐的 `rtabmap`；也可显式选择 `dlio` 或 `fastlio2`。目前只有 RTAB-Map 路径持续维护。DLIO 和 FAST-LIO2 不保证兼容其最新上游版本；当前参考实现中的 `fastlio2` 还被标记为漂移故障，仅用于诊断和复现。新本体应先使用 RTAB-Map 完成接入和验收，再根据传感器与平台条件评估其他算法。

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

原生二维激光雷达使用 `provider_ids.scan`。三维点云改用 `provider_ids.scan_cloud`，并显式配置 `scan_projection`；投影高度、目标坐标系、量程和本体过滤边界必须依据雷达安装位姿与 Soma 本体描述设置。

自定义行为树保存在部署仓库，并通过 `bt_xml_file` 引用。Nav2 参数文件中对应的行为树路径必须写为 `__ROBONIX_BT_XML__`，Navigation 才会把占位符替换成部署文件的绝对路径。只设置 `bt_xml_file` 不会切换行为树。

全部字段、默认值和弃用项以 Navigation 仓库根目录的 [`config.spec`](https://github.com/syswonder/service-navigation-rbnx/blob/main/config.spec) 为准；Nav2 参数语义参考 [Nav2 配置指南](https://docs.nav2.org/configuration/index.html)。

旧 `params_profile` 仍可用于迁移并产生警告；新部署不得依赖它。

### 6.3 场景服务

Scene 使用共享 `robonix/lifecycle/driver` 接收实例配置。目标清单和配置都写在 `robonix_manifest.yaml` 中：

```yaml
system:
  scene:
    manifest: package_manifest.jetson-native.yaml
    config:
      camera_provider_id: front_camera
      web_port: 50107
```

用于 RGB-D Scene 的提供方必须由同一个 `camera_provider_id` 同时暴露标准 `camera/rgb`、`camera/depth` 和 `camera/intrinsics` 能力。存在多个相机时必须显式绑定，不能把不同提供方的图像、深度和标定混合。

Scene 的相机到地图变换按以下顺序解析：

1. 首先通过 TF2 直接查询世界坐标系到相机光学坐标系的完整变换。这是新本体的主路径；整机 URDF、关节状态和机器人描述原语必须共同发布一棵连通且唯一的 TF 树。
2. TF2 不可用时，才使用 Atlas 找到的 `robonix/service/map/pose`（或 `robonix/service/map/odom`）与 `robonix/primitive/camera/extrinsics` 组合。这是兼容路径。
3. 两条路径都不可用时，Scene 只会进入用于接线排查的退化路径；不得把该结果作为可靠的米制三维投影。

因此，新本体不需要实现旧的 `robonix/system/soma/sensor_extrinsics`，也不应为了 Scene 再维护一份与 TF 树独立的外参。只有准备使用第二条兼容路径的相机原语，才需要暴露 `camera/extrinsics`；其数据必须从同一整机 URDF/TF 或同一份标定结果派生。迁移背景见 [Issue #156](https://github.com/syswonder/robonix/issues/156)。

`camera/extrinsics` 消息的 `header.frame_id` 是父坐标系，通常为 `base_link`；`child_frame_id` 是所选相机的光学坐标系，例如 `camera_color_optical_frame`。`transform` 表示 `T(parent ← child)`，即：

```text
p_parent = R_parent_child · p_child + t_parent_child
```

例如 `header.frame_id=base_link`、`child_frame_id=camera_color_optical_frame` 表示相机光学原点在 `base_link` 中的位姿，并把相机坐标中的点变换到 `base_link`；不能发送它的逆变换。固定相机可以从静态 TF 派生；安装在机械臂等关节链上的相机必须依赖实时 URDF、关节状态和 TF，不能用固定外参代替动态链。

`camera/intrinsics` 返回 ROS `sensor_msgs/CameraInfo`：`header.frame_id` 是相机光学坐标系，x 轴向图像右、y 轴向下、z 轴向前；`width/height` 是标定对应的图像尺寸；`distortion_model` 与 `D` 描述畸变；`K` 是原始图像的 3×3 内参；`R` 是立体校正旋转；`P` 是校正图像的 3×4 投影矩阵。当前 Scene 使用 `K` 中的 `fx/fy/cx/cy` 与 `width/height`，不会自行应用 `D/R/P`，因此输入的 RGB 和度量深度必须已校正并配准到同一 RGB 光学坐标系，分辨率与 `K` 对应。当前 Scene 分别读取两路最新帧，提供方还应保证 RGB 与深度时间近似同步。

因此新本体必须同时满足：Mapping 绑定正确的里程计与传感器提供方并发布地图位姿；所选相机提供方声明 RGB、深度和内参；完整 URDF/TF 树连接 `map`、`base_link` 与相机光学坐标系。只有无法提供完整 TF 查询时，才额外要求同一相机提供方声明外参兼容能力。Soma 保存完整本体模型，坐标变换由机器人描述原语或仿真器中的唯一 TF 发布方发布。

## 7. 按风险分阶段验收

不要第一次就把全部驱动和运动服务一起启动。每一阶段通过后再加入下一组软件包。

### 7.1 静态检查与构建

```bash
rbnx build -f robonix_manifest.yaml
```

该命令适用于不需要额外宿主准备的通用部署。AgileX Ranger Mini v3 应在其仓库根目录执行 `bash build.sh`。

构建前确认每个原语、服务和技能实例只设置了 `path` 或 `url` 中的一项。构建摘要必须满足：

- `Failed: 0`；
- `Skipped: 0`；
- `Built` 等于 `Total`；
- 没有条目解析警告。

当前 `Total` 只统计已解析条目。缺少 `path`/`url` 的条目会被排除，同时设置两者时会优先使用 `path`。因此还要把摘要计数与清单中的原始实例数逐类核对，不能只看命令状态码。

Jetson 原生软件包使用宿主已安装且与 JetPack 匹配的 CUDA、PyTorch、TorchVision、TorchAudio 等系统组件；有些开发套件出厂已带这些组件。先核对 JetPack/L4T 与实际 Python wheel，再参考 [Jetson AI Lab 软件包索引](https://pypi.jetson-ai-lab.io/)；不要照抄另一块板子的 CUDA 索引。软件包自身携带的厂商 SDK 或 ROS 驱动由软件包构建脚本处理。

### 7.2 无运动启动

先创建无运动清单：

```bash
cp robonix_manifest.yaml robonix_manifest.no-motion.yaml
${EDITOR:-vi} robonix_manifest.no-motion.yaml
```

从副本中删除底盘、机械臂和其他可运动硬件的整个实例条目，保留 Atlas、Soma、Vitals、机器人描述原语、只读传感器和已审查的非运动组件。当前部署格式没有通用启停字段；`enabled: false` 或 `disabled: true` 会被忽略，不能用于安全禁用。

若副本保留 Pilot，启动前在当前 shell 中提供模型配置；任一值为空都会使 Pilot 预检失败：

```bash
export VLM_BASE_URL='https://your-provider.example/v1'
export VLM_API_KEY='replace-with-your-key'
export VLM_MODEL='replace-with-your-model'
```

通用部署在终端 A 执行：

```bash
rbnx boot -f robonix_manifest.no-motion.yaml
```

`rbnx boot` 会保持在前台并等待 Ctrl-C 或关闭信号。若部署必须通过包装脚本准备 CAN、RMW 或其他宿主环境，应为无运动清单建立对应包装脚本。脚本必须明确启动 `robonix_manifest.no-motion.yaml`；不要调用仍指向完整清单的 `start.sh`。

保持终端 A 运行，在终端 B 检查提供方状态：

```bash
rbnx caps -v
```

检查原语和服务提供方为 `ACTIVE`；技能在首次调用前通常保持 `INACTIVE`。同时检查提供方 ID 与清单中的 `name` 一致、URDF/TF 连通、传感器坐标系和时间戳正确。失败时查看：

```bash
provider_id=front_camera
rbnx logs -t "$provider_id"
```

启动输出不得包含 `failures`、`[FAIL]` 或 `packages failed to start`。逐条检查 `[SKIP]`：只接受启动器明确说明已由另一组件接管的条目，其他跳过都要查明原因。

原语或技能的解析失败可能记入 skipped 而不是 failure，因此没有 `[FAIL]` 不能单独证明整机通过。建立一份期望提供方清单，逐项对照 `rbnx caps -v` 的提供方 ID、状态和能力约定。

在 ROS 2 终端验证整机 TF。下面的命令应持续输出从底盘到相机的有限数值变换，而不是重复报告 lookup 失败：

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

### 7.3 单设备运动

先停止无运动部署，再为单设备测试建立新清单：

```bash
rbnx shutdown -f robonix_manifest.no-motion.yaml
cp robonix_manifest.no-motion.yaml robonix_manifest.chassis-test.yaml
${EDITOR:-vi} robonix_manifest.chassis-test.yaml
```

每个测试清单只加回一个可运动实例。在硬件安全条件下，依次验证底盘低速前进和停止、机械臂单关节、夹爪开合以及急停；通过后再启用导航。任何提供方在输入超时、进程退出或 `CMD_SHUTDOWN` 时都必须停止其硬件输出。

### 7.4 使用 RViz2 验证地图、定位与导航

RViz2 是本体接入验收工具，不是机器人无头运行时必须常驻的进程。验收时用它同时检查地图、定位、TF、传感器、代价地图、规划路径和机器人 footprint。基础操作参考 [RViz 用户指南](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)，Nav2 的坐标树与导航流程参考 [坐标变换配置](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html)和 [Nav2 入门](https://docs.nav2.org/getting_started/index.html)。

AgileX Ranger Mini v3 在部署仓库中保存 `rviz/ranger_mapping.rviz`，Stack 启动后从仓库根目录运行：

```bash
bash start_rviz2.sh
```

脚本加载 ROS 2 Humble 与 `rmw_zenoh_cpp` 环境，并打开该 RViz 配置；它不会启动 Stack，也不会向机器人发送运动命令。临时选择另一份配置时使用绝对路径：

```bash
ROBONIX_RVIZ_CONFIG=/absolute/path/custom.rviz bash start_rviz2.sh
```

新机器人应在自己的部署仓库保存一份 `.rviz` 配置。至少完成以下检查：

1. 将 Fixed Frame 设为 `map`，在 TF 显示中确认 `map → odom → base_link → sensor frames` 连通；
2. 同时显示 `/map`、LaserScan 或 PointCloud、机器人 footprint、全局/局部代价地图以及全局/局部路径；
3. 加载已有地图后，用 **2D Pose Estimate** 在机器人的实际位置按下并拖动箭头指定朝向，等待 scan 或点云重新贴合地图；
4. 通过 Robonix 导航接口提交一条短距离目标，确认全局路径位于可通行区域、局部路径不穿过障碍、footprint 不覆盖致命代价格，并验证到点停止与取消；
5. 如果要直接从 RViz 下发 Nav2 目标，配置 `nav2_rviz_plugins/GoalTool` 或 Navigation 2 面板。`rviz_default_plugins/SetGoal` 只发布 `/goal_pose`，没有对应 bridge 时不能驱动 `/navigate_to_pose` action。

![Ranger Mini v3 联调时的 RViz2 窗口效果。该图仅说明界面布局；实际显示项和话题以机器人部署仓库中的 RViz 配置为准。](/img/integration/ranger-rviz-overview.webp)

### 7.5 保存地图并标注房间

建图期间先确认机器人静止时 scan 或点云与墙面重合，旋转时 TF 和定位不漂移。空间地图稳定后，打开 `http://<ROBOT_IP>:50107/user`；如果修改了 `system.scene.config.web_port`，使用实际端口。

1. 在 **Map ID** 输入唯一标识，点击 **Save current**。操作窗口关闭前不要刷新页面或继续编辑；第一次保存会写入空间地图和 Scene 数据；
2. 点击 **Annotate room**，沿房间边界依次单击至少三个角点，最后双击完成多边形；
3. 输入房间或区域名称后点击对话框中的 **OK**；
4. 继续使用同一个 Map ID 点击 **Save current**。已有空间地图不会重建，本次只更新房间和 Scene 对象；
5. 重启 Stack 后，在保存列表中点击该地图的 **Load**。加载窗口完成后，确认 occupancy、房间和 Scene 对象都恢复；定位偏移时再使用页面的 **Pose estimate** 或 RViz2 的 **2D Pose Estimate**。

![Scene 的 Map & rooms 页面。加载空间地图后，可在地图上绘制并命名房间或区域。](/img/integration/scene-room-annotations.webp)

### 7.6 Client 与健康状态联调

完成地图与导航验收后，依次验证 Client 文本任务、相机画面理解、语音输入/输出和停止操作，并确认 Vitals 能区分本体状态、提供方状态与故障。客户端启动和音频桥接配置见[客户端使用](../getting-started/client.md)。

地图文件和联调环境属于测试数据。验收后只删除本次创建的临时 map id，不要清理操作者已有地图目录。

**整机完成标志：** 一台新机器克隆部署仓库、设置文档列出的环境变量并运行构建和启动后，可以复现相同的能力提供方列表、Soma/TF、传感器、地图、导航与客户端链路；不依赖未记录的绝对路径或手工后台进程。

清单中的 `branch: main` 只适合开发阶段跟踪上游。修改 `branch` 或标签后，已有缓存不会自动切换检出；执行以下命令后重新构建：

```bash
rbnx clean -f robonix_manifest.yaml --cache
```

验收与发布时改用软件包仓库提供的不可变发布标签，并把 `rbnx-boot/cache/` 中每个仓库实际的 `git rev-parse HEAD` 记录到验收材料。`rbnx` 当前不支持在 `branch:` 中直接填写任意提交哈希。

## 8. 发布和持续维护

原语、服务和技能各自提交到独立社区仓库；机器人部署仓库只组装它们并保存机器人专属配置。提交格式和软件包目录流程见[软件包目录发布流程](package-catalog.md)。

发布前至少确认：

- 软件包清单与机器人目录元数据完整，许可证为有效 SPDX 标识；
- 每个公开 `config` 字段在根目录 `config.spec` 中有解释；
- README 写明系统依赖、硬件连接、构建/启动、权限和安全限制；
- 软件包在它声明支持的目标平台上通过 `rbnx validate`、构建、启动和关闭；
- 提交机器人目录条目时，`assets/robot.jpg` 已压缩为适合网页预览的尺寸；
- 维护者、默认分支与软件包目录记录一致。

新设备接入后的参数变化优先提交到对应机器人部署仓库；可复用的驱动修复提交到原语仓库；标准接口变更才进入 Robonix 主仓库。这样其他本体可以升级软件包，而不会继承某台机器的私有参数。
