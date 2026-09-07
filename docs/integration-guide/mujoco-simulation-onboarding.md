# MuJoCo 仿真本体接入 Robonix 指南

这份指南面向已经取得厂家模型或驱动资料、准备把 MuJoCo 仿真机器人接入 Robonix 的
开发者。接入工作以复用现有模型、控制器和 ROS 接口为主，再补充目标本体包所需的加载、
桥接和 Robonix 能力约定（Capability）配置。本文不讨论从 CAD 开始重建机器人动力学模型。

文中的文件结构参考 Robonix 社区发布的
[`Ranger Mini V3 + Piper`](https://packages.robonix.ai/robots/robonix.robot.agilex.ranger_with_piper_mujoco/)
本体包。所有相对路径均以该本体包的仓库根目录为基准。

完成本文后，机器人应当能够由 `start.sh` 独立启动 Web 或 Native MuJoCo 仿真，由
`rbnx boot` 加载同一套本体包，并通过 Robonix 原语（Primitive）、服务（Service）和
技能（Skill）完成移动、感知或操作任务。

开始前应准备好下列资料：

- 厂家提供的可运行 MuJoCo 工程，或 URDF/Xacro、mesh 与 ROS 2 驱动；
- 厂家的接口说明、关节限制、初始姿态和许可证；
- 一个可以正常构建、启动的 Robonix 本体包仓库。

---

## 接入工作的目标

用户在 Robonix 中输入：

```text
探索当前房间并建图。
```

或者：

```text
导航到桌边，抓起桌上的水杯，然后安全放下。
```

要让这些命令真正执行，系统需要形成以下闭环：

```text
rbnx chat
    │
    ▼
Robonix 对任务进行理解和能力编排
    │
    ├── Mapping / Navigation / Scene / Explore / Pick
    │
    ▼
Robonix 原语
    │  将“底盘、相机、雷达、机械臂”等设备包装成标准能力
    ▼
ROS 2 topic / action / TF
    │
    ▼
共用的仿真 Bridge 和消息协议
    │
    ├── Web MuJoCo：浏览器、WASM、JavaScript 控制适配器
    └── Native MuJoCo：Python/C++ 进程、原生控制适配器
    │
    ▼
同一套 MJCF、执行器语义、物理状态和传感器结果
    │
    └──────────────反馈到 Robonix
```

在目标本体包中，XML 能够加载只是第一步。一次完整接入还包括：

- 模型如何加载；
- 厂家控制输入如何接收；
- 仿真状态如何转成 ROS 2；
- TF、URDF 和传感器是否一致；
- Robonix 如何发现能力；
- Mapping、Nav2 或抓取技能所需的数据是否完整。

---

## MuJoCo 基础

接入已有模型时，不必成为 MuJoCo 建模专家，但需要能读懂厂家模型中的关键接口。

一个 MuJoCo XML 通常包含：

| 元素 | 接入时主要关注什么 |
| --- | --- |
| `<worldbody>` / `<body>` | 机器人刚体层级、部件坐标和安装关系 |
| `<joint>` / `<freejoint>` | 可运动关节、关节名称、轴、范围和根自由度 |
| `<geom>` | 视觉模型、碰撞模型、摩擦、质量来源 |
| `<inertial>` | 质量、重心和惯量 |
| `<actuator>` | 厂家控制器最终写入的控制通道 |
| `<sensor>` | IMU、编码器、力传感器等原生传感器 |
| `<site>` | LiDAR、TCP、IMU、相机光心等参考坐标 |
| `<camera>` | MuJoCo 相机名称、位置、方向和视场角 |
| `<asset>` | mesh、纹理、材质以及子模型 |
| `<default>` | actuator、joint、geom 的公共参数 |
| `<keyframe>` | 厂家定义的初始状态或参考姿态 |

## 先确定 MuJoCo 运行时

本文所说的 Web MuJoCo，是指通过 WebAssembly（WASM）在浏览器中运行 MuJoCo；Native
MuJoCo 是指通过官方 Python/C++ 库在本机进程或容器中运行 MuJoCo。两者使用相同的 MJCF
模型语义和物理引擎，但加载资产、执行控制器、渲染画面和管理进程的方式不同。

| 对比项 | Web MuJoCo | Native MuJoCo |
| --- | --- | --- |
| 物理状态持有者 | 浏览器中的 MuJoCo WASM | Python/C++ 原生进程 |
| 模型加载 | 将 XML、mesh 和纹理复制到 WASM 虚拟文件系统 | 从本地或容器文件系统加载 XML 和资产 |
| 控制适配器 | JavaScript，或通过网络调用外部控制进程 | Python/C++ 中直接调用厂家控制器或策略 |
| 交互界面 | 浏览器页面，便于增加面板、远程访问和自定义视觉层 | `mujoco.viewer` 或自建原生界面，适合本机调试 |
| RGB 渲染 | WebGL/Three.js 或浏览器中的自定义渲染器 | `mujoco.Renderer` 或原生 OpenGL viewer |
| 进程依赖 | Node.js、浏览器、WebGL、WASM 资产 | MuJoCo Python/C++、系统 OpenGL，viewer 还需要 X11/Wayland/WSLg |
| 厂家原生代码复用 | C++/Python 控制器通常要改写或放到外部进程 | 可直接复用的机会更大 |
| 无界面运行 | 需要无头浏览器或专门的运行页 | 可不启动 viewer；是否仍需 GPU 取决于 RGB 传感器实现 |
| 调试重点 | 浏览器生命周期、WASM 内存、WebSocket 和 WebGL | 动态库、Python/C++ 依赖、OpenGL 上下文和进程生命周期 |

无论选择哪种运行时，以下内容应保持一致：

- 机器人和环境使用的 MJCF，以及 joint、actuator、body、site、camera 名称；
- 底盘、机械臂、急停、重置等命令的含义、单位和限幅；
- 里程计、关节状态、LiDAR、相机、IMU 和抓取状态的消息结构；
- ROS 2 topic、TF、QoS 和仿真时间；
- Robonix 原语、服务、技能、`robonix_manifest.yaml` 和 `soma.yaml`；
- 同一组端到端验收任务。

不必强行共用控制器源码和渲染代码。Web 控制器围绕 WASM 数组和 JavaScript 生命周期
编写；Native 控制器直接操作原生 `MjModel`、`MjData` 和厂家 Python/C++ API。更合适的
共用边界是 Bridge 消息协议，而不是控制器内部类。

参考本体包目前允许通过 `sim/start.sh --backend web|native` 选择后端。Web 后端支持
Mesh 和 SPZ；Native 后端只支持 Mesh，选择 SPZ 会在加载模型前报错。这个限制来自当前
SPZ 渲染链路只接入了浏览器，并不是 MuJoCo 物理引擎本身的限制。若要让 Native 后端显示
Gaussian Splatting，需要额外集成原生 GS 渲染器，并同步 MuJoCo 相机、时间和坐标变换；
碰撞、LiDAR 和深度仍应来自 MJCF 几何。

参考实现中的 MuJoCo 物理步进使用 CPU，Native viewer 和 RGB 离屏相机使用硬件 OpenGL。
因此 `--headless` 只关闭 viewer，仍发布 RGB 时仍需要 GPU 上下文；启动脚本默认拒绝
`llvmpipe`、`softpipe` 等软件 renderer。其他项目若不发布 RGB，可以根据自己的传感器
实现放宽这项要求。

选择运行时可以遵循以下原则：

- 厂家已经提供 Python/C++ MuJoCo 控制器、MPC、强化学习策略或原生插件时，优先 Native；
- 需要浏览器交互、远程展示、SPZ 视觉或已有 Web 前端时，选择 Web；
- 同一仿真包需要兼顾展示和算法验证时，保留双后端，并让二者共用 Bridge 协议和 ROS 验收；
- 不需要双后端时，只实现一种即可，不要为了形式一致复制两套未经测试的控制逻辑。

## 先盘点厂家提供了什么

下面两种厂家交付形式有相对清晰的接入边界：

1. 厂家提供可独立运行的 MuJoCo XML/MJCF、模型资产和运动控制实现；
2. 厂家提供 URDF/Xacro、mesh、完整 ROS 2 驱动和控制接口说明。

其他仿真平台工程、SDK、强化学习 checkpoint、CAD/mesh 或通信协议通常还缺少 MuJoCo
动力学模型、控制语义或 ROS 2 接口。遇到这类资料时，先向厂家补充索取；仍有缺项再
根据具体机器人单独处理。

## 根据厂家资产选择接入路线

```text
厂家有可运行 MuJoCo 工程吗？
    ├── 有：路线 A，包装现有 MJCF 和控制器
    └── 没有
         │
         ├── 有 URDF/Xacro + ROS 2 驱动：路线 B，转换模型并保持厂家 ROS 接口
         └── 其他情况：先向厂家补充索取，必要时请求大模型或专业人员协助
```

如果厂家同时提供两种资产，采用顺序如下：

1. 官方且可运行的 MuJoCo 工程；
2. 官方 URDF + 官方控制器/驱动；

社区模型可以作为补充，但在使用前要核对关节、质量、控制器接口和许可证。

---

## 路线 A：厂家已经提供 MuJoCo XML 或完整仿真

厂家已有可运行的 MuJoCo 工程时，保留其中的模型和运动实现，在外层补充本框架的注册
和协议适配即可。

### 保留厂家原始文件

厂家包保留为只读上游，在外层建立项目自己的包装目录：

```text
assets/robots/my_robot/
├── robot.json
├── index.json
├── robot_wrapper.xml
├── controller.js                 # Web 后端需要时提供
└── vendor/
    ├── LICENSE
    ├── robot.xml
    ├── controller.py
    ├── meshes/
    └── textures/

sim/native/
├── controller_my_robot.py        # Native 后端需要时提供
└── runtime.py                    # 选择并运行对应 Native 控制器
```

`robot_wrapper.xml` 用于引用厂家模型，`vendor/robot.xml` 保持原样，方便后续升级和
对比。

可以使用 `<include>`、MuJoCo `<model>/<attach>` 或小范围自动补丁：

- 厂家模型本身就是完整根机器人时，优先 include；
- 把官方机械臂挂到底盘时，优先 model/attach；
- 需要增加传感器支架或固定安装变换时，优先 wrapper；
- 名称冲突、路径或 group 无法在外层处理时，再维护明确的补丁文件。

### 按运行时复用厂家控制器

#### 厂家已经提供 Native MuJoCo 控制器

厂家工程本身使用 Python/C++ MuJoCo 时，优先保留原生实现。Native runtime 持有
`MjModel` 和 `MjData`，在每个仿真步调用厂家控制器，再把状态和传感器数据转成共用的
Bridge 消息：

```text
Native MuJoCo runtime
    ├── 加载厂家 MJCF 和资产
    ├── 调用厂家 Python/C++ 控制器
    ├── 写入 actuator 并执行 mj_step
    ├── 读取状态和传感器
    └── 通过 WebSocket 与 ROS 2 Bridge 交换消息
```

这种方式通常不需要把厂家控制算法翻译成 JavaScript，但仍要包装以下边界：

- 将 Bridge 命令映射到厂家控制器输入；
- 将厂家状态转换为统一状态结构；
- 统一控制频率、watchdog、急停和 reset；
- 隔离厂家 Python、CUDA 或动态库依赖；
- 在无 viewer 模式下保持物理、传感器和 Bridge 正常运行。

#### 控制器可以在浏览器 JavaScript 中实现

适合简单轮式运动学、位置控制机械臂或纯数学控制器。把厂家算法翻译或封装为 JS 后，
需要做以下对比：

- 保留公式和参数来源；
- 用同样输入输出做离线对比测试；
- 不擅自改变控制频率；
- 明确浮点精度差异；
- 对照厂家 demo 验证同一命令下的轨迹。

Web runtime 自己持有 `MjData`，因此控制器可以直接读写 WASM 数组。厂家控制器使用了
原生 MuJoCo 回调、动态库或 Python API 时，不应机械翻译；此时改用 Native 后端，或者
把控制器放到外部进程。

#### 控制器作为独立进程运行

如果必须使用 Web viewer，同时厂家控制器又依赖 Python/C++、MPC、RL policy 或专用库，
可以把控制器作为独立进程：

```text
浏览器 MuJoCo
    │ 状态/观测
    ▼
WebSocket Bridge
    │
    ▼
厂家控制进程（独立 Python/C++/CUDA 环境）
    │ actuator target
    ▼
浏览器控制器写入 data.ctrl
```

厂家控制进程可以运行在单独容器或 venv 中。Bridge 需要定义：

- 观测消息结构；
- action 消息结构；
- 控制频率；
- action 超时；
- reset policy state；
- 丢帧和断连处理；
- 模型版本和归一化参数校验。

依赖特定 CUDA/PyTorch 的策略放在独立环境中运行，避免改变 Robonix 或 ROS Bridge 的
基础环境。

Native 后端也可以把策略拆成独立进程，但厂家控制器能够在 Native runtime 的依赖环境中
直接运行时，进程内调用通常更简单，时延和状态同步也更容易控制。

#### 厂家已经提供 ROS 2 仿真驱动

如果厂家仿真已经发布标准 ROS 2 topic，适配版继续沿用这些接口。具体做法包括：

- 让厂家仿真节点继续运行；
- 本体包 Bridge 只补充厂家驱动缺失的数据；
- Robonix primitive 直接绑定厂家 topic；
- 用 namespace/remap 解决名称冲突；
- 避免把同一状态重复发布两次。

### 包装成框架机器人目录

双后端项目可以让 `robot.json` 同时保存模型入口、运行资产和传感器配置。Web loader 用它
构建 WASM 场景；Native scene builder 读取同一份配置，在临时目录中组装 XML 和资产：

```json
{
  "schemaVersion": 1,
  "id": "my_robot",
  "label": "Vendor My Robot",
  "description": "Vendor-provided MuJoCo robot adapted for Web and native runtimes",
  "model": "robot_wrapper.xml",
  "dragRootBody": "vendor_base",
  "files": "index.json",
  "controller": {
    "module": "controller.js",
    "export": "MyRobotAdapter"
  },
  "controlledActuators": [
    "vendor_left_wheel",
    "vendor_right_wheel"
  ],
  "sensors": {}
}
```

`index.json` 是运行资产清单。Web 后端把其中的 XML、mesh 和纹理复制到 MuJoCo MEMFS；
参考本体包的 Native scene builder 则把同一批文件复制到临时场景目录，再调用
`MjModel.from_xml_path`。`controller.js` 通过 `robot.json.controller.module` 单独加载，
不要求列入该索引；Native 控制器由 Python runtime 按机器人 ID 选择，也不属于模型资产。

然后把机器人 ID 加入：

```text
assets/robots/index.json
```

### 编写薄控制适配器

适配器的目标不是替代厂家控制器，而是完成协议转换。Web 后端可以实现 JavaScript
适配器：

```javascript
export class MyRobotAdapter extends BaseController {
  async initialize(model, data, mujoco) {
    // 找到厂家 actuator、joint 和 body ID
    // 初始化或连接厂家控制后端
    // 验证模型版本和必需名称
  }

  setTwist(command) {
    // 把 Robonix/ROS Twist 转为厂家控制器的目标输入
  }

  async step(keys, model, data, mujoco) {
    // 读取最新厂家 action；超时则进入安全状态
    // 将厂家控制输出写入对应 actuator
  }

  getRobotState(model, data) {
    // 将厂家模型状态整理为 Bridge 协议
  }

  emergencyStop() {
    // 调用厂家急停或发送安全零命令
  }
}
```

Native 后端实现对应的 Python/C++ 适配器，内部接口可以不同，但对 Bridge 的行为应一致：

```python
class MyNativeController:
    def command(self, message):
        """接收 cmd_vel、关节、TCP、急停或 reset 命令。"""

    def step(self):
        """调用厂家控制器，并在 mj_step 前写入 actuator。"""

    def state(self):
        """返回与 Web runtime 相同语义的底盘、机械臂和物体状态。"""

    def reset(self):
        """恢复确定的初始状态，并清理控制器内部状态。"""
```

如果同时支持两个后端，应为同一组命令建立对照测试，确认速度方向、单位、关节顺序、
限幅、急停、初始姿态和抓取结果一致。两套控制器不要求逐步产生完全相同的浮点轨迹，
但必须满足相同的安全约束和任务验收标准。

目标本体包的适配器还需要保留以下运行保护：

- 名称存在性检查；
- 控制范围检查；
- watchdog；
- 急停；
- 确定性 reset；
- 上游版本校验；
- 错误日志；
- 不可达命令的明确拒绝。

---

## 路线 B：厂家提供 URDF/Xacro 和 ROS 2 驱动

URDF 是很有价值的起点，但它通常只完整描述 ROS 运动学、视觉和碰撞树，不一定包含
足够的 MuJoCo 动力学与控制信息。

### URDF 中可以直接复用的内容

- link/joint 层级；
- joint axis、limit 和 origin；
- mesh 和材质路径；
- collision 几何；
- inertial，如果厂家认真提供；
- transmission 和 ros2_control 接口；
- sensor frame；
- robot_state_publisher 使用的命名；
- 真实驱动的 JointState 顺序。

### 转换过程

1. 使用厂家推荐方式展开 Xacro，得到固定 URDF；
2. 解析所有 `package://` 路径并收集实际 mesh；
3. 使用当前 MuJoCo 版本支持的 URDF 导入或可靠的[转换工具](https://github.com/xiongy26/urdf2mjcf/tree/main)生成初始 MJCF；
4. 将转换结果作为初始模型，继续核对动力学和控制接口；
5. 对照 URDF 检查 link/joint/axis/origin/limit；
6. 对照实物参数检查质量和惯量；
7. 从厂家 ROS 驱动、控制 demo 或手册补齐 actuator 语义；
8. 为移动底座确认根 freejoint 与地面接触；
9. 为闭环机构、mimic、传动和夹爪单独验证；
10. 用厂家 ROS 接口作为 Bridge 和 primitive 的目标语义。

转换得到的 MJCF 可以同时供 Web 和 Native 后端使用。需要分别验证浏览器 WASM 使用的
MuJoCo 版本与 Native Python/C++ 使用的版本，尤其是 `<compiler>`、插件、mesh 格式、
执行器默认值和接触参数。厂家 ROS 2 驱动位于上层接口，不应因为选择 Web 或 Native
运行时而改变 topic、frame 和 action 语义。

### 保持 ROS 接口一致

如果厂家真实驱动已经提供：

```text
/cmd_vel
/odom
/joint_states
/arm_controller/follow_joint_trajectory
/camera/color/image_raw
/scan
```

仿真端提供相同或可 remap 的接口后：

- Robonix primitive 可以同时适配仿真和真实机器人；
- Mapping/Nav2 参数更容易复用；
- 从仿真切换到真机时上层能力变化更小；
- 可以对比相同命令在真机和仿真中的表现。

---

## 资料不完整时的处理

如果厂家只提供 Gazebo、Isaac Sim、Webots、PyBullet、SDK、强化学习 checkpoint、CAD、
mesh 或通信协议，暂时无法进入目标本体包的注册步骤。这些资料缺少的部分因机器人而异，无法
用同一套转换规则补齐。

可以先向厂家索取以下任一组合：

- 可运行的 MuJoCo 模型、依赖资产、控制器、启动示例、版本和许可证；
- URDF/Xacro、mesh、质量惯量、碰撞、ROS 2 驱动、控制接口、TF 和启动示例。

厂家无法补充时，可以把厂家仓库、版本、README、模型文件、控制 demo、ROS graph、
正常运行日志和许可证交给熟悉该本体的开发者分析，也可以借助大模型梳理文件和接口。
提问时先让其列出已有资料和缺项，再讨论转换方案；质量、惯量、关节轴、控制增益、安装
位置和策略观测顺序仍以厂家资料为准。缺少这些关键参数时，暂时停止集成更稳妥。

---

## 将厂家模型注册到仿真框架

完成路线 A 或路线 B 的模型准备后，需要把它包装为自包含的仿真机器人包。

### 目录结构

```text
assets/robots/<robot_id>/
├── robot.json
├── index.json
├── robot_wrapper.xml
├── controller.js                 # Web 后端控制适配器，可选
├── UPSTREAM.md
├── LICENSES/
└── vendor/
    ├── robot.xml
    ├── meshes/
    └── textures/
```

如果许可证不允许重新分发厂家模型，公开仓库中只保留：

- 下载脚本；
- 固定版本或校验值；
- 用户接受厂家许可的步骤；
- 构建时复制到预期目录的脚本；
- 缺失资产时的明确错误提示。

### `index.json`

它列出运行时加载模型所需的资产：

```json
[
  "robot_wrapper.xml",
  "vendor/robot.xml",
  "vendor/meshes/base.stl",
  "vendor/textures/base.png"
]
```

要求：

- 路径相对于机器人目录；
- 不允许绝对路径和 `..`；
- index 需要覆盖所有 include、mesh 和 texture；
- checkpoint、日志和源 CAD 不属于运行资产；
- `controller.js` 由 ES module 路径加载，不要求列入该索引。

双后端本体包可以让 Web loader 和 Native scene builder 共用该清单。前者复制到 WASM
虚拟文件系统，后者复制到临时文件系统；这样能够避免维护两份容易漂移的资产列表。

### `robot.json`

```json
{
  "schemaVersion": 1,
  "id": "my_robot",
  "label": "Vendor My Robot",
  "description": "Vendor robot adapted to Web and native MuJoCo runtimes",
  "model": "robot_wrapper.xml",
  "dragRootBody": "vendor_base",
  "files": "index.json",
  "controller": {
    "module": "controller.js",
    "export": "MyRobotAdapter"
  },
  "controlledActuators": [],
  "sensors": {
    "lidar": null,
    "imu": null,
    "cameras": []
  }
}
```

`controlledActuators` 填写本适配器实际拥有的 actuator，防止其他控制逻辑同时写入。
其中 `controller` 是 Web loader 使用的字段。Native runtime 还需要维护机器人 ID 到
Python/C++ 控制器类的映射；如果项目只支持 Native，可以采用更简单的原生注册表，但
模型入口、资产清单和传感器配置仍应只有一个权威来源。

### 根注册表

将 ID 加入：

```text
assets/robots/index.json
```

`defaultRobot` 决定默认加载哪个机器人。新增机器人时不一定要修改默认值。

### 参考框架中的关键加载文件

| 文件 | 作用 |
| --- | --- |
| `src/utils/RobotRegistry.js` | 校验并注册 `robot.json` |
| `src/utils/RobotLoader.js` | 将模型资产复制到 MuJoCo MEMFS |
| `src/utils/SceneManager.js` | 组合环境、机器人和动态物体 |
| `src/utils/controllers/BaseController.js` | 控制器适配器基础接口 |
| `src/utils/KeyboardControl.js` | 加载机器人 controller，并转发外部命令 |
| `sim/native/scene_builder.py` | 在本地文件系统组装环境、机器人和动态物体 |
| `sim/native/runtime.py` | 持有原生 `MjModel`/`MjData`、步进仿真并收发 Bridge 消息 |
| `sim/native/controller.py` | Native 控制适配器和状态提取 |
| `sim/native/sensors.py` | Native LiDAR、RGB-D 和 IMU 实现 |

---

## 控制器适配

### Web 控制适配器需要暴露什么

当前前端控制层会调用：

```text
initialize(model, data, mujoco)
reset(model, data)
step(keyStates, model, data, mujoco)
setExternalControlEnabled(enabled)
setTwist(command)
setArmJointCommand(command, model, data)       # 有机械臂时
setArmPoseCommand(command, model, data)        # 有 TCP 控制时
startPickObject(name, model, data)              # 使用当前抓取后端时
emergencyStop()
getRobotState(model, data)
getControlKeys()
getDescription()
```

机械臂和抓取方法按本体能力实现。其他类型的机器人可以扩展 Bridge 协议，不需要填充
无意义的机械臂字段。

### Native 控制适配器需要暴露什么

参考本体包中的 Native runtime 调用：

```text
command(message)
step()
state()
reset()
pick_status                         # 有抓取状态机时
```

`command` 接收 Bridge 下发的命令，`step` 在每次 `mj_step` 前更新执行器，`state` 输出
底盘、机械臂和物体状态。Native runtime 还负责仿真时钟、消息发布频率、viewer 同步和
进程退出。若厂家控制器使用不同的方法名，在这一层做薄包装即可，不要为了与 Web 类接口
相同而改写厂家代码。

### 两个后端共用的消息边界

参考实现中，两个 runtime 都连接同一个 WebSocket Bridge。常用的下行消息包括
`cmd_vel`、`arm_joint_command`、`arm_pose_command`、`pick_object`、
`emergency_stop` 和 `reset`；上行消息包括 `hello`、`state`、`scan`、`pointcloud`、
`camera`、`pick_status` 和 `command_ack`。

`hello` 至少应携带 backend、environment、robot 和 visual mode，便于健康检查确认真正
连入的是预期运行时。新增机器人类型时可以扩展消息，但 Web 和 Native 必须同时升级协议
版本、Bridge 解析和验收测试。

### 厂家命令到统一命令的映射

先建立表格：

| Robonix/ROS 输入 | 厂家输入 | 适配方式 |
| --- | --- | --- |
| `Twist.linear.x` | `target_velocity[0]` | 单位和符号转换 |
| `Twist.linear.y` | 不支持 | 明确拒绝或置零 |
| `Twist.angular.z` | `target_yaw_rate` | 限幅 |
| JointState | 厂家关节数组 | 按名称重排 |
| TCP Pose | 厂家 IK 接口 | frame 转换后调用 |
| estop | 厂家 disable | 立即调用并清空缓存 |

关节映射以名称为准，并在初始化阶段核对集合和顺序。

### 厂家状态到 Bridge 状态的映射

当前移动操作机器人 Bridge 需要：

```javascript
{
  base: {
    position: [x, y, z],
    quaternion: [w, qx, qy, qz],
    linearVelocity: [vx, vy, vz],
    angularVelocity: [wx, wy, wz]
  },
  arm: {
    names: [],
    positions: [],
    velocities: [],
    endPose: null
  },
  objects: []
}
```

四足、人形、双臂或无人机需要先扩展 WebSocket protocol，再增加对应的 Bridge
publisher。

---

## 传感器如何接入

### 使用厂家定义的安装位置

如果厂家 URDF 或仿真工程已经提供相机、LiDAR 和 IMU frame，沿用其中的：

- frame 名；
- 父 link；
- xyz/rpy；
- 量程；
- FOV；
- 图像尺寸；
- 扫描频率。

如果厂家模型没有传感器，可根据真实安装图、CAD 或标定文件增加包装 body/site。仅有
照片时无法可靠确定安装位姿；临时估计值需要在文档中标明，并在取得标定结果后替换。

### 参考框架支持的通用传感器

Web 后端的 `src/utils/RobotSensorSuite.js` 和 Native 后端的
`sim/native/sensors.py` 都可以根据 `robot.json` 生成：

- LiDAR 3D 点；
- 2D LaserScan；
- MuJoCo gyro/accelerometer IMU；
- RGB 相机；
- 几何深度相机。

一个传感器需要同时出现在：

1. 厂家或 wrapper MJCF 的 site/camera/sensor；
2. `robot.json.sensors`；
3. Web `MujocoBridgeClient.js` 或 Native runtime 的发送协议；
4. `sim/bridge/bridge_node.py` 的 ROS publisher；
5. 对应 primitive 的配置和 capability；
6. URDF/TF；
7. 测试脚本。

两个后端应共用传感器安装位姿、量程、FOV、frame 和发布频率，但底层实现可以不同：

| 数据 | Web 后端 | Native 后端 |
| --- | --- | --- |
| LiDAR/几何深度 | WASM 中调用 MuJoCo 射线查询 | Python/C++ 中调用 `mj_multiRay` |
| IMU | 读取 MuJoCo sensor data | 读取原生 `sensordata` |
| RGB | Three.js/WebGL 或浏览器渲染链路 | `mujoco.Renderer`/OpenGL |

RGB 图像不要求逐像素相同，因为渲染器、材质和光照实现可能不同；相机内外参、画面方向、
可见范围和时间戳必须一致。LiDAR 与深度应命中同一碰撞分组，否则 Mapping、Navigation
在两个后端会表现不同。

### 厂家已有传感器 ROS 驱动时

如果厂家仿真已经发布相机或 LiDAR ROS topic，可以让 primitive 直接消费厂家 topic，
避免再由 Web 或 Native runtime 重复模拟。需要确认：

- topic 中数据确实来自当前 MuJoCo 场景；
- 时间戳使用同一 `/clock`；
- frame 与本体 TF 连通；
- QoS 兼容；
- 没有两个节点重复发布同名 topic。

### 碰撞过滤

本体 LiDAR 和深度相机默认排除底盘、机械臂和传感器支架，否则地图会出现跟随机器人
移动的障碍。当前框架用 geom group mask 排除机器人 group 4，同时保留环境 group 3
和任务物 group 5。

厂家碰撞分组不同，需要修改 mask 或加载时重分组，并验证不会破坏接触。

---

## ROS 2 Bridge 如何适配厂家接口

`sim/bridge/bridge_node.py` 是 MuJoCo runtime 与 ROS 2 的边界。Web 浏览器和 Native
进程都作为 runtime 连接它；Bridge 根据 `hello.backend` 记录当前后端，但向 ROS 2 发布
相同的 topic、TF 和时钟。它不是完全通用的任意机器人 Bridge，当前 topic 和部分 frame
面向 Ranger/Piper，接入新本体时需要明确修改。

### 建立 ROS 接口清单

接口名称和消息类型尽量与厂家真实驱动一致：

| 功能 | 常见 ROS 2 接口 |
| --- | --- |
| 底盘命令 | `geometry_msgs/Twist` |
| 里程计 | `nav_msgs/Odometry` |
| TF | `/tf`、`/tf_static` |
| 2D LiDAR | `sensor_msgs/LaserScan` |
| 点云 | `sensor_msgs/PointCloud2` |
| IMU | `sensor_msgs/Imu` |
| RGB/Depth | `sensor_msgs/Image` |
| 相机内参 | `sensor_msgs/CameraInfo` |
| 关节反馈 | `sensor_msgs/JointState` |
| 关节轨迹 | `control_msgs/FollowJointTrajectory` action 或厂家接口 |
| TCP 目标 | `PoseStamped` 或厂家 action/service |

### 需要同步修改的位置

修改 topic 或 frame 时需要同步检查：

1. Web `MujocoBridgeClient.js`；
2. Native `runtime.py`、controller 和 sensors；
3. `bridge_node.py`；
4. primitive `config.spec`；
5. primitive Python 实现；
6. `robonix_manifest.yaml`；
7. `soma.yaml`；
8. URDF；
9. Mapping/Nav2 参数；
10. 两个后端的 acceptance 测试。

### TF 设计

移动机器人常见结构：

```text
map
└── odom
    └── base_link
        ├── lidar_link
        ├── imu_link
        ├── front_camera_optical_frame
        └── arm_base_link
            └── ...
```

- `map -> odom` 通常由 SLAM/定位服务发布；
- `odom -> base_link` 来自仿真或厂家里程计；
- 固定传感器使用静态 TF；
- 运动关节由 JointState + robot_state_publisher 或 Bridge 动态 TF 提供；
- ROS URDF 沿用厂家真实本体的 frame 命名。

### 仿真时钟和 QoS

所有仿真节点统一使用 `/clock` 和 `use_sim_time: true`。传感器通常使用 best-effort；
静态变换、相机内参和地图常需要 reliable/transient-local。topic 存在但没有数据时，先
检查 QoS、时间戳和 frame，而不是立刻修改算法。

---

## 将 ROS 接口包装成 Robonix 原语

### 复用真实本体包的能力约定

如果 Robonix 已有同一真实机器人的本体包，可以直接对照它：

- provider 如何拆分；
- capability 名称；
- `config.spec`；
- `soma.yaml` 组件；
- ROS topic；
- service 依赖；
- capability 描述和限制。

仿真包和真实包可以对上层暴露相同 capability，把差异留在 provider 的底层驱动和
启动方式中。

### 原语的合理粒度

通常按设备/驱动边界拆分：

- chassis；
- 每个独立相机；
- LiDAR；
- IMU；
- arm + gripper；
- 厂家专属 locomotion controller。

primitive 不随环境变化，办公室和厨房共用同一个本体 provider。

### 软件包结构

```text
primitives/<device>/
├── package_manifest.yaml
├── config.spec
├── CAPABILITY.md
└── 实现源码或对 common 实现的启动引用
```

示例：

```yaml
manifestVersion: 1
package:
  name: com.example.sim.my_chassis
  version: 0.1.0
  description: Simulated chassis adapter for Vendor My Robot.
  tags: [primitive, chassis, simulation, mujoco]
  maintainers:
    - Your Name <you@example.com>
  license: Apache-2.0

build: bash ../../scripts/build-primitive.sh primitives/my_chassis
start: bash ../../scripts/start-primitive.sh primitives/my_chassis primitives.common.chassis my_chassis
stop: bash ../../scripts/stop-primitive.sh my_chassis

capabilities:
  - name: robonix/primitive/chassis/move
  - name: robonix/primitive/chassis/odom
  - name: robonix/primitive/chassis/twist_in
  - name: robonix/primitive/chassis/driver
```

### 初始化哨兵

仿真 provider 在 `on_init` 阶段等待第一帧真实数据，例如 `/odom`、`/scan` 或
JointState。Web/Native runtime 或厂家控制器没有运行时，provider 返回初始化失败，
而不是显示 ACTIVE。

### `CAPABILITY.md`

内容包括：

- 能做什么；
- 输入输出单位和 frame；
- 使用的是厂家原始接口还是兼容接口；
- 支持 Web、Native 还是两个后端，以及不支持的视觉模式；
- 何时使用；
- 哪些任务需要改用 Navigation 或 Skill；
- 超时、急停和失败行为；
- 与真实本体包有哪些差异。

---

## 配置 `robonix_manifest.yaml`

部署清单把 primitive、service 和 skill 组成完整本体。

```yaml
manifestVersion: 1
name: my-robot-mujoco-deploy

catalog:
  name: robonix.robot.vendor.my_robot_mujoco
  version: 0.1.0
  description: Vendor My Robot MuJoCo body package for Robonix.
  license: Apache-2.0
  tags: [robot, deploy, simulation, mujoco]
  maintainers:
    - Your Name <you@example.com>

env:
  ROS_DOMAIN_ID: "0"
  RMW_IMPLEMENTATION: rmw_fastrtps_cpp

primitive:
  - name: my_chassis
    path: ./primitives/my_chassis
    config:
      odom_topic: /odom
      command_topic: /cmd_vel

  - name: my_lidar
    path: ./primitives/my_lidar
    config:
      scan_topic: /scan
      cloud_topic: /points
      sentinel_timeout_s: 90
```

如果要使用 Mapping/Nav2：

```yaml
service:
  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    branch: main
    config:
      use_sim_time: true
      params_file: config/rtabmap_params.yaml
      sensor_providers:
        lidar2d: my_lidar
        lidar3d: my_lidar
        rgb: my_front_camera
        depth: my_front_camera
        odom: my_chassis

  - name: nav2
    url: https://github.com/syswonder/service-navigation-rbnx
    branch: main
    config:
      params_file: config/nav2_params.yaml
      use_sim_time: true
      provider_ids:
        map: mapping
        odom: my_chassis
        scan: my_lidar
```

发布版本固定已经测试的 service/skill 版本或 commit，避免远程 `main` 变化后影响
可复现性。

如果两个后端发布相同 ROS 接口，保持一份 `robonix_manifest.yaml` 即可。`SIM_BACKEND`
属于仿真启动器配置，不应通过复制整套 primitive、service 和 skill 清单来实现后端切换。

---

## 配置 `soma.yaml` 和 URDF

### 沿用厂家 URDF

如果厂家已有真实本体 URDF，可复制固定版本，或者在构建时从指定版本获取；项目只增加
仿真特有的 frame。MJCF 与 ROS 命名不同时，在 Bridge 中显式映射，避免改动真实接口
名称。

MJCF 与 URDF 需要对齐以下内容：

- 根 link；
- 关节名称、轴和限位；
- 传感器安装位置；
- 机械臂基座；
- TCP；
- 相机光学 frame；
- 前进方向。

### Soma 描述的重点

`soma.yaml` 描述 Robonix 看到的本体结构、尺寸、footprint、组件和 capability：

```yaml
urdf:
  path: ./urdf/my_robot.urdf
  root_link: base_link
  model_name: my_robot

robot:
  id: my_robot_sim_01
  display_name: "Vendor My Robot MuJoCo"
  family: mobile_robot
  root_part: base
  dimensions: {length_m: 0.70, width_m: 0.50, height_m: 0.80}
  footprint:
    base_frame: base_link
    points: [[0.35, 0.25], [0.35, -0.25], [-0.35, -0.25], [-0.35, 0.25]]
```

后续 components 中的 `provider_id` 与 `robonix_manifest.yaml` 实例名保持一致。

### Footprint 的取值

取值包括：

- 轮子突出部分；
- 固定传感器架；
- 收纳后的机械臂；
- 线缆保护架；
- 真实运行时不可收回的附件。

如果机械臂展开后超出 footprint，可以在导航前自动收纳，或者实现动态 footprint。

---

## 复用 Mapping、Navigation、Scene 和 Explore 的条件

厂家模型加载成功后，还要满足下面的数据条件才能使用导航服务。

### Mapping

Mapping 使用以下数据：

- 正确且连续的 odom；
- 连通的 TF；
- LiDAR 或 RGB-D；
- 统一仿真时间；
- 正确 frame 和单位；
- 环境碰撞可被传感器射线检测。

### Navigation

Navigation 使用以下数据和接口：

- Mapping 或加载好的地图；
- `map -> odom -> base_link`；
- 可执行的速度接口；
- 与厂家底盘匹配的运动学参数；
- 正确 footprint；
- 障碍 scan；
- 急停和速度限制。

全向底盘、Ackermann 和足式机器人需要使用各自的运动学参数。

### Scene

Scene 需要一个明确的主相机和一致的 RGB-D/TF。多个相机同时声明全局 RGB capability
时，Scene 可能选错。腕部相机用于抓取 skill，前部相机作为移动机器人的 Scene 主视觉。

### Explore

Explore 依赖 Mapping 和 Navigation。其异步任务行为与具体机器人无关，实际执行仍由
Nav2 和底盘完成。狭窄环境、大 footprint 或不能原地旋转的底盘需要重新
调整探索和恢复策略。

---

## 厂家机械臂和抓取能力如何接入

### 使用厂家运动规划或控制接口

接口选择顺序：

1. 厂家提供的 MoveIt/轨迹 action；
2. 厂家官方 IK/SDK；
3. 厂家仿真控制器；
4. 经过验证的社区控制器；
5. 前几项均不可用时，再自行实现 IK 和抓取状态机。

厂家接口已经能执行关节轨迹时，所选 MuJoCo runtime 只负责调用和状态转发。

### 抓取技能的适配层

抓取 skill 通常串联以下步骤：

1. 目标解析；
2. 获取目标位姿；
3. 选择抓取姿态；
4. 调用厂家轨迹/IK 接口；
5. 闭合夹爪；
6. 检查接触或夹持状态；
7. 抬升；
8. 持续验证；
9. 放置；
10. 收纳。

当前仓库的 Piper 抓取实现包含该机械臂专用的安装方向、IK 种子、关节范围和夹爪参数，
不能原样复制到其他机械臂。

### 成功判定

厂家控制器返回“轨迹完成”不等于物体抓取成功，还需要检查：

- 指定物体被夹持；
- 物体相对支撑面被抬升；
- 持续数秒没有掉落；
- 相对 TCP 漂移在阈值内；
- 放置后物体稳定；
- 机械臂回到安全姿态。

---

## 环境与机器人的边界

机器人包保存：

- 本体模型；
- 本体控制器或厂家控制后端适配；
- 本体传感器；
- Robonix capability。

环境包保存：

- 房间视觉；
- 房间碰撞；
- 出生点；
- 可交互物体；
- 视觉/碰撞变换；
- 场景许可证。

本体包不应为 `kitchen_my_robot`、`office_my_robot` 分别复制机器人和原语；场景
切换只替换环境资产。

---

## 环境可以从哪里获得

### 现成 MuJoCo 场景

厂家 demo 自带的场景可以用于本体运动验证。作为导航环境使用前，还要检查连续地面、
墙体、传感器可见碰撞和场景尺度。

### SceneSmith 或其他结构化 Mesh 场景

结构化场景通常提供 mesh、纹理、物体层级和房间布局。参考本体包的
`scripts/prepare-scenesmith.py` 展示了预处理方式：

- 保留视觉 mesh；
- 将家具静态化；
- 为主要家具生成简化碰撞；
- 计算机器人安全出生点；
- 将少量指定物体转换为动态任务物体。

机器人尺寸变化后，重新计算出生点和通道可通行性。

### CAD、Blender 和扫描 Mesh

可以作为视觉来源，但需要另外处理：

- 单位；
- 坐标轴；
- 法线和纹理；
- 面数；
- 连续地面；
- 简化碰撞；
- 门洞和通道；
- 许可证。

### Gaussian Splatting PLY/SPZ

在参考本体包的 Web 后端中，SPZ 负责视觉，MuJoCo XML 负责碰撞和传感器射线：

```text
Gaussian PLY -> SPZ -> 视觉
Gaussian PLY/其他几何 -> collision.xml -> 接触、LiDAR、Depth
```

`transform.json` 记录 SPZ 与碰撞模型之间的对齐关系。只有 SPZ 画面而没有碰撞模型时，
机器人不会获得地面接触，LiDAR 和深度相机也没有场景返回。

当前 Native 后端使用 MuJoCo 原生 renderer，不能直接显示 SPZ，因此只接受 Mesh 环境。
不要在 Native 启动失败后退化为“SPZ 不显示但仍加载碰撞”，否则操作者看到的环境与机器人
实际碰撞环境不一致。场景构建阶段应检查 `visualMode` 并明确拒绝不支持的组合。

---

## 环境注册

Mesh 环境示例：

```json
{
  "id": "my_mesh_room",
  "label": "My Mesh Room",
  "visualMode": "mesh",
  "xmlPath": "./assets/environments/my_mesh_room/scene.xml",
  "filesPath": "./assets/environments/my_mesh_room/index.json",
  "spawnPath": "./assets/environments/my_mesh_room/spawn.json"
}
```

SPZ 环境示例：

```json
{
  "id": "my_spz_room",
  "label": "My SPZ Room",
  "visualMode": "spz",
  "xmlPath": "./assets/environments/my_spz_room/collision.xml",
  "spzPath": "./assets/environments/my_spz_room/scene.spz",
  "transformPath": "./assets/environments/my_spz_room/transform.json",
  "spawnPath": "./assets/environments/my_spz_room/spawn.json"
}
```

统一注册在：

```text
assets/environments/manifest.json
```

`SceneManager` 根据 environment 自动选择视觉模式，不需要用户单独选择 `visual_mode`。
在双后端项目中，还要由启动器或 Native scene builder 检查 backend 与视觉模式是否兼容。
参考实现中，Mesh 可用于 Web 和 Native，SPZ 只能用于 Web；`visualMode` 仍由环境决定，
`backend` 决定由哪个 MuJoCo runtime 加载该环境。

---

## 添加可交互物体

### 可交互物体属于环境

物体的位置由房间决定，文件放在：

```text
assets/environments/<environment_id>/objects.xml
```

并在环境 manifest 中设置 `objectsPath`。

Web `SceneManager` 和 Native scene builder 都应读取同一个 `objectsPath`，保证可交互物体
的 ID、世界位姿、质量和碰撞属性不随后端变化。

### 动态物体最低要求

```xml
<mujoco model="room_objects">
  <worldbody>
    <body name="task_cup" pos="1.2 0.5 0.76" quat="1 0 0 0">
      <freejoint name="task_cup_freejoint"/>

      <geom name="task_cup_visual"
            type="mesh" mesh="cup_mesh" material="cup_material"
            contype="0" conaffinity="0" density="0" group="1"/>

      <geom name="task_cup_collision"
            type="cylinder" pos="0 0 0.05" size="0.035 0.05"
            density="350" friction="0.9 0.03 0.003"
            contype="1" conaffinity="1" group="5"/>
    </body>
  </worldbody>
</mujoco>
```

当前抓取链路约定：

- body 以 `task_` 开头；
- freejoint 为 `<body_name>_freejoint`；
- body 名是稳定对象 ID；
- 视觉使用 group 1；
- 动态碰撞推荐 group 5。

### 物体来源

可交互物体可以来自：

- 场景中已有的 SceneSmith 物体；
- MuJoCo Menagerie 或其他允许再分发的模型；
- 厂家夹爪 demo 的测试物；
- 自己制作的低复杂度物体；
- 从场景 mesh 中拆分出的独立物体。

这些物体需要记录来源和许可证。视觉 mesh 可以保留细节，碰撞通常使用 box、cylinder、capsule 或
凸分解。

---

## 启动与依赖隔离

推荐保持仿真和 Robonix 两个独立生命周期：

```text
终端 1：sim/start.sh
  ├── Bridge 容器
  ├── Web 后端：静态服务 + 浏览器 MuJoCo
  ├── Native 后端：Python/C++ MuJoCo + 可选 viewer
  └── 可选厂家 policy server

终端 2：rbnx boot
  ├── primitive
  ├── service
  └── skill
```

厂家控制器如果依赖特殊 Python/CUDA：

- 使用独立 venv 或容器；
- 不修改系统 Python；
- 不污染 Robonix 环境；
- 固定版本和 lockfile；
- 通过明确网络协议与 Bridge 通信；
- 停止脚本同时回收控制进程和 GPU 资源。

双后端项目还应隔离各自的运行依赖：

- Web 后端固定 Node.js lockfile、WASM 包和浏览器版本；
- Native 后端固定 MuJoCo Python/C++ 版本、NumPy、WebSocket 库和系统图形依赖；
- ROS 2 Bridge 使用独立容器或环境，不继承其他终端中的 `RMW_IMPLEMENTATION`；
- Native viewer 与无界面运行使用同一物理和 Bridge 代码，`--headless` 只控制 viewer；
- 如果无界面模式仍发布 RGB，相应 OpenGL/EGL/OSMesa 上下文仍是运行依赖。

构建和启动：

```bash
cp .env.example .env
bash scripts/bootstrap.sh

# 终端 1
bash sim/start.sh --environment scenesmith_house_187

# 终端 2
source scripts/env.sh
rbnx boot

# 终端 3
source scripts/env.sh
rbnx chat
```

上例默认启动 Web 后端。启动 Native viewer：

```bash
bash sim/start.sh \
  --backend native \
  --viewer \
  --environment scenesmith_house_187
```

不显示 viewer、但继续运行物理和传感器：

```bash
bash sim/start.sh \
  --backend native \
  --headless \
  --environment scenesmith_house_187
```

启动器应等到 Bridge 健康接口确认预期 backend、environment 和状态帧均已就绪，再提示
用户执行 `rbnx boot`。后端切换不应改变终端 2 和终端 3 的命令。

验收分为两层：

- 共用验收检查 ROS topic、TF、传感器有效数据、建图、导航、抓取、急停和 reset；
- 运行时专项测试检查 Web 浏览器连接/WASM 资产，以及 Native 模型加载、SPZ 拒绝、
  viewer 交互、硬件 RGB、射线传感器和无界面生命周期。

停止：

```bash
source scripts/env.sh
rbnx shutdown
bash sim/stop.sh
```

---

## 参考本体包中的实现

Ranger/Piper 参考本体包采用“共享本体与 Bridge 协议、分离运行时实现”的结构：Piper
复用了 MuJoCo Menagerie 模型；Web 与 Native 后端加载同一机器人和环境注册信息，分别
实现控制、传感器与渲染，再共用 ROS 2 Bridge、Robonix 原语、服务、技能和验收流程。

| 目标 | 参考文件 |
| --- | --- |
| 机器人注册 | `assets/robots/index.json`、`assets/robots/ranger_mini_v3_piper/robot.json` |
| 组合厂家/上游模型 | `assets/robots/ranger_mini_v3_piper/ranger_mini_v3_piper.xml` |
| Web 控制适配和状态提取 | `assets/robots/ranger_mini_v3_piper/controller.js` |
| 控制器接口 | `src/utils/controllers/BaseController.js` |
| 机器人资产加载 | `src/utils/RobotRegistry.js`、`src/utils/RobotLoader.js` |
| 环境与机器人组合 | `src/utils/SceneManager.js` |
| Web 传感器模拟 | `src/utils/RobotSensorSuite.js` |
| 浏览器 Bridge 客户端 | `src/utils/MujocoBridgeClient.js` |
| Native 场景构建 | `sim/native/scene_builder.py` |
| Native runtime | `sim/native/runtime.py` |
| Native 控制适配 | `sim/native/controller.py` |
| Native 传感器模拟 | `sim/native/sensors.py` |
| 双后端启动与生命周期 | `sim/start.sh`、`sim/compose.native.yaml` |
| ROS 2 Bridge | `sim/bridge/bridge_node.py` |
| Robonix 部署 | `robonix_manifest.yaml` |
| 本体语义结构 | `soma.yaml` |
| ROS 坐标树 | `urdf/ranger_piper.urdf` |
| 通用 primitive | `primitives/common/` |
| 本地 package 示例 | `primitives/*/{package_manifest.yaml,config.spec,CAPABILITY.md}` |
| 抓取 skill | `skills/pick/` |
| 环境注册 | `assets/environments/manifest.json` |
| 环境动态物体 | `assets/environments/scenesmith_house_187/objects.xml` |
| SceneSmith 预处理 | `scripts/prepare-scenesmith.py` |
| 端到端验收 | `scripts/acceptance.sh`、`sim/tests/ros_acceptance.py` |
| Native 快速测试 | `sim/tests/native_smoke.py` |

推荐阅读顺序：

1. 厂家或上游模型的 README 和 demo；
2. 参考本体包的 `robot.json`；
3. wrapper/组合 MJCF；
4. Web `controller.js` 或 Native `controller.py`；
5. 对应 runtime、传感器实现和共用 `bridge_node.py`；
6. 一个 primitive；
7. `robonix_manifest.yaml` 与 `soma.yaml`；
8. 共用 acceptance 与运行时专项测试。

---

## 延伸阅读

- [MuJoCo Modeling](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [MuJoCo XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo Computation](https://mujoco.readthedocs.io/en/stable/computation/index.html)
- [Robonix 文档](https://book.robonix.ai/)

接入完成后，厂家模型和控制接口仍保持可追溯；本体包新增的部分集中在加载、协议桥接、
Robonix 能力约定和测试。验收以传感器、导航和操作任务的实际结果为准。
