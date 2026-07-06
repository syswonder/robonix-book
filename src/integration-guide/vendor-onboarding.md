# 本体接入指南

[toc]

本手册面向机器人本体厂商的研发人员，给出将一台硬件设备接入 Robonix 的完整、可照做的步骤。读者无需预先了解 Robonix 的内部架构。

全文以一台差速底盘为例。数据通路这一步（步骤 5）按厂商 SDK 的形态分情况给出——已自带 ROS 2 驱动、只有 C++ / Python SDK、或专有消息；其余步骤对各种本体通用。

---

## 1. 设计模型（请先阅读）

接入工作只涉及三个概念：

| 概念 | 含义 |
|---|---|
| 能力（capability） | 系统中一个带稳定 ID 的接口，例如 `robonix/primitive/chassis/move`。 |
| 能力约定（contract） | 对一个能力的描述：载荷数据的结构。底盘、相机、激光雷达等的标准能力约定已由 Robonix 定义（见[能力约定参考](../reference/contracts.md)），厂商直接实现，无需自行设计。 |
| 原语（primitive） | 一个直接驱动硬件的进程，向系统声明它实现了哪些能力。一台设备对应一个原语。 |

### 1.1 能力约定：一套与通信方式无关的描述

Robonix 用两份文件描述每个能力的能力约定：

- 一份 TOML：元数据——能力约定 ID、版本、种类（primitive / service / skill）、传输模式等。
- 一份 ROS IDL（`.msg` / `.srv`）：载荷的结构——消息或服务的字段。

"TOML + ROS IDL" 是 Robonix 规定的描述语法。它只刻画"这个能力承载什么形状的数据"，本身不关心用什么通信方式传输。底盘、相机、激光雷达等的标准能力约定都已用这套语法定义好，随主仓库一起分发，位于 Robonix 源码树的 `capabilities/`（TOML）与 `capabilities/lib/`（IDL）下。

### 1.2 codegen：把描述投射到各种通信方式

`robonix-codegen`（经 `rbnx codegen` 调用）读取上述与通信方式无关的描述，为 Robonix 目前支持的每一种通信方式生成实现/消费该能力所需的全部代码：

| 通信方式 | codegen 产物 | 适用接口 |
|---|---|---|
| gRPC | protobuf 定义 + 各语言桩 | RPC 类接口（如 `move`、生命周期 `driver`） |
| MCP | 带类型的 Python dataclass | 暴露给大模型的工具 |
| ROS 2 | 可 `colcon` 编译的 IDL 消息包 | ROS 2 话题 / 服务 |

也就是说：同一份能力约定描述，codegen 按需生成 gRPC、MCP、ROS 2 各自所需的产物。厂商不自行定义任何消息或接口类型，只消费 codegen 为所选通信方式生成的结果。

### 1.3 消息类型一律以 Robonix 的 IDL 为准

所有类型的"形状"都源自同一套 Robonix IDL，因此各通信方式生成的代码彼此一致。这同样适用于 `geometry_msgs/Twist`、`nav_msgs/Odometry` 这类看似"标准"的类型——它们也在 Robonix 的 IDL 集合内，由 codegen 统一生成。厂商应使用这套生成结果，而不是某个 ROS 2 发行版自带的同名定义。具体每条消息 / 服务的字段定义见 [ROS IDL 参考](../reference/idl.md)。

---

## 2. 流程概览

把一个本体接入 Robonix，依次完成以下步骤（下面以底盘为例，其它本体同理）：

1. 安装工具链
2. 创建部署项目与原语包骨架
3. 声明本体要实现的能力约定
4. 生成 Robonix 标准 ROS 2 消息包
5. 用 C++ SDK 实现数据通路（一个 ROS 2 节点）
6. 实现 Python 原语（声明能力 + 生命周期）
7. 编写启动脚本
8. 登记到部署清单并启动验证

下面逐步展开。

---

## 步骤 1：准备环境

接入一台底盘需要两套工具链：Robonix 自己的（`rbnx` + `robonix-api`），以及 ROS 2（底盘的 `twist_in` / `odom` 走 ROS 2 话题，所以本体侧需要 ROS 2 环境）。下面分别配置。

### Robonix 工具链

当前版本中，构建工具 `rbnx`、能力约定/IDL 定义、以及 Python 客户端库 `robonix-api` 都随 Robonix 主仓库分发。克隆并安装：

```bash
git clone --recursive https://github.com/syswonder/robonix.git   # --recursive 必须：能力约定 IDL 的上游 ROS 消息（common_interfaces / rcl_interfaces）是子模块
cd robonix
make install        # 编译并安装 rbnx 等二进制到 ~/.cargo/bin，
                    # 同时把本仓库登记为能力约定 / IDL / robonix-api 的来源
```

`make install` 需要 Rust 工具链（见 [rustup.rs](https://rustup.rs)）与 Python ≥ 3.10。装完确认 `rbnx` 可用、且源码树已登记：

```bash
rbnx --help
rbnx path robonix-api    # 应打印 <你克隆的 robonix>/pylib/robonix-api
```

`robonix-api`（原语基类、atlas 客户端、生命周期）随源码树分发，无需单独 `pip install`：步骤 7 的 `start.sh` 通过 `rbnx path robonix-api` 将其加入 `PYTHONPATH`。本手册假定目标机器上已部署 Robonix 源码树；若目标机器不部署源码树，可改用 PyPI 版（`pip install robonix-api`），并相应调整 `start.sh` 中的 `PYTHONPATH` 设置。

### ROS 2 环境

Robonix 框架本身不硬绑定某个 ROS 2 发行版，但以 **Humble（Ubuntu 22.04）为参考发行版**（其它发行版尽力兼容），本手册即以 Humble 为例。按官方文档装好二进制发行版后，本体侧只需三样东西：

```bash
# 1) ROS 2 发行版本体（按官方文档安装，下面是 Humble/Ubuntu 22.04 的包名）
sudo apt install ros-humble-ros-base

# 2) colcon —— 编译 Robonix 生成的 ROS 2 消息 overlay 用（步骤 4 / 7）
sudo apt install python3-colcon-common-extensions

# 3) 每个要用 ROS 2 的 shell 都要先 source 发行版环境；可写进 ~/.bashrc
source /opt/ros/humble/setup.bash
```

> **发行版一致性（重要）**：Robonix 不绑定某个 ROS 2 发行版，但这只是说**框架本身**不挑发行版——Robonix 的 ROS IDL 只规定能力载荷的**字段与数据结构**（消息长什么样），它统一不了 ROS 2 发行版之间的通信层差异。真正的约束在通信层：不同发行版的 **DDS 实现 / 通信库 ABI 版本**互不兼容，话题在线上是按各自的通信库序列化与收发的。所以——
>
> **同一次 Robonix 部署内，所有用到 ROS 2 的系统服务、原语、技能，其 ROS 2 通信库版本必须一致**（要么都 Humble、要么都 Foxy、要么都 Jazzy）。推荐的统一方式是整机选定一个目标发行版，所有 ROS 2 组件都用它。
>
> 若实在无法统一（例如某个本体的厂商驱动只支持另一个发行版），**不要直接混跑**——用 [zenoh bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) 在两个发行版 / DDS 域之间手动转发需要互通的话题，把数据桥接过去。
>
> 本手册的 `build.sh` / `start.sh` 里都写 `source /opt/ros/humble/setup.bash`，请按你机器上的实际发行版路径替换。
>
> **host 装不了 ROS 2？** 若本体主机系统无法原生安装目标发行版（不是 Ubuntu 22.04、架构不符、或需要隔离环境），可以把需要 ROS 2 的部分放进 Docker 容器运行，`rbnx` 与 atlas 仍在 host——完整做法见 [7.4 Docker 变体](#74-docker-变体host-没有-ros-2-时)。

### Robonix 复用了哪些 ROS 2 工具

Robonix 把 ROS 2 当作**三种数据传输方式之一**（另两种是 gRPC、MCP），只复用它的消息类型系统与话题传输，**不**使用 ROS 2 的服务发现 / 生命周期 / 参数等机制——那些由 Robonix 自己的 atlas（发现）和 `driver` 能力约定（生命周期）负责。具体复用的部分：

| ROS 2 组件 | Robonix 怎么用 | 出现在 |
|---|---|---|
| 消息类型 (`.msg`/`.srv`) + `rosidl` | 能力约定的载荷 IDL 就是 ROS IDL；codegen 生成可编译的消息包 | 步骤 4 |
| `colcon` | 编译上面生成的消息包，得到 overlay | 步骤 4 / `build.sh` |
| `rclpy` | Python 原语的话题收发（`create_publisher` / `create_subscription` / `emit` 内部就是 rclpy） | 步骤 6 |
| `rclcpp` | 自写 C++ 数据节点时链接它（情形 B-1 / C） | 步骤 5 |
| DDS（话题传输层） | 话题数据实际在 DDS 上跑；发行版一致性要求即来源于此 | 运行期 |
| `ros2` CLI（`ros2 topic` / `ros2 run`） | 调试：核对话题名、类型、收发是否正常 | 排错 |

也就是说：你在本体侧装好 ROS 2 + colcon 即可，话题怎么被发现、谁来连，交给 Robonix。

---

## 步骤 2：创建部署项目与原语包骨架

Robonix 部署以"项目"为单位组织。创建一个项目，再在其中创建底盘原语包：

```bash
rbnx init my_robot                          # 生成 robonix_manifest.yaml 与 primitives/ services/ skills/
cd my_robot
rbnx package-new my_chassis --type primitive   # 注意：--type 默认为 service，须显式指定 primitive
```

第二条命令在 `primitives/my_chassis/` 下生成一个可用的骨架：

```
primitives/my_chassis/
├── package_manifest.yaml      # 包元数据 + 能力约定清单
├── scripts/
│   ├── build.sh               # 已填好：调用 rbnx codegen
│   └── start.sh               # 已填好：设置 PYTHONPATH 并启动 python3 -m my_chassis.main
├── my_chassis/
│   ├── __init__.py
│   └── main.py                # Python 原语骨架（含 on_init 与 run）
└── capabilities/              # 仅当本包要自带能力约定时使用；本例留空
```

后续步骤在此骨架上填写。

---

## 步骤 3：声明底盘要实现的能力约定

底盘的标准能力约定如下（定义在步骤 1 克隆的 Robonix 源码树里：能力约定 TOML 在 `<robonix>/capabilities/primitive/chassis/`，IDL 在 `<robonix>/capabilities/lib/chassis/`；`<robonix>` 的绝对路径可用 `rbnx path capabilities` 查看。这些是 Robonix 预置的，你只实现、不修改）。"传输方式"一列是参考实现采用的方式；除 `driver` 固定为 gRPC 外，其余均为推荐，厂商可自行选择：

| 能力约定 ID | 传输方式 | 载荷 | 消费方 |
|---|---|---|---|
| `robonix/primitive/chassis/driver` | gRPC（固定） | 生命周期（框架内置） | `rbnx boot` 启动握手 |
| `robonix/primitive/chassis/move` | gRPC（推荐） | `chassis/MoveCommand` → `std_msgs/String` | 导航服务、遥操作 |
| `robonix/primitive/chassis/twist_in` | ROS 2 话题（推荐） | `geometry_msgs/Twist` | 导航控制器（下发速度） |
| `robonix/primitive/chassis/odom` | ROS 2 话题（推荐） | `nav_msgs/Odometry` | 建图、定位 |

将这四条能力约定填入 `primitives/my_chassis/package_manifest.yaml` 的 `capabilities` 段：

```yaml
capabilities:
  - name: robonix/primitive/chassis/driver
  - name: robonix/primitive/chassis/move
  - name: robonix/primitive/chassis/twist_in
  - name: robonix/primitive/chassis/odom
```

`package.name`（如 `robonix.primitive.acme.ranger_mini.chassis`）是包的发布名，与运行时的原语 ID 无关。准备提交到社区 catalog 的包还需要在 `package` 段填写 `version`、`description`、`tags`、`maintainers`；`maintainers` 每项使用 `Name <email@domain>` 格式。

---

## 步骤 4：理解 Robonix 标准 ROS 2 消息包

底盘的 `twist_in`、`odom` 走 ROS 2 话题，其载荷类型（`geometry_msgs/Twist`、`nav_msgs/Odometry`）必须采用 Robonix 的标准定义（[见 1.3](#13-消息类型一律以-robonix-的-idl-为准)），而不是 ROS 2 发行版自带的同名类型。

这套消息以**源码**形式由 `rbnx codegen --ros2` 生成，和 gRPC/MCP 桩一样落在包的 `rbnx-build/codegen/ros2_idl/` 下。源码要用 `colcon` 编译一次，得到一个 ROS 2 overlay（`ros2_idl/install/`）；之后任何 `source` 了它的进程，拿到的 `geometry_msgs` / `nav_msgs` 就都是 Robonix 的定义。

本步不需要你单独敲命令——生成与编译都写进步骤 7 的 `build.sh`，由 `rbnx build`（步骤 8）一次执行。这里只需记住产物路径：

```
primitives/my_chassis/rbnx-build/codegen/ros2_idl/install/setup.bash
```

步骤 5 的 C++ 节点（编译期）与步骤 7 的 `start.sh`（运行期）都会 `source` 它。

---

## 步骤 5：让底盘数据出现在 ROS 2 话题上

底盘的数据通路——接收速度命令、反馈里程计——走两条 ROS 2 话题：一条速度话题（`geometry_msgs/Twist`，下发）、一条里程计话题（`nav_msgs/Odometry`，反馈）。本步只有一个目标：**让这两条话题在 ROS 2 上真实存在并跑起来**。把它们声明给 atlas、实现生命周期与 `move`，都是步骤 6 的事，本步不碰。

怎么做取决于你的厂商 SDK 已经做到哪一步，下面四种情形选其一。

> 提醒：本步起所有 ROS 2 节点的通信库版本必须与整机其余 ROS 2 组件一致（发行版一致性是硬约束，详见 [步骤 1 · ROS 2 环境](#ros-2-环境)）。

### 情形 A（最常见）：厂商驱动已发布标准 /cmd_vel + /odom

很多本体开箱即在 ROS 2 上跑：松灵 Scout / Ranger 的 `*_ros2` 驱动订阅 `/cmd_vel`（`Twist`）、发布 `/odom`（`Odometry`）；iRobot Create 3 固件原生跑 ROS 2；相机 / 雷达（RealSense / Orbbec / Livox）都有官方 ROS 2 wrapper。

这种情形本步无需写任何代码——把厂商驱动正常跑起来即可。你要做的是**记录两件事**，留给步骤 6 用：

1. 速度话题、里程计话题的**实际话题名**。注意常带命名空间：例如 Clearpath 是 `/<robot_ns>/cmd_vel`，里程计叫 `odometry/filtered`（EKF 融合后的，不叫 `/odom`）。用 `ros2 topic list` 核对。
2. 这两条话题的**消息类型**是否与 Robonix 能力约定一致。标准类型（`Twist` / `Odometry`）天然一致；若厂商用了变体（如 Clearpath 新平台的 `geometry_msgs/TwistStamped`），先写一个小 relay 节点转换成标准类型，把 relay 的输出话题作为上面记录的话题名。

### 情形 B：厂商只有核心 SDK（C++ 或 Python），未接入 ROS 2

有些本体只给一个底层 SDK：松灵 `ugv_sdk` 是 ROS-independent 的 C++ 库（走 CAN）、云深处 Lite3 是 C++ over UDP、UR 的 `ur_rtde` 是 `pip` 装的 Python 库。这时由你写一个薄适配：订阅 `/cmd_vel` 调 SDK、读 SDK 状态发 `/odom`。按 SDK 语言二选一。

#### B-1　C++ SDK → 写一个 C++ ROS 2 节点

写一个 `ament_cmake` 工程：链接你的 SDK，并依赖步骤 4 的 Robonix 消息包。在**包内**新建 `primitives/my_chassis/ros2_nodes/my_chassis_node/`（含 `package.xml` / `CMakeLists.txt` / `src/chassis_node.cpp`）——放在包内，后续的 `build.sh` / `start.sh` 才能用包相对路径找到它。

`CMakeLists.txt`——`find_package` 同时引入 Robonix 消息包与你的 SDK：

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_chassis_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)   # 来自 Robonix IDL 消息包
find_package(nav_msgs REQUIRED)        # 来自 Robonix IDL 消息包

add_executable(my_chassis_node src/chassis_node.cpp)

# 厂商 C++ SDK：头文件目录 + 动态库（按你的 SDK 实际路径调整）
target_include_directories(my_chassis_node PRIVATE /opt/vendor_sdk/include)
target_link_libraries(my_chassis_node /opt/vendor_sdk/lib/libvendor.so)

ament_target_dependencies(my_chassis_node rclcpp geometry_msgs nav_msgs)

install(TARGETS my_chassis_node DESTINATION lib/${PROJECT_NAME})
ament_package()
```

`src/chassis_node.cpp`——订阅速度 → 调 SDK；读 SDK 位姿 → 发布里程计：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vendor_sdk.h>   // 厂商 SDK 头文件

class ChassisNode : public rclcpp::Node {
public:
  ChassisNode() : Node("my_chassis_node") {
    sdk_ = vendor_connect("/dev/ttyUSB0");   // ← 替换为你的 SDK 初始化

    // 订阅速度命令，转交 SDK 控制底盘。
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10),
        [this](geometry_msgs::msg::Twist::SharedPtr m) {
          vendor_set_velocity(sdk_, m->linear.x, m->angular.z);
        });

    // 周期读取 SDK 位姿，发布为里程计。
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
      nav_msgs::msg::Odometry odom;
      VendorPose p = vendor_read_pose(sdk_);
      odom.pose.pose.position.x = p.x;
      odom.pose.pose.position.y = p.y;
      // ... 按需填充其余字段 ...
      odom_pub_->publish(odom);
    });
  }
private:
  VendorHandle* sdk_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisNode>());
  rclcpp::shutdown();
}
```

节点源码写好后先放着，本步不用手动编译——它的编译会在步骤 7 的 `build.sh` 里统一处理。节点跑起来后订阅 `/cmd_vel`、发布 `/odom`，消息类型为 Robonix 的定义。记下这两个话题名（`/cmd_vel`、`/odom`），留给步骤 6 用。

#### B-2　Python SDK → 数据收发并入 Python 原语

厂商是 Python SDK 时，不必单独起一个数据节点：速度订阅与里程计发布直接用 `robonix-api` 封装的 rclpy 在 Python 原语里完成。所以本步对 B-2 没有独立产物——这部分代码和原语写在一起，见步骤 6 的情形 B-2 写法。本步你只需确认 Python SDK 能正常连上硬件。

### 情形 C：厂商有 ROS 2，但用专有消息（非 `cmd_vel` / `odom`）

部分本体（多为腿足，如 Unitree 的 `unitree_ros2`）虽在 ROS 2 上，但走的是专有消息（如 `/lowcmd`、`/sportmodestate`），并不提供标准的 `/cmd_vel` + `/odom`。这时写一个适配节点（C++ 或 Python 皆可，做法同 B-1 / B-2）在两者之间互转：订阅 `/cmd_vel` → 转成厂商的速度请求；订阅厂商状态 → 填成 `Odometry` 发 `/odom`。转出标准的 `/cmd_vel` + `/odom` 后，本步即完成；记下这两个话题名，留给步骤 6。

前提：转出的话题消息类型须与 Robonix 的定义一致——标准消息（`Twist` / `Odometry`）天然一致；若用到 Robonix 自定义消息类型，适配节点同样要编译在步骤 4 的 IDL overlay 上、使用它生成的类型。

---

## 步骤 6：实现 Python 原语

Python 原语负责把数据通路的能力声明给 atlas（使其可被导航、建图发现），并实现 `driver` 生命周期与 `move` 命令。下面这份是薄层写法（情形 A / B-1 / C）：数据由厂商驱动或 C++ 节点经 ROS 2 承载，原语不直接接触 SDK，只做声明。若是情形 B-2（Python SDK），则把步骤 5 的 `create_subscription` / `create_publisher` 也放进同一个 `on_init`，由原语兼任数据收发。

编辑骨架生成的 `primitives/my_chassis/my_chassis/main.py`：

```python
#!/usr/bin/env python3
from robonix_api import Primitive, Ok
from geometry_msgs.msg import Twist   # 来自 Robonix IDL 消息包

# id 须与部署清单中本条目的 name 一致（见步骤 8）。
chassis = Primitive(id="my_chassis", namespace="robonix/primitive/chassis")

cmd_vel_pub = None


@chassis.on_init
def init(cfg: dict):
    global cmd_vel_pub
    # 把两条 ROS 2 话题能力声明给 atlas：能力约定 ID → C++ 节点所用的话题名。
    # 消费方据能力约定 ID 向 atlas 查询，即可发现并连接这些话题。
    chassis.declare_ros2_topic("robonix/primitive/chassis/twist_in", "/cmd_vel", qos="reliable")
    chassis.declare_ros2_topic("robonix/primitive/chassis/odom", "/odom", qos="reliable")

    # move 命令通过向 /cmd_vel 发布 Twist 实现——与 C++ 节点订阅的是同一话题。
    cmd_vel_pub = chassis.create_publisher(
        "robonix/primitive/chassis/twist_in", topic="/cmd_vel",
        msg_type=Twist, qos="reliable", declare=False,
    )
    return Ok()


@chassis.grpc("robonix/primitive/chassis/move")
def move(req):
    """突发运动命令：翻译为一段定时的 Twist，发到 /cmd_vel。"""
    import json, time
    import std_msgs_pb2, chassis_pb2   # rbnx codegen 生成

    c = req.command
    tw = Twist()
    tw.linear.x = float(c.linear_x)
    tw.angular.z = float(c.angular_z)
    dur = float(c.duration_sec) or 1.0
    for _ in range(max(1, int(dur / 0.1))):
        cmd_vel_pub.publish(tw)
        time.sleep(0.1)
    cmd_vel_pub.publish(Twist())   # 收尾停车
    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(data=json.dumps({"status": "done"})))


if __name__ == "__main__":
    chassis.run()
```

关键 API：

- `Primitive(id, namespace)` 构造原语；`@chassis.on_init` 注册初始化（返回 `Ok()` / `Err("原因")` / `Deferred("原因")`）。
- `chassis.declare_ros2_topic(contract_id, topic, qos=...)` 把"能力约定 ID → 话题名"声明给 atlas。
- `@chassis.grpc(contract_id)` 将函数注册为该能力约定的 gRPC 处理函数；其请求/返回为 `rbnx codegen` 生成的 protobuf 消息。
- `chassis.run()` 阻塞运行，内部处理 gRPC、ROS 2、心跳与信号。

### 生命周期与资源释放

`driver` 能力约定（gRPC）由 `Primitive` 基类自动提供并声明，厂商不实现协议，只按需编写四个回调，对应状态机：

| 回调 | 触发 | 状态迁移 | 说明 |
|---|---|---|---|
| `on_init(cfg)` | `rbnx boot` 的 `CMD_INIT` | REGISTERED → INACTIVE | 必填。连接硬件、建立发布/订阅、声明能力。返回 `Ok()` / `Err("原因")` / `Deferred("原因")`。 |
| `on_activate()` | `CMD_ACTIVATE` | INACTIVE → ACTIVE | 可选。获取运行时资源（线程、模型句柄等）。primitive 省略时框架自动晋级。 |
| `on_deactivate()` | `CMD_DEACTIVATE` | ACTIVE → INACTIVE | 可选。释放上面这些运行时资源。 |
| `on_shutdown()` | SIGTERM / 进程退出 | 任意 → TERMINATED | 可选，但持有硬件资源的驱动建议实现。 |

退出时框架自动关闭：你打开的 channel、`spawn` 起的子进程、`driver` 的 gRPC server。它不会替你停 rclpy 节点或断开你的 SDK——进程退出时 rclpy 节点由系统回收，但显式停掉后台线程、断开硬件是好习惯，放在 `on_shutdown` 里。上面的薄层写法（情形 A / B-1 / C）原语不持有硬件资源，无需 `on_shutdown`；情形 B-2（Python 原语兼任数据收发）则需要：

```python
import threading
_stop = threading.Event()

@chassis.on_init
def init(cfg: dict):
    global sdk
    sdk = my_vendor_sdk.connect(cfg.get("device", "/dev/ttyUSB0"))
    chassis.create_subscription(
        "robonix/primitive/chassis/twist_in", topic="/cmd_vel", msg_type=Twist,
        callback=lambda m: sdk.set_velocity(m.linear.x, m.angular.z), qos="reliable")
    chassis.create_publisher(
        "robonix/primitive/chassis/odom", topic="/odom", msg_type=Odometry, qos="reliable")
    threading.Thread(target=_odom_loop, daemon=True).start()
    return Ok()

@chassis.on_shutdown
def shutdown():
    _stop.set()                      # 停后台发布线程
    if sdk is not None:
        sdk.disconnect()             # 断开硬件

def _odom_loop():
    import time
    while not _stop.is_set():
        chassis.emit("robonix/primitive/chassis/odom", _sdk_pose_to_odom(sdk.read_pose()))
        time.sleep(0.02)
```

---

## 步骤 7：编写 `build.sh` 与 `start.sh`

包里有两个脚本，分别对应两个生命周期阶段，是接入工作的核心，必须按本包实际情况写对：

- `scripts/build.sh` —— `rbnx build`（步骤 8）调用，负责**离线准备**：代码生成 + 编译。一次执行，产物落在 `rbnx-build/` 下。
- `scripts/start.sh` —— `rbnx boot` / `rbnx start`（步骤 8）调用，负责**运行时拉起进程**：每次启动都跑。

骨架（步骤 2）生成的是最小可用版本——`build.sh` 只跑 `rbnx codegen -p "$PKG"`（仅出 gRPC/Python 桩），`start.sh` 只跑 Python 原语。底盘要走 ROS 2 话题、还要编译 C++ 节点，所以两个脚本都需要在骨架基础上补全。下面给出完整内容并逐段解释。

### 7.1 `build.sh`

编辑 `primitives/my_chassis/scripts/build.sh`：

```bash
#!/usr/bin/env bash
# 由 rbnx build 调用。职责：把本包能力约定所需的代码生成出来，并编译 ROS 2 产物。
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# 1) 代码生成。默认产出 gRPC proto + Python 桩（move 命令与 driver 生命周期用）；
#    --ros2 额外产出 ROS 2 消息 overlay 的源码（twist_in / odom 的话题类型用）。
#    全部落在 $PKG/rbnx-build/codegen/{proto_gen,ros2_idl}/。
rbnx codegen -p "$PKG" --ros2

# 2) 编译 ROS 2 消息 overlay（需要一个已 source 的 ROS 2 环境，发行版按你自己的填）。
#    编译后 ros2_idl/install/ 即 Robonix 标准类型的 overlay。
source /opt/ros/humble/setup.bash
( cd "$PKG/rbnx-build/codegen/ros2_idl" && colcon build )

# 3) 仅情形 B-1 / C（自己写 C++ 节点）需要这一段：编译你的 C++ 数据节点。
#    先 source 上一步的 overlay，让 find_package(geometry_msgs) 等解析到 Robonix 的类型，
#    而不是发行版自带的同名类型。
source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"
( cd "$PKG/ros2_nodes" && colcon build )

echo "[my_chassis] build done"
```

逐段说明：

- **第 1 段**（代码生成）是相对骨架唯一必须加的东西：骨架默认的 `rbnx codegen -p "$PKG"` 不带 `--ros2`，只出 gRPC/Python 桩；底盘有 ROS 2 话题，必须加 `--ros2` 才会生成 `ros2_idl/`。
- **第 2 段**（编译消息）对所有走 ROS 2 的本体都需要（含情形 A / B-2）——因为即便不写 C++ 节点，Python 原语里 `from geometry_msgs.msg import Twist` 取的也是这个 overlay 里的类型。`source /opt/ros/humble/setup.bash` 换成你机器上实际的发行版路径。
- **第 3 段**（编译 C++ 节点）只有情形 B-1 / C（自己写 C++ 适配节点）才需要，节点源码放在包内 `ros2_nodes/`（见步骤 5 B-1）。情形 A（直接用厂商驱动）、情形 B-2（Python SDK 在原语里收发）删掉这一段。

### 7.2 `start.sh`

编辑 `primitives/my_chassis/scripts/start.sh`：

```bash
#!/usr/bin/env bash
# 由 rbnx boot / rbnx start 调用，拉起本包的运行进程。
set -eo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

# 1) ROS 2 基础环境 + Robonix 标准消息 overlay（运行期的话题类型来源）。
source /opt/ros/humble/setup.bash
source "$PKG_ROOT/rbnx-build/codegen/ros2_idl/install/setup.bash"

# 2) 仅情形 B-1 / C：source 你在 build.sh 第 3 段编出来的 C++ 节点 overlay。
source "$PKG_ROOT/ros2_nodes/install/setup.bash"

# 3) 让 `from robonix_api import ...` 可解析；rbnx-build/codegen/ 下的
#    proto_gen / robonix_mcp_types 由 robonix-api 自动发现，无需手动加进 PYTHONPATH。
export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"

# 4) 后台拉起 C++ 数据节点（情形 B-1 / C），前台运行 Python 原语。
#    前台进程是本包的主进程：它退出，rbnx 即认为本包停止。
ros2 run my_chassis_node my_chassis_node &
exec python3 -m my_chassis.main
```

逐段说明：

- **第 1 段（source overlay）**：`source setup.bash` 只做一件事——设置 / prepend 环境变量（`PATH`、`PYTHONPATH`、`LD_LIBRARY_PATH`、`AMENT_PREFIX_PATH` 等），它本身不加载任何库；真正的加载发生在之后进程启动时（Python `import`、动态链接器 dlopen `.so`）。先 source 发行版、再 source overlay，overlay 的路径排在前面，于是进程启动解析消息包 / 库时优先命中 overlay——也就是 **Robonix IDL** 生成的那套定义，而不是发行版自带的同名包。Robonix IDL 是 Robonix 规定并**冻结**的一套规范消息库，**不随发行版升级变动**，全系统统一以它为准；发行版只提供 rclpy / rclcpp / DDS 运行时与 `ros2` 工具。以 **Humble 为参考发行版**（在 Humble 上与发行版自带消息一致），其它发行版尽力兼容。（overlay 必须用部署所选的发行版编译——生成的 typesupport `.so` 链接的是该发行版的 rosidl 运行时。）
- **第 2 段**只有情形 B-1 / C 需要；情形 A / B-2 删掉。
- **第 3 段（`PYTHONPATH`）**：`rbnx path robonix-api` 打印 robonix-api 源码目录（步骤 1 用 `make install` 登记了 Robonix 源码树，故此命令可用）。codegen 出的 `proto_gen` / `robonix_mcp_types` 不必手动加——`robonix-api` 会按包根下的 `rbnx-build/codegen/` 自动发现。
- **第 4 段（拉起进程）**：`ros2 run ... &` 后台跑 C++ 数据节点，`exec python3 -m my_chassis.main` 前台跑 Python 原语；用 `exec` 让原语接管本进程，`rbnx boot` 的信号能直接送达。情形 A：把 `ros2 run` 那行换成拉起厂商自己的 ROS 2 驱动（或让集成方单独拉起，本脚本只跑原语）。情形 B-2：删掉 `ros2 run` 那行，数据收发在 Python 原语内部完成。

> `rbnx boot` 启动本包时会注入 `ROBONIX_ATLAS`（atlas 地址）、`RBNX_PACKAGE_ROOT` 等环境变量，脚本里无需自行设置。

### 7.3 各情形下两个脚本的差异速查

| | `build.sh` 第 3 段（编 C++ 节点） | `start.sh` 第 2 段（source C++ overlay） | `start.sh` 第 4 段（拉起进程） |
|---|---|---|---|
| A（厂商自带 ROS 2 驱动） | 删 | 删 | 拉起厂商驱动（或交集成方），前台跑原语 |
| B-1（C++ SDK，自写 C++ 节点） | 保留 | 保留 | 后台 C++ 节点 + 前台原语 |
| B-2（Python SDK，原语内收发） | 删 | 删 | 仅前台原语 |
| C（专有消息，自写适配节点） | 保留（若用 C++） | 保留（若用 C++） | 后台适配节点 + 前台原语 |

### 7.4 Docker 变体（host 没有 ROS 2 时）

当本体主机装不了目标 ROS 2 发行版时，把需要 ROS 2 的部分（colcon 编译、原语进程）放进一个带 ROS 2 的容器里运行。分工不变：`rbnx`、atlas、`rbnx boot` 仍在 **host**，只有 colcon 与原语进程进**容器**。三个要点：

- **`--network host`**：容器内的原语用 `127.0.0.1:50051` 就能连到 host 上的 atlas，并和其他 ROS 2 进程处在同一 DDS 域（话题互通）。
- **`--ipc host`**：DDS 默认走共享内存，需要和 host 共享 IPC 命名空间。
- **路径一致**：bind-mount 时让容器内路径 == host 路径（`-v "$PKG":"$PKG"`），这样 host 上 codegen 出的 `install/` 路径在容器里照样有效，两边不用各算一套。

先在 host 上装好 Docker（只装一次）：

```bash
# Ubuntu/Debian：装 Docker Engine（其它发行版见 https://docs.docker.com/engine/install/）
curl -fsSL https://get.docker.com | sh
# 让当前用户免 sudo 跑 docker（重新登录后生效）
sudo usermod -aG docker "$USER"
```

> 硬件访问：本体驱动通常要读串口 / CAN / USB。给容器加设备直通——例如 `--device /dev/ttyUSB0`（串口）、`--device /dev/bus/usb`（USB），或使用 `--privileged`（权限更大，按需取舍）。这些参数加在下面 `start.sh` 的 `docker run` 上。

再备一个镜像（官方 ros 镜像 + colcon + 原语运行所需的 Python 依赖）：

```dockerfile
# Dockerfile.ros2（放在项目根目录）
FROM ros:humble-ros-base
RUN apt-get update \
 && apt-get install -y python3-colcon-common-extensions python3-pip \
 && pip install --no-cache-dir grpcio protobuf pyyaml \
 && rm -rf /var/lib/apt/lists/*
# 你的原语 import 的其它 Python 包（如硬件 SDK）也在这里装
```

```bash
docker build -t my-robot-ros2 -f Dockerfile.ros2 .
```

**`build.sh`（Docker 变体）**——codegen 在 host（`rbnx` 在 host），colcon 进容器：

```bash
#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# 1) 代码生成在 host 上跑（rbnx 在 host）。
rbnx codegen -p "$PKG" --ros2

# 2) colcon 进容器编译（容器里才有 ROS 2）。挂载到同一路径，产物 host 可见。
docker run --rm -v "$PKG":"$PKG" my-robot-ros2 bash -lc "
  source /opt/ros/humble/setup.bash
  cd '$PKG/rbnx-build/codegen/ros2_idl' && colcon build"

# 3) 仅情形 B-1 / C：再编译 C++ 节点（先 source 上一步的 overlay）。
docker run --rm -v "$PKG":"$PKG" my-robot-ros2 bash -lc "
  source /opt/ros/humble/setup.bash
  source '$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash'
  cd '$PKG/ros2_nodes' && colcon build"

echo "[my_chassis] build done"
```

**`start.sh`（Docker 变体）**——原语进程进容器跑，atlas 用 host 网络。Robonix 源码树（提供 `robonix-api`）和包目录都挂进去，路径保持一致：

```bash
#!/usr/bin/env bash
set -eo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROBONIX="$(rbnx path root)"              # host 上的 Robonix 源码树根，挂进容器供 robonix-api 用

exec docker run --rm --network host --ipc host \
  -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
  -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-}" \
  -v "$PKG_ROOT":"$PKG_ROOT" -v "$ROBONIX":"$ROBONIX" \
  my-robot-ros2 bash -lc "
    source /opt/ros/humble/setup.bash
    source '$PKG_ROOT/rbnx-build/codegen/ros2_idl/install/setup.bash'
    [ -d '$PKG_ROOT/ros2_nodes/install' ] && source '$PKG_ROOT/ros2_nodes/install/setup.bash'
    export PYTHONPATH='$ROBONIX/pylib/robonix-api:$PKG_ROOT:'\${PYTHONPATH:-}
    ros2 run my_chassis_node my_chassis_node &
    exec python3 -m my_chassis.main"
```

说明：

- 容器里取 `robonix-api` 用挂载进来的源码树（`$ROBONIX/pylib/robonix-api`），不必在容器里再 `make install`——`rbnx` 只有 host 用得到（codegen、boot）。
- `ROBONIX_ATLAS` 等由 host 上的 `rbnx boot` 注入到 `start.sh`，再用 `-e` 透传进容器。
- 情形 A（厂商驱动自带 ROS 2）：通常厂商驱动也用 Docker 发布，按上面同样的 `--network host --ipc host` 跑起来即可；原语容器只做声明。情形 B-2 / 无 C++ 节点：删掉 `ros2 run ... &` 那行。
- 容器镜像里的 ROS 2 发行版，要和整机其余 ROS 2 组件（其它原语 / 服务，无论原生还是容器）选的同一个发行版一致——发行版一致性约束对容器同样成立（见 [步骤 1 · ROS 2 环境](#ros-2-环境)）。

---

## 步骤 8：登记到部署清单并启动

到这里你交付的是一个原语包。把它跑起来还需要一份部署清单 `robonix_manifest.yaml`——它列出"这台机器上要启动哪些系统组件、哪些设备、哪些服务"，由 `rbnx init`（步骤 2）在项目根目录生成。

> 谁来写这份清单？ 通常是集成方/部署方（把你的包和其它组件组装成一台完整机器人的人），而不是只交付一个设备驱动的硬件厂商。作为厂商你只需提供原语包 + 告诉集成方"在 `primitive:` 下加一条指向我的包"。本步是为了让你能在本地自测，也让你看清自己的包在部署里长什么样。

部署清单里，`primitive:` / `service:` 下的每一条目就是一个硬件/能力实例（device instance）。在 `primitive:` 下为你的底盘加一条：

```yaml
primitive:
  - name: my_chassis              # 实例名，必须 == main.py 里 Primitive(id=...)
    path: ./primitives/my_chassis # 指向你的包目录
    config:                       # 这一台设备实例的参数 → 经 Driver(CMD_INIT) 进 on_init 的 cfg
      device: /dev/ttyUSB0
```

- 一条 = 一台设备。 同一个包可以被列多条来描述多台同型号设备——各用不同的 `name` 和 `config`（例如两台底盘 `chassis_left` / `chassis_right`，串口不同）。这就是"硬件实例"在清单里的表达方式。
- `config` 是这台实例的私有参数（串口、设备号、速度上限等），启动时序列化成 JSON、经 `Driver(CMD_INIT, config_json)` 注入到你 `on_init(cfg)` 的字典。放什么由你的包决定。
- 唯一的硬性一致要求：条目的 `name` 必须与 `main.py` 里 `Primitive(id=...)` 完全相同——`rbnx boot` 据此确认进程注册成功，不一致会启动失败。`package_manifest.yaml` 的 `package.name` 与此无关。

构建、启动、验证：

```bash
rbnx build -p ./primitives/my_chassis   # 跑本包 build.sh：codegen(--ros2) + colcon 编译 overlay/C++ 节点
rbnx boot                                # 启动 atlas、系统服务及清单中各包
rbnx caps                                # 应看到 my_chassis 的四条 chassis/* 能力为 ACTIVE
```

`rbnx boot` 会启动 atlas 与系统服务，运行本包的 `start.sh`，待进程注册后调用 `Driver(CMD_INIT)` 触发 `on_init`，原语进入 `ACTIVE`。此后导航服务通过 `twist_in` 下发速度、订阅 `odom`；任务规划需要移动时调用 `service/navigation/*`，由其在内部调用 `move`。

---

## 能力自动发现

Robonix 通过 atlas 实现能力的自动发现，厂商无需在包之间手工配置话题名或地址：

- 提供方（本底盘原语）在 `on_init` 中通过 `declare_ros2_topic` / `create_publisher` 等向 atlas 声明能力，即建立"能力约定 ID → 通道端点（如 ROS 2 话题名）"的登记。
- 消费方（导航、建图、场景等）向 atlas 按能力约定 ID 查询，例如 `ATLAS.find_capability(contract_id="robonix/primitive/chassis/odom")`，atlas 返回提供方声明的端点，消费方据此建立连接。

提供方与消费方之间无需互知，仅通过能力约定 ID 经 atlas 对接。

---

## 常见问题

- 为什么 `move` 走 gRPC 而非 MCP？ `move` 下发的是未经避障的瞬时速度，不应暴露给大模型的工具列表。需要带路径规划的运动时，应经 `service/navigation/navigate`，由其组合安全目标与导航后再调用 `move`。
- 生命周期接口需要自己实现吗？ 不需要。`*/driver` 由 `Primitive` 基类自动提供并向 atlas 声明。
- C++ 节点与 Python 原语为何分两个进程？ `robonix-api` 是 Python 库；C++ 节点负责与硬件 SDK 交互的数据通路，Python 原语负责向 atlas 声明能力与生命周期。二者经 ROS 2（数据）与 gRPC（控制）协作，由同一个 `start.sh` 一并拉起。
- 硬件 / SDK 如何安装？ Robonix 不作约束——只要 `start.sh` 拉起的进程能正常运行并注册进 atlas 即可。

---

## 接入其他类型的本体

同一套流程可用于相机、激光雷达、IMU 等其它本体/传感器，区别仅在实现的能力约定不同。先在[接口目录](../interface-catalog/index.md)查阅对应本体的标准能力约定清单：

- 底盘 → [`primitive/chassis`](../interface-catalog/primitive/chassis.md)
- 相机 / 深度模组 → [`primitive/camera`](../interface-catalog/primitive/camera.md)
- 激光雷达 → [`primitive/lidar`](../interface-catalog/primitive/lidar.md)
- IMU → [`primitive/imu`](../interface-catalog/primitive/imu.md)
- 音频（麦克风 / 扬声器）→ [`primitive/audio`](../interface-catalog/primitive/audio.md)

将能力约定清单与数据节点的收发类型替换为对应本体即可，其余步骤完全一致。需要实现更复杂的逻辑（自带能力约定、技能包、服务包）时，参阅[开发者指南](../developer-guide.md)与 [Package 构建与代码生成](build-and-codegen.md)。

---

## 参考：厂商 SDK 形态 {#参考厂商-sdk-形态}

步骤 5 的分情形基于对真实厂商 SDK 的调研。下表按"是否已提供标准 ROS 2 话题"归类，供判断你的本体落在哪种情形（轮式底盘以情形 A 居多）：

| 形态 | 对应情形 | 代表 | 说明 |
|---|---|---|---|
| 自带 ROS 2 驱动，发标准 `/cmd_vel` + `/odom` | A | 松灵 Scout/Ranger `*_ros2`、LIMO | 核心 C++ 库 `ugv_sdk` + 独立的 `*_ros2` wrapper 仓库 |
| 自带 ROS 2，但话题带命名空间 / 非裸类型 | A（+ relay） | Clearpath Husky/Jackal | apt 安装；`/<ns>/cmd_vel` 用 `TwistStamped`，里程计为 `odometry/filtered` |
| 机器人固件原生跑 ROS 2 | A | iRobot Create 3 / TurtleBot 4 | 无驱动可写；`irobot_create_msgs` 用 deb 包安装 |
| 传感器：核心 SDK + 官方 ROS 2 wrapper（分仓） | A | RealSense、Orbbec、Livox | `librealsense2` + `realsense-ros` 等 |
| 核心 C++ SDK，无 ROS（走 CAN / UDP） | B-1 | 松灵 `ugv_sdk`、云深处 `Lite3_MotionSDK` | ROS-independent，CMake 构建 |
| pip 的 Python SDK，无 ROS | B-2 | UR `ur_rtde`、Unitree `unitree_sdk2_python` | `pip install`，无话题 |
| 自带 ROS 2，但专有消息（非 cmd_vel/odom） | C | Unitree `unitree_ros2` | 走 `/lowcmd` `/sportmodestate` 等，需适配 |

> 提示：腿足 / 小众本体（Unitree、云深处、Segway 等）多落在情形 B / C，需要自己写适配；轮式底盘多落在情形 A。Segway / Ninebot 等只有社区 ROS 2 驱动，集成前需自行评估其可靠性与发行版匹配。

参考文献（GitHub 仓库与官方文档，均访问于 2026 年 6 月）：

1. AgileX Robotics. `ugv_sdk`: AgileX 移动平台 C++ 控制库. GitHub. <https://github.com/agilexrobotics/ugv_sdk>
2. AgileX Robotics. `scout_ros2`: Scout 底盘 ROS 2 驱动. GitHub. <https://github.com/agilexrobotics/scout_ros2>
3. AgileX Robotics. `ranger_ros2`: Ranger 底盘 ROS 2 驱动. GitHub. <https://github.com/agilexrobotics/ranger_ros2>
4. AgileX Robotics. `limo_ros2`: LIMO 底盘 ROS 2 驱动. GitHub. <https://github.com/agilexrobotics/limo_ros2>
5. Clearpath Robotics. Robot Installation (ROS 2) 文档. <https://docs.clearpathrobotics.com/docs/ros/installation/robot/>
6. iRobot. Create 3 ROS 2 Interface 文档. <https://iroboteducation.github.io/create3_docs/api/ros2/>
7. Unitree Robotics. `unitree_sdk2`: 核心 C++ SDK（基于 CycloneDDS）. GitHub. <https://github.com/unitreerobotics/unitree_sdk2>
8. Unitree Robotics. `unitree_ros2`: ROS 2 DDS 桥接. GitHub. <https://github.com/unitreerobotics/unitree_ros2>
9. DEEP Robotics. `Lite3_MotionSDK`: 绝影 Lite3 运动控制 SDK（C++ / UDP）. GitHub. <https://github.com/DeepRoboticsLab/Lite3_MotionSDK>
10. Universal Robots. `Universal_Robots_ROS2_Driver`: 官方 ROS 2 驱动. GitHub. <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver>
11. SDU Robotics. `ur_rtde`: UR RTDE C++ / Python 接口库. GitLab. <https://gitlab.com/sdurobotics/ur_rtde>
12. Intel RealSense. `realsense-ros`: RealSense ROS 2 wrapper. GitHub. <https://github.com/IntelRealSense/realsense-ros>
13. Orbbec. `OrbbecSDK_ROS2`: Orbbec ROS 2 wrapper. GitHub. <https://github.com/orbbec/OrbbecSDK_ROS2>
14. Livox. `livox_ros_driver2`: Livox ROS 1 / ROS 2 驱动. GitHub. <https://github.com/Livox-SDK/livox_ros_driver2>
