# 本体接入手册（厂商版）

本手册面向**机器人本体厂商的研发人员**，给出将一台硬件设备接入 Robonix 的完整、可照做的步骤。读者无需预先了解 Robonix 的内部架构。

全文以一台**差速底盘**为完整示例，并以厂商提供 **C++ SDK**（cmake + 动态库 `.so` + 头文件）为典型情形。其他类型的本体、其他形态的 SDK 遵循相同流程，差异点在文中标注。

---

## 1. 设计模型（请先阅读）

接入工作只涉及三个概念：

| 概念 | 含义 |
|---|---|
| **能力（capability）** | 系统中一个带稳定 ID 的接口，例如 `robonix/primitive/chassis/move`。 |
| **契约（contract）** | 对一个能力的描述：载荷数据的结构。底盘、机械臂、相机等的标准契约已由 Robonix 定义，厂商直接实现，无需自行设计。 |
| **原语（primitive）** | 一个直接驱动硬件的进程，向系统声明它实现了哪些能力。一台设备对应一个原语。 |

### 1.1 契约：一套与通信方式无关的描述

Robonix 用两份文件描述每个能力的契约：

- 一份 **TOML**：元数据——契约 ID、版本、种类（primitive / service / skill）、传输模式等。
- 一份 **ROS IDL**（`.msg` / `.srv`）：载荷的结构——消息或服务的字段。

"TOML + ROS IDL" 是 Robonix 规定的**描述语法**。它只刻画"这个能力承载什么形状的数据"，**本身不关心用什么通信方式传输**。底盘、机械臂、相机等的标准契约都已用这套语法定义好，随主仓库一起分发，位于 Robonix 源码树的 `capabilities/`（TOML）与 `capabilities/lib/`（IDL）下。

### 1.2 codegen：把描述投射到各种通信方式

`robonix-codegen`（经 `rbnx codegen` 调用）读取上述与通信方式无关的描述，为 Robonix 目前支持的**每一种通信方式**生成实现/消费该能力所需的全部代码：

| 通信方式 | codegen 产物 | 适用接口 |
|---|---|---|
| gRPC | protobuf 定义 + 各语言桩 | RPC 类接口（如 `move`、生命周期 `driver`） |
| MCP | 带类型的 Python dataclass | 暴露给大模型的工具 |
| ROS 2 | 可 `colcon` 编译的 IDL 消息包 | ROS 2 话题 / 服务 |

也就是说：同一份契约描述，codegen 按需生成 gRPC、MCP、ROS 2 各自所需的产物。厂商不自行定义任何消息或接口类型，只消费 codegen 为所选通信方式生成的结果。

### 1.3 消息类型一律以 Robonix 的 IDL 为准

所有类型的"形状"都源自同一套 Robonix IDL，因此各通信方式生成的代码彼此一致。这同样适用于 `geometry_msgs/Twist`、`nav_msgs/Odometry` 这类**看似"标准"**的类型——它们也在 Robonix 的 IDL 集合内，由 codegen 统一生成。厂商应使用这套生成结果，而**不是**某个 ROS 2 发行版自带的同名定义。

---

## 2. 流程概览

接入一台底盘，依次完成以下步骤：

1. 安装工具链
2. 创建部署项目与原语包骨架
3. 声明底盘要实现的契约
4. 生成 Robonix 标准 ROS 2 消息包
5. 用 C++ SDK 实现数据通路（一个 ROS 2 节点）
6. 实现 Python 原语（声明能力 + 生命周期）
7. 编写启动脚本
8. 登记到部署清单并启动验证

下面逐步展开。

---

## 步骤 1：安装工具链

当前版本中，构建工具 `rbnx` 与契约/IDL 定义随 Robonix 主仓库分发。先获取主仓库并安装：

```bash
git clone https://github.com/syswonder/robonix.git
cd robonix
make install                 # 安装 rbnx 等二进制，并把本仓库登记为契约/IDL 来源
pip install robonix-api      # Python 客户端库（原语基类、atlas 客户端、生命周期等）
```

环境要求：Rust 工具链（编译 `rbnx`，见 [rustup.rs](https://rustup.rs)）、Python ≥ 3.10、ROS 2 发行版（推荐 Humble）。

安装完成后确认可用：

```bash
rbnx --help
```

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
├── package_manifest.yaml      # 包元数据 + 契约清单
├── scripts/
│   ├── build.sh               # 已填好：调用 rbnx codegen
│   └── start.sh               # 已填好：设置 PYTHONPATH 并启动 python3 -m my_chassis.main
├── my_chassis/
│   ├── __init__.py
│   └── main.py                # Python 原语骨架（含 on_init 与 run）
└── capabilities/              # 仅当本包要自带契约时使用；本例留空
```

后续步骤在此骨架上填写。

---

## 步骤 3：声明底盘要实现的契约

底盘的标准契约如下（定义在步骤 1 克隆的 Robonix 源码树里：契约 TOML 在 `<robonix>/capabilities/primitive/chassis/`，IDL 在 `<robonix>/capabilities/lib/chassis/`；`<robonix>` 的绝对路径可用 `rbnx path capabilities` 查看。这些是 Robonix 预置的，你只实现、不修改）。"传输方式"一列是参考实现采用的方式；**除 `driver` 固定为 gRPC 外，其余均为推荐，厂商可自行选择**：

| 契约 ID | 传输方式 | 载荷 | 消费方 |
|---|---|---|---|
| `robonix/primitive/chassis/driver` | gRPC（固定） | 生命周期（框架内置） | `rbnx boot` 启动握手 |
| `robonix/primitive/chassis/move` | gRPC（推荐） | `chassis/MoveCommand` → `std_msgs/String` | 导航服务、遥操作 |
| `robonix/primitive/chassis/twist_in` | ROS 2 话题（推荐） | `geometry_msgs/Twist` | 导航控制器（下发速度） |
| `robonix/primitive/chassis/odom` | ROS 2 话题（推荐） | `nav_msgs/Odometry` | 建图、定位 |

将这四条契约填入 `primitives/my_chassis/package_manifest.yaml` 的 `capabilities` 段：

```yaml
capabilities:
  - name: robonix/primitive/chassis/driver
  - name: robonix/primitive/chassis/move
  - name: robonix/primitive/chassis/twist_in
  - name: robonix/primitive/chassis/odom
```

`package.name`（如 `com.vendor.my_chassis`）是包的发行标识，与运行时的原语 ID 无关，可保持骨架默认值。

---

## 步骤 4：生成 Robonix 标准 ROS 2 消息包

底盘的 `twist_in`、`odom` 走 ROS 2 话题，其载荷类型须采用 Robonix 的标准定义（[见 1.3](#13-消息类型一律以-robonix-的-idl-为准)）。这套消息包由 `rbnx codegen --ros2` 生成，与 gRPC/MCP 桩一样落在包的 `rbnx-build/codegen/` 下（`ros2_idl/`），由步骤 5 的 `build.sh` 一并产出，无需单独命令。生成的是源码，用 `colcon` 编译一次即可：

```bash
cd primitives/my_chassis/rbnx-build/codegen/ros2_idl
colcon build                            # 任意 ROS 2 工具链（推荐 Humble）
source install/setup.bash               # 之后 ROS 2 节点即使用 Robonix 的类型定义
cd -
```

`source` 之后，当前 shell 中的 `geometry_msgs`、`nav_msgs` 等均来自 Robonix 的定义。后续的 C++ 节点（编译期）与 Python 原语（运行期）都 source 这份 `install/setup.bash`（见步骤 5、7）。

---

## 步骤 5：实现数据通路（取决于厂商 SDK 是否已提供 ROS 2）

底盘的数据通路——接收速度命令、反馈里程计——走 ROS 2 话题。怎么做的**关键判断**是：你的本体**是否已经在 ROS 2 上给出标准的 `/cmd_vel`（速度）+ `/odom`（里程计）**。现实中，**多数轮式底盘厂商已经自带 ROS 2 驱动**（见[文末"参考：厂商 SDK 形态"](#参考厂商-sdk-形态)），所以情形 A 往往最省事；只有少数本体（多为腿足 / 小众）需要你自己写适配节点。

> **关于 ROS 2 发行版**：Robonix **不绑定**某个发行版（Humble / Foxy / Jazzy 均可）。但**同一次部署内，所有走 ROS 2 通信的原语与服务必须属于同一个发行版**——不同发行版的 DDS / 类型实现不兼容，混用会导致话题之间根本连不上。**当前推荐统一到 Humble。**

### 情形 A（最常见）：厂商已自带 ROS 2 驱动，已发布/订阅标准话题

很多本体开箱即在 ROS 2 上跑：松灵 Scout / Ranger 的 `*_ros2` 驱动直接订阅 `/cmd_vel`（`Twist`）、发布 `/odom`（`Odometry`）；iRobot Create 3 固件原生跑 ROS 2；相机 / 雷达（RealSense / Orbbec / Livox）都有官方 ROS 2 wrapper。

此时**不要再包一层**。把厂商驱动正常跑起来，步骤 6 的 Python 原语只用 `declare_ros2_topic` 把它的话题声明给 atlas（让导航 / 建图能发现），既不收也不发数据：

```python
chassis.declare_ros2_topic("robonix/primitive/chassis/twist_in", "<厂商驱动的 cmd_vel 话题>", qos="reliable")
chassis.declare_ros2_topic("robonix/primitive/chassis/odom",     "<厂商驱动的 odom 话题>",    qos="reliable")
```

**两个真实的坑**（务必核对，否则话题对不上）：

- **话题名常带命名空间**：例如 Clearpath 是 `/<robot_ns>/cmd_vel`，里程计叫 `odometry/filtered`（EKF 融合后的，不叫 `/odom`）。把 `declare_ros2_topic` 的话题名填成厂商驱动**实际**用的那个。
- **消息类型未必是裸标准类型**：例如 Clearpath 新平台用 `geometry_msgs/TwistStamped` 而非 `Twist`。若与 Robonix 契约要求的类型不一致，加一个小 relay 节点做转换；只有类型确实一致时才能直接 `declare`。

### 情形 B：厂商只有核心 SDK（C++ 或 Python），未接入 ROS 2

有些本体只给一个底层 SDK：松灵 `ugv_sdk` 是 ROS-independent 的 C++ 库（走 CAN）、云深处 Lite3 是 C++ over UDP、UR 的 `ur_rtde` 是 `pip` 装的 Python 库。这时由**你**写一个薄适配：订阅 `/cmd_vel` 调 SDK、读 SDK 状态发 `/odom`。按 SDK 语言二选一。

#### B-1　C++ SDK → 写一个 C++ ROS 2 节点

`ament_cmake` 工程，链接你的 SDK + 依赖步骤 4 的 Robonix 消息包；步骤 6 的 Python 原语退为薄层（只声明 + 生命周期 + `move`）。在项目下新建 `ros2_nodes/my_chassis_node/`（含 `package.xml` / `CMakeLists.txt` / `src/chassis_node.cpp`）。

**`CMakeLists.txt`**——`find_package` 同时引入 Robonix 消息包与你的 SDK：

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

**`src/chassis_node.cpp`**——订阅速度 → 调 SDK；读 SDK 位姿 → 发布里程计：

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

编译此节点（先 source 步骤 4 的消息包，使 `find_package(geometry_msgs)` 等解析到 Robonix 的定义）：

```bash
source primitives/my_chassis/rbnx-build/codegen/ros2_idl/install/setup.bash
cd ros2_nodes && colcon build && cd ..
```

至此底盘数据已在 ROS 2 图中：节点订阅 `/cmd_vel`、发布 `/odom`，消息类型为 Robonix 的定义。

#### B-2　Python SDK → 直接在 Python 原语里用 rclpy

不需要单独的 C++ 节点。在步骤 6 的 Python 原语 `on_init` 中：

```python
# 订阅速度命令，回调里调用你的 Python SDK。
chassis.create_subscription(
    "robonix/primitive/chassis/twist_in", topic="/cmd_vel", msg_type=Twist,
    callback=lambda m: sdk.set_velocity(m.linear.x, m.angular.z), qos="reliable")
# 里程计：建发布器，由一个读位姿线程周期 emit（msg_type=nav_msgs/Odometry）。
chassis.create_publisher(
    "robonix/primitive/chassis/odom", topic="/odom", msg_type=Odometry, qos="reliable")
```

`create_subscription` / `create_publisher` / `emit` 是 `robonix-api` 在内部 `rclpy` 节点上的封装，自动向 atlas 声明能力；你不必单独写 ROS 2 节点。其余（生命周期、`move`）和步骤 6 一致。

### 情形 C：厂商有 ROS 2，但用专有消息（非 `cmd_vel` / `odom`）

部分本体（多为腿足，如 Unitree 的 `unitree_ros2`）虽在 ROS 2 上，但走的是**专有消息**（如 `/lowcmd`、`/sportmodestate`），并不提供标准的 `/cmd_vel` + `/odom`。这时写一个适配节点（C++ 或 Python 皆可）在两者之间互转：订阅 `/cmd_vel` → 转成厂商的速度请求；订阅厂商状态 → 填成 `Odometry` 发 `/odom`。转出标准话题后，再按**情形 A** 用 `declare_ros2_topic` 声明给 atlas。

前提：这些话题的消息类型须与 Robonix 的定义一致。标准消息（`Twist` / `Odometry` 等）天然一致；若你的话题用到 Robonix **自定义**消息类型，则你的节点也要 build 在步骤 4 的 IDL 包上、用它生成的类型。

---

## 步骤 6：实现 Python 原语

Python 原语负责把数据通路的能力**声明给 atlas**（使其可被导航、建图发现），并实现 `driver` 生命周期与 `move` 命令。下面这份是**薄层**写法（情形 A / B-1 / C）：数据由厂商驱动或 C++ 节点经 ROS 2 承载，原语不直接接触 SDK，只做声明。若是**情形 B-2（Python SDK）**，则把步骤 5 的 `create_subscription` / `create_publisher` 也放进同一个 `on_init`，由原语兼任数据收发。

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
    # 把两条 ROS 2 话题能力声明给 atlas：契约 ID → C++ 节点所用的话题名。
    # 消费方据契约 ID 向 atlas 查询，即可发现并连接这些话题。
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
- `chassis.declare_ros2_topic(contract_id, topic, qos=...)` 把"契约 ID → 话题名"声明给 atlas。
- `@chassis.grpc(contract_id)` 将函数注册为该契约的 gRPC 处理函数；其请求/返回为 `rbnx codegen` 生成的 protobuf 消息。
- `chassis.run()` 阻塞运行，内部处理 gRPC、ROS 2、心跳与信号。

`driver` 生命周期接口由 `Primitive` 基类自动提供并声明，厂商无需自行实现，仅按需编写 `on_init`（必填）及可选的 `on_activate` / `on_deactivate` / `on_shutdown`。

---

## 步骤 7：编写启动脚本

本包需同时启动 C++ 节点（数据通路）与 Python 原语（声明与生命周期）。编辑 `primitives/my_chassis/scripts/start.sh`：

```bash
#!/usr/bin/env bash
set -eo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
DEPLOY_ROOT="$PKG_ROOT/../.."           # 项目根目录（按你的布局调整）
cd "$PKG_ROOT"

# Robonix 标准 ROS 2 消息（rbnx-build/codegen/ros2_idl）+ 你的 C++ 节点。
source "$PKG_ROOT/rbnx-build/codegen/ros2_idl/install/setup.bash"
source "$DEPLOY_ROOT/ros2_nodes/install/setup.bash"

# robonix-api 所在路径，使 `from robonix_api import ...` 可解析。
export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"

# 后台拉起 C++ 数据节点，前台运行 Python 原语。
ros2 run my_chassis_node my_chassis_node &
exec python3 -m my_chassis.main
```

`rbnx boot` 启动本包时会注入 `ROBONIX_ATLAS`（atlas 地址）等环境变量，无需在脚本中设置。

`scripts/build.sh` 由骨架生成、已调用 `rbnx codegen` 生成 `move` 等接口的 gRPC 桩，通常无需改动。

---

## 步骤 8：登记到部署清单并启动

到这里你交付的是一个**原语包**。把它跑起来还需要一份**部署清单** `robonix_manifest.yaml`——它列出"这台机器上要启动哪些系统组件、哪些设备、哪些服务"，由 `rbnx init`（步骤 2）在项目根目录生成。

> **谁来写这份清单？** 通常是**集成方/部署方**（把你的包和其它组件组装成一台完整机器人的人），而不是只交付一个设备驱动的硬件厂商。作为厂商你只需提供原语包 + 告诉集成方"在 `primitive:` 下加一条指向我的包"。本步是为了让你能在本地自测，也让你看清自己的包在部署里长什么样。

部署清单里，`primitive:` / `service:` 下的**每一条目就是一个硬件/能力实例（device instance）**。在 `primitive:` 下为你的底盘加一条：

```yaml
primitive:
  - name: my_chassis              # 实例名，必须 == main.py 里 Primitive(id=...)
    path: ./primitives/my_chassis # 指向你的包目录
    config:                       # 这一台设备实例的参数 → 经 Driver(CMD_INIT) 进 on_init 的 cfg
      device: /dev/ttyUSB0
```

- **一条 = 一台设备。** 同一个包可以被列多条来描述**多台同型号设备**——各用不同的 `name` 和 `config`（例如两台底盘 `chassis_left` / `chassis_right`，串口不同）。这就是"硬件实例"在清单里的表达方式。
- **`config` 是这台实例的私有参数**（串口、设备号、速度上限等），启动时序列化成 JSON、经 `Driver(CMD_INIT, config_json)` 注入到你 `on_init(cfg)` 的字典。放什么由你的包决定。
- **唯一的硬性一致要求**：条目的 `name` 必须与 `main.py` 里 `Primitive(id=...)` 完全相同——`rbnx boot` 据此确认进程注册成功，不一致会启动失败。`package_manifest.yaml` 的 `package.name` 与此无关。

构建、启动、验证：

```bash
rbnx build -p ./primitives/my_chassis   # 生成 gRPC 桩
rbnx boot                                # 启动 atlas、系统服务及清单中各包
rbnx caps                                # 应看到 my_chassis 的四条 chassis/* 能力为 ACTIVE
```

`rbnx boot` 会启动 atlas 与系统服务，运行本包的 `start.sh`，待进程注册后调用 `Driver(CMD_INIT)` 触发 `on_init`，原语进入 `ACTIVE`。此后导航服务通过 `twist_in` 下发速度、订阅 `odom`；任务规划需要移动时调用 `service/navigation/*`，由其在内部调用 `move`。

---

## 能力自动发现

Robonix 通过 atlas 实现能力的自动发现，厂商无需在包之间手工配置话题名或地址：

- **提供方**（本底盘原语）在 `on_init` 中通过 `declare_ros2_topic` / `create_publisher` 等向 atlas 声明能力，即建立"契约 ID → 通道端点（如 ROS 2 话题名）"的登记。
- **消费方**（导航、建图、场景等）向 atlas 按契约 ID 查询，例如 `ATLAS.find_capability(contract_id="robonix/primitive/chassis/odom")`，atlas 返回提供方声明的端点，消费方据此建立连接。

提供方与消费方之间无需互知，仅通过契约 ID 经 atlas 对接。

---

## 常见问题

- **为什么 `move` 走 gRPC 而非 MCP？** `move` 下发的是未经避障的瞬时速度，不应暴露给大模型的工具列表。需要带路径规划的运动时，应经 `service/navigation/navigate`，由其组合安全目标与导航后再调用 `move`。
- **生命周期接口需要自己实现吗？** 不需要。`*/driver` 由 `Primitive` 基类自动提供并向 atlas 声明。
- **C++ 节点与 Python 原语为何分两个进程？** `robonix-api` 是 Python 库；C++ 节点负责与硬件 SDK 交互的数据通路，Python 原语负责向 atlas 声明能力与生命周期。二者经 ROS 2（数据）与 gRPC（控制）协作，由同一个 `start.sh` 一并拉起。
- **硬件 / SDK 如何安装？** Robonix 不作约束——只要 `start.sh` 拉起的进程能正常运行并注册进 atlas 即可。

---

## 接入其他类型的本体

同一套流程可用于机械臂、相机等，区别仅在实现的契约不同。先在[接口目录](../interface-catalog/index.md)查阅对应本体的标准契约清单：

- **机械臂** → [`primitive/arm`](../interface-catalog/primitive/arm.md)
- **相机、深度模组** → [`primitive/camera`](../interface-catalog/primitive/camera.md)
- **底盘** → [`primitive/chassis`](../interface-catalog/primitive/chassis.md)

将契约清单与数据节点的收发类型替换为对应本体即可，其余步骤完全一致。需要实现更复杂的逻辑（自带契约、技能包、服务包）时，参阅[开发者指南](../developer-guide.md)与 [Package 构建与代码生成](build-and-codegen.md)。

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

> 提示：腿足 / 小众本体（Unitree、云深处、Segway 等）多落在情形 B / C，需要自己写适配；轮式底盘多落在情形 A。Segway / Ninebot 等只有**社区** ROS 2 驱动，集成前需自行评估其可靠性与发行版匹配。

参考仓库（截至调研时）：

- 松灵 AgileX：[`ugv_sdk`](https://github.com/agilexrobotics/ugv_sdk) · [`scout_ros2`](https://github.com/agilexrobotics/scout_ros2) · [`ranger_ros2`](https://github.com/agilexrobotics/ranger_ros2) · [`limo_ros2`](https://github.com/agilexrobotics/limo_ros2)
- Clearpath：[robot 安装文档](https://docs.clearpathrobotics.com/docs/ros/installation/robot/)
- iRobot Create 3：[ROS 2 API](https://iroboteducation.github.io/create3_docs/api/ros2/)
- Unitree：[`unitree_sdk2`](https://github.com/unitreerobotics/unitree_sdk2) · [`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2)
- 云深处 DEEP Robotics：[`Lite3_MotionSDK`](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)
- Universal Robots：[`Universal_Robots_ROS2_Driver`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) · [`ur_rtde`](https://gitlab.com/sdurobotics/ur_rtde)
- 传感器：[RealSense `realsense-ros`](https://github.com/IntelRealSense/realsense-ros) · [Orbbec `OrbbecSDK_ROS2`](https://github.com/orbbec/OrbbecSDK_ROS2) · [Livox `livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2)
