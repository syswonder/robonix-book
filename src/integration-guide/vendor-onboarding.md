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

"TOML + ROS IDL" 是 Robonix 规定的**描述语法**。它只刻画"这个能力承载什么形状的数据"，**本身不关心用什么通信方式传输**。底盘、机械臂、相机等的标准契约都已用这套语法定义好，随主仓库一起分发（`capabilities/` 存 TOML，`capabilities/lib/` 存 IDL）。

### 1.2 codegen：把描述投射到各种通信方式

`robonix-codegen`（经 `rbnx codegen` / `rbnx ros2-idl` 调用）读取上述与通信方式无关的描述，为 Robonix 目前支持的**每一种通信方式**生成实现/消费该能力所需的全部代码：

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

底盘的标准契约如下（IDL 在 `capabilities/lib/chassis/`，TOML 在 `capabilities/primitive/chassis/`）。"传输方式"一列是参考实现采用的方式；**除 `driver` 固定为 gRPC 外，其余均为推荐，厂商可自行选择**：

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

底盘的 `twist_in`、`odom` 走 ROS 2 话题，其载荷类型须采用 Robonix 的标准定义（[见 1.3](#13-消息类型一律以-robonix-的-idl-为准)）。在项目根目录生成并编译这套消息包：

```bash
rbnx ros2-idl -o robonix_idl_ws        # 生成到 robonix_idl_ws/src/
cd robonix_idl_ws
colcon build                            # 用任意 ROS 2 工具链编译（推荐 Humble）
source install/setup.bash               # 之后你的 ROS 2 节点即使用 Robonix 的类型定义
cd ..
```

`source` 之后，当前 shell 中的 `geometry_msgs`、`nav_msgs` 等均来自 Robonix 的定义。后续的 C++ 节点与 Python 原语在启动时都需要 source 这套环境（见步骤 7）。

---

## 步骤 5：用 C++ SDK 实现数据通路（一个 ROS 2 节点）

底盘的数据通路（接收速度、反馈里程计）由一个 **ROS 2 节点**承担。当厂商 SDK 是 C++（cmake + `.so` + 头文件）时，编写一个 C++ ROS 2 节点：它链接你的 SDK，同时依赖步骤 4 生成的 Robonix 消息包，订阅速度话题转交 SDK、读取 SDK 里程计发布出去。

> 若你的 SDK 是 Python，可跳过本步，在步骤 6 的 Python 原语中直接用 `rclpy` 收发话题。

在项目下新建一个 ROS 2 包 `ros2_nodes/my_chassis_node/`，包含 `package.xml`、`CMakeLists.txt` 与 `src/chassis_node.cpp`。

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
source robonix_idl_ws/install/setup.bash
cd ros2_nodes && colcon build && cd ..
```

至此，底盘的数据已在 ROS 2 图中：节点订阅 `/cmd_vel`、发布 `/odom`，且消息类型为 Robonix 的标准定义。

---

## 步骤 6：实现 Python 原语

Python 原语负责把上述能力**声明给 atlas**（使其可被导航、建图发现），并实现 `driver` 生命周期与 `move` 命令。它不直接接触 SDK——数据已由步骤 5 的 C++ 节点经 ROS 2 承载。

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

# Robonix 标准 ROS 2 消息 + 你的 C++ 节点（步骤 4、5 的编译产物）。
source "$DEPLOY_ROOT/robonix_idl_ws/install/setup.bash"
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

编辑项目根目录的 `robonix_manifest.yaml`，在 `primitive:` 下加入本包：

```yaml
primitive:
  - name: my_chassis              # 必须与 main.py 中 Primitive(id=...) 完全一致
    path: ./primitives/my_chassis
    config:
      device: /dev/ttyUSB0        # 可选；通过 Driver(CMD_INIT) 传入 on_init 的 cfg
```

**唯一的一致性要求**：部署清单中本条目的 `name` 必须与 `main.py` 中 `Primitive(id=...)` 完全相同——`rbnx boot` 据此确认进程已注册成功，不一致将导致启动失败。

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
