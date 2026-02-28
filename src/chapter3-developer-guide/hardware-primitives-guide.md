# Robonix HAL 原语设计与接入指南

作者：wheatfox <wheatfox17@icloud.com>

> **TODO**：本文档的设计目前暂未在代码仓库中实现

## 核心架构与隔离模型

Robonix HAL 采用逻辑命名槽位与物理域双重隔离架构，旨在彻底解决 ROS2 环境下 Topic 全局可见导致的安全风险。

### 物理域隔离
所有硬件 Provider 进程必须运行在受限的私有域中 ( `Domain ID != 0` )。该域与 System Domain 物理隔离。跨域通信由 Robonix Runtime 作为唯一网关进行数据中转。

### 命名槽位与随机注入
Provider 源代码中禁止硬编码物理 Topic 路径。Stub 层根据 RIDL 定义自动生成强类型配置结构。Runtime 在启动时动态生成随机 UUID 前缀的路径，并以键值对形式注入，其中 Key 严格对应 RIDL 中的参数名。



## 设计思路

### 物理隔离与 Topic 随机化提升安全性
传统的 ROS2 驱动中，任何接入网络的节点都可以通过扫描发现所有 Topic 路径。在 Robonix 架构下：
* **域隔离**：Provider 被物理锁定在私有 Domain，无法直接触达系统核心服务。
* **名称随机化**：即便攻击者进入了私有域，面对随机生成的 UUID 路径，也无法通过名称推测其业务含义。

### 核心逻辑解耦与开发提效
驱动开发者无需再为 Topic 路径、重映射或复杂的通信参数配置耗费精力：
* **零配置感知**：所有的物理连接由 Runtime 自动建立。
* **业务聚焦**：开发者只需继承 Stub 并实现定义的业务虚函数。硬件的更换与迁移只需修改 Manifest 映射，无需变动驱动核心代码。

## 原语定义规范 (RIDL)

RIDL 文件描述原语的数据契约。配置空间（Config Space）定义了参数的访问权限。

```yaml
# interfaces/base/move.ridl
name: prm::base.move
version: 1.0
type: stream

inputs:
  - name: cmd_vel
    message: geometry_msgs/msg/Twist

outputs:
  - name: odom
    message: nav_msgs/msg/Odometry

config:
  model: { type: string, access: "ro" }
  firmware: { type: string, access: "ro" }
  max_linear_mps: { type: number, unit: "m/s", access: "rw" }
  max_angular_rps: { type: number, unit: "rad/s", access: "rw" }
```

## 自动化代码生成 (Stub)

Stub 层封装了参数服务器接口，自动处理 `ro` 参数的初始化和 `rw` 参数的动态更新请求。

### C++ Stub 规范

```cpp
namespace robonix::prm::base {

class MoveProviderStub : public rclcpp::Node {
public:
    explicit MoveProviderStub(const std::string& node_name, 
                              const rclcpp::NodeOptions& options,
                              const std::map<std::string, std::string>& named_topics) 
        : Node(node_name, options) {
        
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            named_topics.at("cmd_vel"), 10, 
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(msg); });
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(named_topics.at("odom"), 10);

        // 声明参数（由 Runtime 注入初始值）
        this->declare_parameter("config.model", "");
        this->declare_parameter("config.firmware", "");
        this->declare_parameter("config.max_linear_mps", 0.0);
        this->declare_parameter("config.max_angular_rps", 0.0);
        
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MoveProviderStub::handle_parameter_update, this, std::placeholders::_1));
    }

    // 状态更新接口：由厂商同步硬件实际状态至系统域
    void sync_config(const std::string& key, double val) {
        this->set_parameter(rclcpp::Parameter("config." + key, val));
    }

    virtual void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) = 0;
    
    // 配置变更钩子：厂商需实现此函数以执行实际硬件写入
    virtual bool on_config_update(const std::string& key, const rclcpp::Parameter& val) = 0;

    void publish_odom(const nav_msgs::msg::Odometry& msg) { pub_odom_->publish(msg); }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult handle_parameter_update(const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
            if (!on_config_update(p.get_name(), p)) {
                result.successful = false;
                result.reason = "Hardware rejected parameter: " + p.get_name();
            }
        }
        return result;
    }
};

}
```

### Python Stub 规范

```python
# move_provider_stub.py
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult

class MoveProviderStub(Node):
    def __init__(self, node_name, named_topics, **kwargs):
        super().__init__(node_name, **kwargs)
        
        self._sub_cmd_vel = self.create_subscription(Twist, named_topics['cmd_vel'], self.on_cmd_vel, 10)
        self._pub_odom = self.create_publisher(Odometry, named_topics['odom'], 10)

        # 注册参数监听
        self.add_on_set_parameters_callback(self._internal_param_callback)

    def _internal_param_callback(self, params):
        for p in params:
            if not self.on_config_update(p.name, p.value):
                return SetParametersResult(successful=False, reason=f"HW reject {p.name}")
        return SetParametersResult(successful=True)

    def on_config_update(self, key, value):
        """由厂商实现硬件写入逻辑"""
        return True

    def sync_config(self, key, value):
        """由厂商调用同步状态"""
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter(f"config.{key}", value=value)])

    def on_cmd_vel(self, msg):
        raise NotImplementedError

    def publish_odom(self, msg):
        self._pub_odom.publish(msg)
```

## 厂商接入实现 (Implementation)

厂商只需继承 Stub 并实现业务语义转换及硬件配置同步逻辑。


### C++ 实现

```cpp
// ranger_provider.cpp
#include "move_provider_stub.hpp"
#include "ranger_sdk.hpp"

class RangerProvider : public robonix::prm::base::MoveProviderStub {
public:
    RangerProvider(const rclcpp::NodeOptions& options, 
                   const std::map<std::string, std::string>& named_topics)
        : MoveProviderStub("ranger_base_node", options, named_topics) {
        
        sdk_ = std::make_unique<RangerSDK::Driver>();
        
        // 获取初始配置（Default Values from Manifest）
        max_v_ = this->get_parameter("config.max_linear_mps").as_double();
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), [this](){
                auto data = sdk_->Read();
                geometry_msgs::msg::Odometry m;
                m.twist.twist.linear.x = data.speed;
                this->publish_odom(m);
            });
    }

    bool on_config_update(const std::string& key, const rclcpp::Parameter& val) override {
        if (key == "config.max_linear_mps") {
            max_v_ = val.as_double();
            return sdk_->UpdateLimit(max_v_);
        }
        return true;
    }

    void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) override {
        sdk_->Write(msg->linear.x, msg->angular.z);
    }

private:
    std::unique_ptr<RangerSDK::Driver> sdk_;
    double max_v_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

### Python 实现

```python
# ranger_provider.py
from move_provider_stub import MoveProviderStub
from nav_msgs.msg import Odometry
import ranger_sdk 

class RangerProvider(MoveProviderStub):
    def __init__(self, named_topics, **kwargs):
        super().__init__("ranger_base_node", named_topics, **kwargs)
        self.driver = ranger_sdk.RangerDriver()
        self.driver.connect()
        
        # 读取 Ro 配置：这些值由 Runtime 在启动时通过参数注入
        self.model_name = self.get_parameter('config.model').value
        self.firmware_v = self.get_parameter('config.firmware').value

        # 设置定时上报
        self.create_timer(0.05, self._publish_status)

    def on_config_update(self, key, value):
        """处理来自系统域的 rw 参数修改请求"""
        if key == "config.max_linear_mps":
            return self.driver.set_speed_limit(value)
        return True

    def on_cmd_vel(self, msg):
        """执行运动控制逻辑"""
        self.driver.drive(msg.linear.x, msg.angular.z)

    def _publish_status(self):
        """读取硬件反馈并发布"""
        state = self.driver.get_latest_state()
        msg = Odometry()
        msg.pose.pose.position.x = state.pos_x
        # 也可以在此处反向同步硬件侧产生的参数变化
        # self.sync_config("max_linear_mps", state.current_limit)
        self.publish_odom(msg)
```

## 部署清单 (Manifest)

`manifest.yaml` 负责声明 `ro` 参数并提供 `rw` 参数的默认启动值。

```yaml
# manifest.yaml
instances:
  - id: ranger.base.v1
    primitive: prm::base.move
    provider:
      runtime: ros2
      start: "./rbnx/start_prm_base_move.sh"
      stop: "./rbnx/stop_prm_base_move.sh"
      security_policy: "isolated_random" 
    
    config:
      # ro: 静态属性
      model: "Ranger-Mini-Pro"
      firmware: "2.0.1-stable"
      # rw: 默认初值 (Default Values)
      max_linear_mps: 1.2
      max_angular_rps: 2.0
```

## 交付与合规要求

Provider 源代码中严禁出现任何物理路径字符串。所有通信绑定必须通过 `named_topics` 完成。标记为 `access: ro` 的配置项由 Runtime 强制锁定。`rw` 项的变更必须通过 Stub 提供的 `on_config_update` 钩子与硬件状态保持同步。