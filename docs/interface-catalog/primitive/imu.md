---
title: 惯性测量单元
---
<span id="imu-robonixprimitiveimu"></span>
# 惯性测量单元（IMU）

IMU 原语反馈惯性测量（角速度、线加速度、姿态），`topic_out` 数据面可供三维建图和状态估计使用，例如支持 IMU 与点云融合的建图提供方。`topic_out` 只规定单向输出流，不保证具体建图实现已经接入，也不绑定 ROS 2 或 gRPC 传输。

能力约定 TOML 在 `capabilities/primitive/imu/`，接口定义语言（Interface Definition Language，IDL）文件在 `capabilities/lib/common_interfaces/sensor_msgs/`。

新软件包省略 Driver 条目，由框架自动注册共享的 `robonix/lifecycle/driver`；显式选择共享 Driver 的行为相同。未实现生命周期回调时，框架记录警告并执行空操作。

:::warning[后向兼容：惯性测量单元命名空间 Driver]
`robonix/primitive/imu/driver`、`lifecycle/Driver` 和 `primitive/imu/driver.v1.toml` 只用于仍由软件包自行维护 Driver TOML 的旧实现。目前仍可使用，但计划迁移到共享 Driver；两种 Driver 不能同时注册。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/primitive/imu/imu` | `topic_out` | [`sensor_msgs/Imu`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-imu-msg) | `primitive/imu/imu.v1.toml` |

消费方（如建图服务）应通过 Atlas 按 `robonix/primitive/imu/imu` 能力约定和所需传输方式发现 IMU 数据面，不硬编码 `/livox/imu` 之类的话题名。

当前 Webots Tiago Lite 部署没有 IMU，Robonix 源码树也没有内置 IMU 提供方。接入硬件时需要实现并注册该能力约定，同时确认目标建图提供方明确支持 IMU 输入。
