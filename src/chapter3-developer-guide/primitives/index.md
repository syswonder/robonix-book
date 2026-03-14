# 抽象硬件原语（Primitives）

本章描述 Robonix 系统中各类抽象硬件的接口形态。每种硬件对应一个命名空间（`robonix/prm/*`），厂商按需实现接口子集，无需实现全部；同一领域内硬件形态各异（如纯深度相机、RGB 相机、红外相机），不存在统一最小契约。设计参考 Android HAL：接口定义与实现解耦，运行时通过 meta 解析 channel 完成通信。

## 硬件类型索引

| 类型 | 命名空间 | 说明 |
|------|----------|------|
| [相机 (Camera)](camera.md) | `robonix/prm/camera` | RGB、深度、RGB-D、红外、内参等视觉传感器 |
| [底盘 (Base)](base.md) | `robonix/prm/base` | 导航、运动控制、位姿、里程计、急停等 |
| [通用传感器 (Sensor)](sensor.md) | `robonix/prm/sensor` | 点云、激光雷达、IMU |
| [机械臂 (Arm)](arm.md) | `robonix/prm/arm` | 末端执行器运动、关节运动、状态、轨迹 |
| [夹爪 (Gripper)](gripper.md) | `robonix/prm/gripper` | 开合、宽度设置与状态 |
| [力/力矩 (Force-Torque)](force-torque.md) | `robonix/prm/force_torque` | 六维力/力矩传感器 |
