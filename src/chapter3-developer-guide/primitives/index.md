# 抽象硬件原语（Primitives）

本章概括 Robonix 的**抽象硬件原语**（文档里常简称**原语**）：即 `robonix/prm/*` 等命名空间下，对相机、底盘、机械臂等能力的接口化描述。每一**条**接口在 RIDL 里还会选用 **stream / command / query** 中的一种作为**通信语义**（子页表格中的「通信语义」列）；请勿把这三者与「硬件原语」混为一谈——后者指**抽象哪类硬件**，前者指**该接口如何通信**。

各子页列出可选接口；厂商只需实现与本产品能力相符的**子集**。同一子域内设备差异很大，文档给出的是**推荐组合**而非强制最小集合。

## 硬件类型索引

| 类型 | 命名空间 | 说明 |
|------|----------|------|
| [相机 (Camera)](camera.md) | `robonix/prm/camera` | RGB、深度、RGB-D、红外、内参等视觉传感器 |
| [底盘 (Base)](base.md) | `robonix/prm/base` | 导航、运动控制、位姿、里程计、急停等 |
| [通用传感器 (Sensor)](sensor.md) | `robonix/prm/sensor` | 点云、激光雷达、IMU |
| [机械臂 (Arm)](arm.md) | `robonix/prm/arm` | 末端执行器运动、关节运动、状态、轨迹 |
| [夹爪 (Gripper)](gripper.md) | `robonix/prm/gripper` | 开合、宽度设置与状态 |
| [力/力矩 (Force-Torque)](force-torque.md) | `robonix/prm/force_torque` | 六维力/力矩传感器 |
