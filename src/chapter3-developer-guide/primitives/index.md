# 抽象硬件原语（Primitives）

`robonix/prm/*` 命名空间下的接口描述相机、底盘、机械臂等硬件能力。厂商只需实现与本产品能力相符的子集。

## 硬件类型索引

| 类型 | 命名空间 | 说明 |
|------|----------|------|
| [相机 (Camera)](camera.md) | `robonix/prm/camera` | RGB、深度、RGB-D、红外、内参 |
| [底盘 (Base)](base.md) | `robonix/prm/base` | 导航、运动控制、位姿、里程计 |
| [通用传感器 (Sensor)](sensor.md) | `robonix/prm/sensor` | 点云、激光雷达、IMU |
| [机械臂 (Arm)](arm.md) | `robonix/prm/arm` | 末端/关节运动、轨迹、状态 |
| [夹爪 (Gripper)](gripper.md) | `robonix/prm/gripper` | 开合、宽度设置与状态 |
| [力/力矩 (Force-Torque)](force-torque.md) | `robonix/prm/force_torque` | 六维力/力矩传感器 |
