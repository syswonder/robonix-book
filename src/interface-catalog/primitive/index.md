# 硬件原语（`robonix/prm`）

`RegisterNode.kind` 通常为 **`primitive`**，对应底盘、相机、传感器、机械臂等与物理或仿真硬件直接相关的契约。载荷与 gRPC 形状以 **`rust/robonix-interfaces/lib/**`** 为准；已入库的契约 TOML 见 **`rust/contracts/prm/*.toml`**（与 **`[contract] id`** 对齐），总表见 [接口目录首页 · 契约源码路径](../index.md#contract-toml-sources)。

| 子域 | 页面 |
|------|------|
| `robonix/prm/base` | [底盘](base.md) |
| `robonix/prm/camera` | [相机](camera.md) |
| `robonix/prm/sensor` | [传感器](sensor.md) |
| `robonix/prm/arm` | [机械臂](arm.md) |
| `robonix/prm/gripper` | [夹爪](gripper.md) |
| `robonix/prm/force_torque` | [力/力矩](force-torque.md) |
