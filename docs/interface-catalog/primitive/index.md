# 原语

原语（Primitive）是低层设备抽象。一个原语软件包注册一个提供方，并把设备输入输出统一成 `robonix/primitive/*` 能力约定；同一个提供方可以管理多个系统设备句柄。原语只反馈设备自身的物理事实、接受低层控制或输入流，不承担导航、建图或场景理解等上层职责。

| 域 | 覆盖 | 能力约定数 |
|----|------|--------|
| [机械臂](arm.md) | 关节指令与反馈、末端位姿 | 4 |
| [底盘](chassis.md) | 差速或全向底盘的速度控制与里程反馈 | 4 |
| [相机](camera.md) | RGB 与深度图像，流式与快照两种取图方式 | 7 |
| [激光雷达](lidar.md) | 2D 扫描与 3D 点云 | 4 |
| [惯性测量单元](imu.md) | 角速度、线加速度与姿态 | 2 |
| [音频](audio.md) | 麦克风采集、扬声器播放、设备选择与桥接发现 | 6 |
| [设备健康](health.md) | 设备健康快照与连续数据流 | 3 |
| [机器人描述](robot-description.md) | 机器人描述能力提供者的生命周期入口 | 1 |

表中 8 个域共有 31 条标准能力约定，不表示某个部署已经注册全部接口。字段和线协议结构以[自动生成的能力约定与 IDL 参考](../../reference/index.md)为准。

当前 Webots Tiago Lite 部署提供底盘、RGB-D 相机、2D 激光雷达和音频，不提供机械臂、IMU 或 3D 点云；机器人描述由模拟器已有的 `robot_state_publisher` 负责，健康原语由具体硬件部署提供。Webots 内置 primitive 位于 `examples/webots/primitives/`。可跨机器人复用的实现应放在独立软件包仓库中，例如 [ALSA 音频原语](https://github.com/syswonder/primitive-audio-driver-rbnx) 和 [客户端音频桥](https://github.com/syswonder/primitive-audio-client-bridge-rbnx)。
