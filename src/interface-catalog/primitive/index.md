# 原语（primitive）

原语是**低层设备抽象**——一个物理设备一个包（one device = one package），把硬件统一成 `robonix/primitive/*` 契约。原语是能力图的叶子节点：只反馈自身的物理事实、只接受瞬时控制，不依赖任何上层服务（这条分层纪律见 [底盘](chassis.md) 页的历史变更说明）。

| 域 | 覆盖 | 契约数 |
|----|------|--------|
| [底盘 chassis](chassis.md) | 差速/全向底盘的速度控制与里程反馈 | 4 |
| [相机 camera](camera.md) | RGB / 深度图像，流式与快照两种取图 | 6 |
| [激光雷达 lidar](lidar.md) | 2D 扫描与 3D 点云 | 4 |
| [IMU](imu.md) | 惯性测量 | 2 |
| [音频 audio](audio.md) | 麦克风采集、扬声器播放、设备选择 | 5 |

参考实现见 `examples/webots/primitives/`（tiago_chassis / tiago_camera / tiago_lidar / audio_driver）。
