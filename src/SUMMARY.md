# Summary

[Robonix 开发手册](README.md)

---

# 开始使用

- [Webots 快速上手](getting-started/quickstart.md)
- [系统部署与启动](architecture/deployment-and-startup.md)
- [多平台与 Target Manifest](architecture/multiplatform-deployment.md)

# 接入机器人

- [接入路线](integration-guide/index.md)
- [本体接入指南](integration-guide/vendor-onboarding.md)
- [Package 与部署清单](integration-guide/packaging-spec.md)
- [构建与代码生成](integration-guide/build-and-codegen.md)
- [发布到 Package Catalog](integration-guide/package-catalog.md)

# 开发能力

- [开发者指南](developer-guide.md)

# 理解 Robonix

- [系统组件与数据流](architecture/components.md)
- [能力、命名空间与 Contract](architecture/namespace-and-contracts.md)
- [Atlas 能力目录](architecture/atlas.md)
- [EAIOS 架构背景](background/eaios.md)

# 接口目录

- [接口目录说明](interface-catalog/index.md)
  - [Primitive](interface-catalog/primitive/index.md)
    - [底盘 chassis](interface-catalog/primitive/chassis.md)
    - [机械臂 arm](interface-catalog/primitive/arm.md)
    - [相机 camera](interface-catalog/primitive/camera.md)
    - [激光雷达 lidar](interface-catalog/primitive/lidar.md)
    - [IMU](interface-catalog/primitive/imu.md)
    - [音频 audio](interface-catalog/primitive/audio.md)
    - [健康数据 health](interface-catalog/primitive/health.md)
    - [机器人描述 robot_description](interface-catalog/primitive/robot-description.md)
  - [Service](interface-catalog/service/index.md)
    - [空间地图 map](interface-catalog/service/map.md)
    - [导航 navigation](interface-catalog/service/navigation.md)
    - [语音 speech](interface-catalog/service/speech.md)
    - [声纹 voiceprint](interface-catalog/service/voiceprint.md)
    - [记忆 memory](interface-catalog/service/memory.md)
  - [System](interface-catalog/system/index.md)
    - [Soma 本体模型](interface-catalog/system/soma.md)
    - [Vitals 健康状态](interface-catalog/system/vitals.md)
    - [Pilot 任务规划](interface-catalog/system/pilot.md)
    - [Executor 方案执行](interface-catalog/system/executor.md)
    - [Liaison 用户交互](interface-catalog/system/liaison.md)
    - [Scene 场景与语义地图](interface-catalog/system/scene.md)

# 参考

- [参考入口](reference/index.md)
  - [能力约定参考（自动生成）](reference/contracts.md)
  - [ROS IDL 参考（自动生成）](reference/idl.md)
  - [代码 API](reference/api.md)

# 参与维护

- [文档平台与迁移边界](maintaining-documentation.md)
- [文档贡献与维护流程](contributing/documentation.md)
