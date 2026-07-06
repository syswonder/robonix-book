# Summary

[Robonix](README.md)

---

- [EAIOS 架构背景](background/eaios.md)

---

- [快速上手（Tiago Demo）](getting-started/quickstart.md)

---

- [系统组件](architecture/components.md)
- [命名空间与能力约定](architecture/namespace-and-contracts.md)
- [Atlas 能力目录](architecture/atlas.md)
- [系统部署与启动流程](architecture/deployment-and-startup.md)
- [多平台部署（x86 / Jetson arm64）](architecture/multiplatform-deployment.md)

---
- [接口目录](interface-catalog/index.md)
  - [原语（primitive）](interface-catalog/primitive/index.md)
    - [底盘 chassis](interface-catalog/primitive/chassis.md)
    - [相机 camera](interface-catalog/primitive/camera.md)
    - [激光雷达 lidar](interface-catalog/primitive/lidar.md)
    - [IMU](interface-catalog/primitive/imu.md)
    - [音频 audio](interface-catalog/primitive/audio.md)
  - [服务（service）](interface-catalog/service/index.md)
    - [空间地图 map](interface-catalog/service/map.md)
    - [导航 navigation](interface-catalog/service/navigation.md)
    - [语音 speech](interface-catalog/service/speech.md)
    - [声纹 voiceprint](interface-catalog/service/voiceprint.md)
    - [记忆 memory](interface-catalog/service/memory.md)
  - [系统（system）](interface-catalog/system/index.md)
    - [Pilot 任务规划](interface-catalog/system/pilot.md)
    - [Executor 方案执行](interface-catalog/system/executor.md)
    - [Liaison 用户交互](interface-catalog/system/liaison.md)
    - [Scene 场景与语义地图](interface-catalog/system/scene.md)
    - [Soma 本体模型](interface-catalog/system/soma.md)

---

- [参考](reference/index.md)
  - [能力约定参考（自动生成）](reference/contracts.md)
  - [ROS IDL 参考（自动生成）](reference/idl.md)
  - [代码 API（rustdoc / Sphinx）](reference/api.md)

---

- [接入指南](integration-guide/index.md)
  - [本体接入指南](integration-guide/vendor-onboarding.md)
  - [开发者指南](developer-guide.md)
  - [Package 与部署规范](integration-guide/packaging-spec.md)
  - [Package Catalog 发布流程](integration-guide/package-catalog.md)
  - [Package 构建与代码生成](integration-guide/build-and-codegen.md)
