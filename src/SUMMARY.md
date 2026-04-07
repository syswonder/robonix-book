# Summary

[Robonix](README.md)

---

- [EAIOS 架构背景](background/eaios.md)

---

- [快速上手（Tiago Demo）](getting-started/quickstart.md)
- [ManiSkill VLA 仿真 Demo](getting-started/maniskill-demo.md)

---

- [系统全景](architecture/overview.md)
- [Crate 索引](architecture/crates.md)
- [命名空间与接口模型](architecture/namespace-and-interfaces.md)
- [多 Transport 并发模型](architecture/multi-transport.md)

---

- [接口目录](interface-catalog/index.md)
  - [原语（prm）](interface-catalog/primitive/index.md)
    - [底盘](interface-catalog/primitive/base.md)
    - [相机](interface-catalog/primitive/camera.md)
    - [传感器](interface-catalog/primitive/sensor.md)
    - [机械臂](interface-catalog/primitive/arm.md)
    - [夹爪](interface-catalog/primitive/gripper.md)
    - [力/力矩](interface-catalog/primitive/force-torque.md)
  - [服务（sys）](interface-catalog/service/index.md)
    - [Pilot](interface-catalog/service/pilot.md)
    - [Executor](interface-catalog/service/executor.md)
    - [Liaison](interface-catalog/service/liaison.md)
    - [VLM Chat](interface-catalog/service/vlm-chat.md)
    - [Memory Search](interface-catalog/service/memory-search.md)

---

- [技能库](skill-library.md)

---

- [接入指南](integration-guide/index.md)
  - [Manifest 与包管理](integration-guide/package-and-manifest.md)
  - [Provider 注册](integration-guide/provider-registration.md)
  - [端到端验收](integration-guide/end-to-end-checklist.md)
