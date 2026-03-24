# Summary

[介绍](README.md)

# 背景与概念

- [EAIOS 具身智能操作系统](chapter0-background/eaios.md)
- [核心概念与实现对照](chapter0-background/concepts.md)

# 第一章：快速开始

- [快速上手](chapter1-getting-started/quickstart.md)

# 第二章：用户手册

- [用户手册概览](chapter2-user-guide/index.md)
  - [robonix-server](chapter2-user-guide/robonix-server.md)

# 第三章：开发文档

- [开发文档概览](chapter3-developer-guide/index.md)
- [节点开发指南](chapter3-developer-guide/package-development.md)
- [抽象硬件原语](chapter3-developer-guide/primitives/index.md)
  - [相机 (Camera)](chapter3-developer-guide/primitives/camera.md)
  - [底盘 (Base)](chapter3-developer-guide/primitives/base.md)
  - [通用传感器 (Sensor)](chapter3-developer-guide/primitives/sensor.md)
  - [机械臂 (Arm)](chapter3-developer-guide/primitives/arm.md)
  - [夹爪 (Gripper)](chapter3-developer-guide/primitives/gripper.md)
  - [力/力矩 (Force-Torque)](chapter3-developer-guide/primitives/force-torque.md)
- [硬件/服务厂商接入指南](chapter3-developer-guide/vendor-integration.md)
- [ridlc 手册](chapter3-developer-guide/ridlc.md)

# RFC 规范

- [RFC001: RIDL](rfc/RFC001-RIDL.md)
- [RFC002: 包管理](rfc/RFC002-Package-Management.md)
- [RFC003: 控制平面设计](rfc/RFC003-Control-Plane.md)
- [RFC004: MCP 集成](rfc/RFC004-MCP-Integration.md)
- [RFC005: SKILL.md 格式](rfc/RFC005-SKILL-Format.md)
- [RFC006: 多传输支持](rfc/RFC006-Multi-Transport.md)
- [RFC007: 零拷贝管线](rfc/RFC007-Zero-Copy-Pipeline.md)
