# 第三章：开发文档

本章面向 Robonix 开发者、硬件厂商和服务接入人员，介绍系统的命名空间设计、多通道传输机制和硬件原语规范。

## 章节索引

- [命名空间与接口契约](namespace-contracts.md) — 命名空间树、抽象接口 ID、`node_id` 规范、标准命名空间与 IDL 源码的对应关系
- [抽象接口标识（协议 ID）](abstract-interface-id.md) — 控制平面字段、路径语法、`robonix/` 与厂商扩展、IDL → ridlc → 注册的流程
- [系统接口规范（RPC / Pub-Sub）](interface-spec.md) — 仅两种逻辑模式；ROS IDL 与 gRPC 的对应；Pub-Sub 在 gRPC 上用何种 streaming 模拟
- [多通道传输](multi-transport.md) — 支持的传输类型、接口声明、通道协商、端口分配策略
- [抽象硬件原语](primitives/index.md) — `robonix/prm/*` 下的标准硬件能力描述（相机、底盘、机械臂等）

## 面向不同角色

| 角色 | 建议阅读 |
|------|---------|
| 硬件厂商 | [命名空间与接口契约](namespace-contracts.md) → [抽象硬件原语](primitives/index.md)：了解如何注册设备、实现哪些接口 |
| Robonix 开发者 | 全部：理解控制平面交互和传输机制 |
| 服务接入人员 | [命名空间与接口契约](namespace-contracts.md) → [多通道传输](multi-transport.md)：了解如何将算法服务接入 Robonix |
