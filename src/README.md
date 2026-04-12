<br>

<img src="robonix.svg" alt="robonix" width="350px" style="display: block; margin: 0 auto;">

<br>

Robonix 具身智能操作系统

> 犀照世界 灵通万物 为机器筑心 为具身立智

Robonix 从系统层面构造具身智能的运行时底座，将 AI 模型与异构硬件软硬解耦，使模型成为可在运行时加载与组合的程序，朝"一次训练，任意机器部署运行"的方向演进。

围绕"感知–理解–规划–行动"全链路中的数据处理与环境交互等共性问题，Robonix 设计了"感知–互联–认知–控制"系统服务框架，降低模型与技能的开发及运行成本，推动软硬件独立演进的生态。

Robonix 是《具身智能操作系统技术白皮书》（CCF 泛在操作系统开放社区，2026）的参考实现。白皮书提出 EAIOS（Embodied AI Operating System）架构，采用"原语–服务–技能–任务"四级抽象体系。更多背景参阅[白皮书原文](https://gitlink.org.cn/zone/uos/source/292)及 [EAIOS 架构背景](background/eaios.md)。

## 本手册导读

- [快速上手](getting-started/quickstart.md)——从克隆代码到运行 Tiago 仿真 Demo 的完整流程
- [系统全景](architecture/overview.md)——控制平面与数据面、组件关系、一次请求的完整链路
- [Crate 索引](architecture/crates.md)——各 Rust crate 的职责与关键 API
- [命名空间与接口模型](architecture/namespace-and-interfaces.md)——namespace 树、契约 ID（`contract_id`）、`rust/contracts` 与 robonix-codegen、多传输、通道协商
- [接口目录](interface-catalog/index.md)——**prm**（Primitive，原语）与 **srv**（Service，服务）分目录说明
- [接入指南](integration-guide/index.md)——硬件厂商与算法开发者将自身组件接入 Robonix 的完整流程
