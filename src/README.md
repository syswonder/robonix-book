<br>

<img src="robonix.svg" alt="robonix" width="350px" style="display: block; margin: 0 auto;">

<br>

欢迎使用 Robonix！

> 犀照世界 灵通万物 为机器筑心 为具身立智

Robonix 探索如何从系统层面构造具身智能的"大脑"，为具身智能提供跨越异构硬件的系统底座，让用户与开发者能更方便地开发、部署新模型，使机器人更容易掌握新技能、完成新任务。项目将 AI 模型与具身硬件软硬解耦，把模型视为可在运行时加载与组合的程序，并朝着"一次训练，任意机器部署运行"的方向演进。

围绕"感知–理解–规划–行动"全链路中数据处理、环境交互等共性问题，Robonix 设计"感知–互联–认知–控制"具身智能系统服务框架，降低模型与技能的开发与运行成本。我们期待 Robonix 能帮助各类机器人更易构筑智脑，推动软硬件独立演进的生态，让具身智能机器人更好用、更易用。

Robonix 是《具身智能操作系统技术白皮书》（CCF 泛在操作系统开放社区，2026）的参考实现。白皮书提出 EAIOS（Embodied AI Operating System）架构，采用"原语–服务–技能–任务"四级抽象体系。更多背景请参阅[白皮书原文](https://gitlink.org.cn/zone/uos/source/292)及 [EAIOS 架构背景](background/eaios.md)。

## 本手册导读

- [快速上手](getting-started/quickstart.md)——从克隆代码到运行 Tiago 仿真 Demo 的完整流程
- [系统全景](architecture/overview.md)——控制平面与数据面、组件关系、一个请求的完整链路
- [Crate 索引](architecture/crates.md)——五个 Rust crate 各自的职责和关键 API
- [命名空间与接口模型](architecture/namespace-and-interfaces.md)——namespace 树、abstract_interface_id、多传输、通道协商
- [接口目录](interface-catalog/index.md)——所有已定义的硬件原语和系统服务接口
- [接入指南](integration-guide/index.md)——硬件厂商和算法开发者将自己的组件接入 Robonix 的完整流程
