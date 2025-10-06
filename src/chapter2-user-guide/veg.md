# Virtual Entity Graph (VEG)

## 概述

Virtual Entity Graph (VEG) 是 robonix 的核心编程模型，提供了一种统一的方式来表示和管理具身智能系统中的各种实体及其关系。

VEG 将物理世界中的机器人、传感器、物体等抽象为**虚拟实体（Entity）**，并通过 **虚拟实体图（Virtual Entity Graph）** 结构来表达它们之间的关系和交互。

## 核心概念

### Entity（实体）

实体是 VEG 中的基本单元，代表物理世界或虚拟空间中的一个对象。每个实体具有：

- **唯一标识符 (ID)**：全局唯一的实体标识
- **类型 (Type)**：如 Robot、Sensor、Object、Space 等
- **属性 (Properties)**：描述实体的状态和特征
- **能力绑定 (Skill Binding)**：实体可以执行的技能
