# RFC 007: 零拷贝管线设计（草稿）

| 版本 | 日期 |
|------|------|
| 0.1 | 2026-03-24 |

## 1. 目标与范围

目标：在 Camera → GPU → SLAM 等视觉链路上，尽可能实现全链路零拷贝，降低延迟与 CPU 带宽占用。

核心机制：以 DMABUF / NVMM（Jetson 等）等为基础，在进程间传递 buffer 句柄（fd 或等价），避免用户空间大块 `memcpy`。

当前状态：本文档处于设计阶段；原型暂不实现，仅作为后续实现与 RFC006 共享内存/Dora 等能力的延伸参考。


## 2. 核心机制

- 生产者（相机/GPU）分配可导出句柄的缓冲区（如 dmabuf fd）。
- 消费者（SLAM/推理）通过句柄导入同一物理内存，在 GPU/ISP 侧直接访问。
- 控制面仍可通过 Robonix 协商「哪条逻辑通道走零拷贝、句柄如何传递」（例如配合 `shared_memory` 或专用 fd 传递通道）；数据面不经过 `robonix-server` 转发。


## 3. Buffer 生命周期状态机

建议统一状态机，便于多算子协同与死锁避免：

| 状态 | 含义 |
|------|------|
| FREE | 缓冲区可回收或待分配。 |
| FILLING | 生产者正在写入（如传感器采集、GPU 渲染目标）。 |
| READY | 填充完成，可供下游读取。 |
| PROCESSING | 消费者占用中（SLAM/推理）。 |
| RELEASE | 已用毕，归还前清理引用，回到 FREE。 |

转移规则由实现保证单写者/多读者等约定，避免 USE-AFTER-FREE。


## 4. 三级调度模型

1. Buffer 调度：哪块 buffer 分配给哪条链路、池大小与回收策略。
2. Operator 调度：SLAM/检测等算子在 READY buffer 上的执行顺序与并行度。
3. Device 调度：GPU/ISP/NPU 时间与同步（fence/semaphore），与状态机转移挂钩。

三级相互约束：buffer 不足时算子让出；设备忙时延迟 READY→PROCESSING。


## 5. 与 ROS2 的关系

- 控制面：拓扑、参数、生命周期仍可用 ROS2（或 Robonix 控制面 + ROS2 共存）。
- 数据面：高带宽图像/点云优先走共享内存 + DMABUF/NVMM，不强制经 ROS 消息拷贝路径。

即：ROS2 管编排与元数据，零拷贝管热点数据面。


## 6. 小结与里程碑

零拷贝管线以 DMABUF/NVMM + 句柄传递为核心，用 Buffer 状态机与 Buffer / Operator / Device 三级调度约束生命周期与性能。与 ROS2 控制面 + 独立数据面分工一致。实现推迟：待 RFC006 多传输与运行时稳定后再立项原型。
