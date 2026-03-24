# 第二章：用户手册

本章介绍如何启动 Robonix 核心服务并验证基本功能。

## 组件说明

- [robonix-server](robonix-server.md)：控制平面，提供 gRPC API 用于节点注册、接口声明、channel 协商、技能查询。
- `robonix-sdk`：Rust SDK，封装 gRPC 客户端调用。
- `robonix-agent`：ReAct 智能体，通过 VLM 驱动工具调用。

## 典型流程

1. 启动 `robonix-server`（见下节）。
2. 各节点（sim_env、vla_service、vlm_service 等）启动后通过 gRPC 注册自身并声明接口。
3. 消费者通过 `NegotiateChannel` 协商通道，获取系统分配的端点后用原生 API 通信。
4. 运行 `robonix-agent` 进行交互式任务。

详见 `rust/examples/README.md` 和 `rust/examples/run_e2e.sh`。
