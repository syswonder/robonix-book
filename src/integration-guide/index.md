# 接入指南

本章面向两类读者：需要将机器人或传感器接入 Robonix 的硬件厂商（实现 PRM provider），以及需要将感知、规划或模型服务接入 Robonix 的算法开发者（实现 System service）。两条路径的整体流程相同，区别在于命名空间与传输方式。

PRM provider 注册在 `robonix/prm/` 下，通常提供 MCP 工具接口（供 Agent 直接调用）和/或 gRPC/ROS 2 数据面接口（供其他节点消费）。System service 注册在 `robonix/srv/` 下，通常通过 gRPC 提供 RPC 接口。

MCP 数据面：声明 MCP 接口（`supported_transports=["mcp"]`）前，须先通过 `robonix-codegen --lang mcp` 生成 `robonix_mcp_types/`，并确保 `robonix_py` 包在 `PYTHONPATH` 中。工具实现使用 `mcp_contract` 装饰器。详见 [Provider 注册](provider-registration.md)和[快速上手](../getting-started/quickstart.md)中的"MCP 与 codegen"一节。

本章以两个示例贯穿全部内容：

- 示例 A（PRM）：`tiago_bridge` — Tiago 底盘与相机的 MCP + gRPC + ROS 2 桥接，代码位于 `rust/examples/packages/tiago_sim_stack/tiago_bridge/node.py`
- 示例 B（Service）：`vlm_service` — VLM 推理服务，gRPC 接口，代码位于 `rust/examples/packages/vlm_service/vlm_service/service.py`

接入流程分三步：

1. [编写 Manifest 和构建脚本](package-and-manifest.md)——定义包的元数据、节点和构建方式；若含 MCP，在 `build.script` 中调用 `robonix-codegen --lang mcp`（或采用与示例包相同的生成步骤），再构建镜像/环境。
2. [实现 Provider 注册](provider-registration.md)——在代码中调用 RegisterNode + DeclareInterface，启动数据面。
3. [端到端验收](end-to-end-checklist.md)——从 validate 到 Agent 可调用的完整检查清单。
