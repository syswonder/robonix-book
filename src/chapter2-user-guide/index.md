# 第二章：用户手册

本章介绍日常使用与简单运维：如何启动核心服务、如何用命令行或示例程序验证 query 通路。

## 组件说明

- **[robonix-server](robonix-server.md)**：提供 gRPC meta API 与内置 ping；负责节点注册，以及 Query 等 channel 的分配与解析。
- **callquery**：Rust 命令行工具，适合快速调用**已注册**且载荷较简单的 query。
- **python_ping_client**：Python package 示例，演示如何在业务代码里通过生成客户端调用某个 node 上的 query。

## 典型流程

1. 在 `rust` 目录执行 `./start_server`，确认 meta 与 ping 正常。
2. 验证 query：执行 `./callquery robonix-server robonix/system/debug/ping '"hello"'`；或先 `rbnx build -p python_ping_client`，再 `rbnx start -p python_ping_client`（若 manifest 含多个 node，请使用 `-n <nodes[].id>` 指定要启动的节点）。
