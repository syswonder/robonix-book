# 第二章：用户手册

<!-- toc -->

本章介绍日常使用与运维。

## 概述

- robonix-server：gRPC meta API + ping query 服务，负责节点注册与 Query channel 分配/解析。详见 [robonix-server](robonix-server.md)。
- callquery：Rust 命令行客户端，用于调用已注册的 query。
- python_ping_client：Python 示例客户端，演示 Python 包如何调用某个节点的 query。

## 使用流程

1. 启动 robonix-server：`./start_server`
2. 调用 query：`./callquery robonix-server robonix/system/debug/ping '"hello"'` 或使用 Python 示例：`rbnx start -p python_ping_client`（需先 `rbnx build -p python_ping_client`）
