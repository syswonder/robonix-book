# 系统服务（`robonix/sys`，`kind = service`）

系统能力注册在 **`robonix/sys/`** 命名空间，与 **`robonix/prm`** 原语区分。已入库契约均在 **`rust/contracts/sys/*.v1.toml`**（完整树见 [接口目录首页 · 契约源码路径](../index.md#contract-toml-sources)）。

## 契约 ↔ 文档

| 契约 ID（`contract_id`） | 契约源码 | 文档 |
|--------------------------|----------|------|
| `robonix/sys/runtime/pilot` | `rust/contracts/sys/pilot.v1.toml` | [Pilot](pilot.md) |
| `robonix/sys/runtime/executor` | `rust/contracts/sys/executor.v1.toml` | [Executor](executor.md) |
| `robonix/sys/runtime/liaison` | `rust/contracts/sys/liaison.v1.toml` | [Liaison](liaison.md) |
| `robonix/sys/model/vlm/chat` | `rust/contracts/sys/vlm_chat.v1.toml` | [VLM Chat](vlm-chat.md) |
| `robonix/sys/memory/search` | `rust/contracts/sys/memory_search.v1.toml` | [Memory Search](memory-search.md) |

## 新增系统服务

1. 在 **`rust/robonix-interfaces/lib/`** 增加 IDL（若走 ridlc）。  
2. 在 **`rust/contracts/sys/`** 增加 **`*.v1.toml`**，`[contract] id` 即 **`contract_id`**。  
3. 运行 **`ridlc --contracts`** 更新 **`robonix_proto/`**（及 **`robonix_contracts.proto`**）。  
4. 将契约 ID 加入 Atlas **`ROBO_SYSTEM_INTERFACE_CATALOG`**（`grpc`/`ros2` 校验）。  
5. 在 **`service/`** 下增加一页，并把本表与侧栏 **`SUMMARY.md`** 链好。

## 预留命名空间（尚无契约）

| 路径前缀 | 用途 |
|---------|------|
| `robonix/sys/map/` | 语义地图 |
| `robonix/sys/planning/` | 规划 |
| `robonix/sys/manager/` | 技能 / 任务 / 模型管理 |
| `robonix/sys/debug/` | 调试工具 |
