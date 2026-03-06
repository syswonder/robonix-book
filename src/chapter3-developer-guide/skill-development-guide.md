# 技能开发指南（基本技能）

<!-- toc -->

本文档说明如何将基本技能（basic skill）接入 Robonix。技能是可被任务执行器复用的动作语义单元；当前系统对 RTDL 技能尚未实现完整执行逻辑，此处仅介绍基本技能的接入方式与 Topic 协议。

## 技能在系统中的角色

技能不需要符合统一的能力 Spec，名称、参数、实现由开发者自定。部署时在 `recipe` 中选择要注册的技能；执行 `rbnx deploy register` 时，会将技能信息（`name`、`start_topic`、`status_topic`、`entry`、`start_args` 等）写入 robonix-core。任务执行器在执行 RTDL 中 `type: "skill"` 的指令时，按技能名调用 QuerySkill，取得 `start_topic`、`status_topic`，向 `start_topic` 发布一条 JSON 字符串（`std_msgs/String`）触发执行，并订阅 `status_topic` 等待结束或错误。技能进程由你在包内实现（例如 Python 节点），通过 `start_script` 启动；进程内需订阅 `start_topic`、发布 `status_topic`，并完成实际逻辑（可再调用原语或服务）。

接入流程：实现“订阅 `start_topic`、发布 `status_topic`”的进程，在 manifest 中声明 basic 技能，在 `recipe` 中列出该技能并执行 register 与 start。

## 通过 Topic 触发技能：协议说明

### 传输与类型

`start_topic` 与 `status_topic` 均使用 `std_msgs/msg/String`，消息内容为 JSON 字符串。执行器向 `start_topic` 发布一条 String，技能进程订阅该 topic 收到后解析 JSON 并执行；技能进程向 `status_topic` 发布 String，执行器订阅并解析，以判断是否完成或失败。

### 启动消息（执行器发往 start_topic）

执行器发布格式为：

```json
{
  "skill_id": "<skill_exec_id>",
  "params": { ... }
}
```

- `skill_id`：本次执行的唯一 ID（由执行器生成），技能在状态反馈中必须原样带回，以便执行器区分并发或多次调用。
- `params`：RTDL 中该技能的参数字典，由规划器或 RTDL 给出（例如 `{"target_object_id":"xxx"}`）。

技能端应根据 `params` 执行逻辑，并在所有 status 消息中带上相同的 `skill_id`，以便执行器正确过滤。

### 终止消息（执行器发往 start_topic）

当用户取消任务时，执行器会向同一 `start_topic` 再发一条消息：

```json
{
  "terminate": true,
  "skill_id": "<skill_exec_id>"
}
```

技能端收到后，若当前正在执行该 `skill_id`，应尽快结束并发布一次 `state: "cancelled"` 的 status。

### 状态消息（技能发往 status_topic）

技能端应发布 JSON 字符串，建议包含以下字段：`skill_id`（必填，与启动时一致）、`state`（取值为 `running`、`finished`、`error` 或 `cancelled`）、`result`（任意 JSON 可序列化的结果）、`errno`（0 表示成功，非 0 表示错误）。执行器会按 `error`、`message`、`error_message` 的顺序提取错误信息。

示例：

```json
{
  "skill_id": "skl::wandering_0_task_0",
  "state": "finished",
  "result": { "message": "Wandering completed", "iterations": 5 },
  "errno": 0
}
```

执行器会等待收到 `state` 为 `finished`、`error` 或 `cancelled` 后，再继续下一条指令或标记任务失败、取消。

## 在 rbnx_manifest.yaml 中声明基本技能

```yaml
skills:
  - name: skl::wandering
    type: basic
    start_topic: /robot1/skill/wandering/start
    status_topic: /robot1/skill/wandering/status
    entry: bash rbnx/start_wandering.sh
    start_args: '{"wander_radius":"number (optional)"}'
    status: '{"state":"string","result":"any"}'
    metadata: '{"domain":"indoor","capability":["navigation"]}'
    version: 0.0.1
    start_script: rbnx/start_wandering.sh
    stop_script: rbnx/stop_wandering.sh
```

- `name`：技能名，RTDL 中 `"name": "skl::wandering"` 会用来调用 QuerySkill。
- `type`：基本技能固定为 `basic`。
- `start_topic` / `status_topic`：与上述协议一致；执行器向 `start_topic` 发布、订阅 `status_topic`。
- `entry`：basic 技能的入口，可为可执行命令或脚本（例如 `bash rbnx/start_wandering.sh`），通常与 `start_script` 一致。
- `start_args` / `status`：JSON 字符串，描述参数与状态结构，供规划或 UI 使用；执行器不强制校验。
- `start_script`：`rbnx deploy start` 时在包根目录执行，用于启动技能进程（该进程内部订阅 `start_topic`、发布 `status_topic`）。

## 技能进程实现要点

1. 订阅 `start_topic`（`std_msgs/String`），解析 JSON：若包含 `terminate` 且 `skill_id` 与当前执行一致，则停止并发布 `state: "cancelled"`；否则取 `skill_id`、`params` 开始执行，并至少发布一次 `state: "running"`。
2. 发布 `status_topic`（`std_msgs/String`）时，每条消息都应带 `skill_id`、`state`、`result`、`errno`（以及可选的 `error`、`message`），结束时发送 `finished` 或 `error`、`cancelled`。
3. 若技能依赖原语（例如导航、相机），在进程内通过 QueryPrimitive 查询 `input_schema`、`output_schema`，得到 Topic 后再订阅或发布；可参考 [robonix-sdk](robonix-sdk.md) 的 Python 客户端。

完整示例见仓库中 `provider/navigation_skills/wandering_skill.py`（订阅 start、发布 status、处理 terminate、调用 prm::base.pose.cov 与 prm::base.navigate）。

## Recipe 中启用技能

```yaml
packages:
  - name: navigation_skills_provider
    skills:
      - skl::wandering
      - skl::move_to_object
```

若不写 `skills` 字段，则会注册该包在 manifest 中列出的全部技能。

## 与任务/RTDL 的关系

规划服务（例如 `srv::task_plan`）返回的 RTDL 当前为 list 形式的 JSON 数组。执行器只处理 `type: "skill"` 的指令：根据 `name` 查询技能库，再按上述 Topic 协议调用；`service`、`primitive` 类型的指令当前未实现执行逻辑。RTDL 技能（`type: "rtdl"`）在 manifest 中需填写 `skill_dir`、`main_rtdl`，系统侧对应的执行逻辑尚未实现，本文档仅覆盖基本技能。

## 小结

| 项目 | 说明 |
|------|------|
| 触发方式 | 执行器向技能的 `start_topic` 发布 JSON（含 `skill_id`、`params`） |
| 终止方式 | 执行器向同一 `start_topic` 发布 JSON（`terminate` + `skill_id`） |
| 反馈方式 | 技能向 `status_topic` 发布 JSON（`skill_id`、`state`、`result`、`errno`） |
| 消息类型 | 均为 `std_msgs/String`，内容为 JSON 字符串 |
| 注册与启动 | manifest 声明后，在 recipe 中列出技能，再执行 `rbnx deploy register` 与 `start` |
