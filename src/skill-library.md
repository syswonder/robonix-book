# 技能库（Skill Library）

## Skill 在 Robonix 里的定位

Robonix 把系统分四层（详见 [EAIOS 架构背景](background/eaios.md)）：

```
原语（prm）  ←  硬件/仿真
服务（srv）  ←  框架中间件
技能（skill）←  Agent 可调用的行为应用 ← 本页
任务（task） ←  运行时 TaskGraph 实例
```

**Skill = 可复用的 Agent 行为单元**。粒度比单次 tool call 大，比一次完整任务小。一个 skill 把"在某个场景下要怎么做事"封装成 Agent 看得见、能在合适时机选择性调用的能力。

Robonix 区分两类 skill：

- **基本技能（basic skill）**：已经训好、固化下来的能力。最常见形态是预训练 RL / VLA 模型，封装成进程对外提供执行入口。
- **RTDL 技能（runtime-defined skill）**：运行期由 Pilot/规划器**动态生成**的结构化技能图（`TaskGraph` / Behavior Tree）——根据当前世界模型和任务目标拼出来的一次性 plan，验证后可固化。

## 基本技能

一个基本技能就是一个**进程**。它向 Atlas `RegisterNode`，通过 MCP 暴露一组工具，Executor 发现后路由到 VLM 可见的工具集。

与 primitive / service 不同，技能暴露的接口**不受统一 contract 约束**——技能本质上语义多样、签名自由（`pick(object_name)`、`go_through(door_id)`、`execute_instruction(prompt)` 各自不同），强加单一 schema 反而限制表达。

### 注册

`RegisterNode` 不再使用 `namespace` / `kind` 字段（已废弃）。能力类别通过每个 interface 的 `contract_id` 体现：

```python
stub.RegisterNode(pb.RegisterNodeRequest(node_id="com.robonix.demo.vla"))

stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id="com.robonix.demo.vla",
    name="mcp_tools",
    supported_transports=["mcp"],
    metadata_json=json.dumps({"tools": [...]}),  # MCP 工具列表
    listen_port=mcp_port,
    contract_id="robonix/skill/manipulation/tools",  # 标注用，Atlas 不校验 schema
))
```

### 典型形态：预训练模型即技能

把训好的 RL 或 VLA 模型封装成独立进程，通过 MCP 暴露执行入口（如 `execute_instruction(instruction)`），运行时该技能可以：

- 调用系统服务获取上下文（位姿、地图）
- 调用硬件原语驱动物理执行（关节角、底盘速度）
- 在自身合适时被 Pilot/VLM 选中调用

这样 Robonix 能承载各类预训练策略，**无需为每种模型写框架层适配**。

### 相似能力的动态选择

语义相近的技能（比如室内 vs 室外的视觉导航各一个）应在自身**描述文本**中讲清各自适用条件。Pilot 把所有已注册的描述以 `<skills>` 索引注入 system prompt，VLM 在推理时按当前任务上下文自主选择，无需额外路由机制。

## RTDL 技能（结构化技能图，规划中）

Pilot 当前每轮把 VLM 的 `tool_calls` 转成一份 `TaskGraph` 直接交 Executor 执行。这个 TaskGraph 是**一次性**的——没有名字、没有持久化、不可复用。

RTDL（Robot Task Description Language）方向把这个流程结构化：

- VLM 按定义的 schema 输出**完整结构化技能图**（含顺序、并行、条件分支、子 BT），不是平铺的 tool_calls
- 经过验证 / 跑通的技能图可以**命名固化**到 Atlas Skill Library
- 后续遇到相同任务，Pilot 可以**直接下发**已固化的技能图，跳过 VLM 推理

参考"持久化 Skill Graph"一项，详见 [roadmap](roadmap.md) 的长期方向。

## Atlas Skill Library

Atlas 维护全局 skill 库，所有已注册节点的 skills 汇聚于此。

### `QueryAllSkills` RPC

Pilot 通过 `QueryAllSkills` 拉取全部 skill 信息：

```protobuf
rpc QueryAllSkills(QueryAllSkillsRequest) returns (QueryAllSkillsResponse);

message SkillInfo {
  string name = 1;          // snake_case 技能名
  string description = 2;   // 一行描述（注入 <skills> 索引）
  string path = 3;          // 可选；若 skill 有 Markdown 文档，此处给绝对路径供 read_file 按需读取
  string metadata_json = 4; // 扩展字段
}
```

### Skill 信息从哪来

注册到 Atlas 的 SkillInfo 有几个来源：

1. **技能进程主动提交**：在 `RegisterNode` 时把自身的 `SkillInfo` 一并发上去（最直接）
2. **用户本地目录**：`~/.robonix/skills/<name>/` 下放 Markdown / 资源文件，Pilot 启动时扫描并注入
3. **CI / 工作目录注入**：`ROBONIX_SKILLS_EXTRA_DIRS` 环境变量指定额外目录

> Skill 的注册路径**与包（`robonix_manifest.yaml`）解耦**。早期 `rbnx start` 会扫描 `<package>/skills/` 自动注册，现已取消——包是部署单元（管进程 / 接口），Skill 是 Agent 能力库（管行为），两者正交。包的自身说明放包根目录的 [`DESCRIPTION.md`](integration-guide/package-and-manifest.md#descriptionmd包说明文件)。

### 查看已注册 skills

```bash
rbnx describe                                # 所有节点的 skill 信息
rbnx describe --node com.robonix.prm.tiago   # 指定节点
rbnx tools                                   # Agent 可见的全部工具（含 skill 与 MCP 工具）
```

## Pilot 加载与合并

Pilot 在每个推理轮次开始时调用 `QueryAllSkills`，并与本地来源三层合并：

| 优先级 | 来源 | 说明 |
|--------|------|------|
| 最低 | Atlas Registry（`QueryAllSkills`） | 已注册节点提交的 skills |
| 中 | `~/.robonix/skills/` | 用户管理的本地 skills |
| 最高 | `ROBONIX_SKILLS_EXTRA_DIRS`（逗号分隔路径） | 工作目录或 CI 环境注入的 skills |

后加载者优先级更高；同名 skill 以最高优先级来源为准。

### 注入 system prompt

合并后的 skills 以 XML 索引形式注入 Pilot 的 system prompt，VLM 看到的是一组**带描述、带可选 path** 的能力清单：

```xml
<skills>
  <skill name="navigate_to_pose"
         description="Send the robot to an (x,y,yaw) goal via Nav2."
         node="com.robonix.prm.tiago"/>
  <skill name="pick_object"
         description="Closed-loop VLA pick of a named object."
         path="/home/user/.robonix/skills/pick_object/playbook.md"
         node="com.robonix.demo.vla"/>
</skills>
```

`description` 必有；如有 Markdown 文档（`path` 字段），VLM 可用内置 `read_file` 工具按需读取详细 playbook，避免在 system prompt 里铺开全部内容。

### 完整注入模式

设置 `ROBONIX_SKILLS_INJECT_BODIES=1` 时，所有 skill 的文档正文直接展开到 system prompt（适合对话轮次少、不需要按需读取的场景）：

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `ROBONIX_SKILLS_INJECT_BODIES` | `0` | `1` 时直接展开正文 |
| `ROBONIX_SKILLS_INJECT_MAX_CHARS` | `16000` | 展开总字符预算 |
| `ROBONIX_SKILLS_INJECT_PER_SKILL_CHARS` | `4000` | 单 skill 正文最大字符数 |

## 未来设想：把 Skill 当成 OS 对象管理

技能和 RTDL 解决了"skill 怎么写、怎么注册、怎么被 VLM 选中"，但 skill 作为**一等操作系统对象**还有更大空间。以下是一些长期方向，具体方案都未定，先把问题列在这里：

- **复用**：同一个 skill 在不同 agent / 不同机器人上反复执行，中间产物（子查询结果、轨迹、嵌入向量）有没有办法缓存复用，而不是每次从零跑？
- **运行前检查**：skill 依赖的 service、硬件、外部 API 是否就绪，能否在 VLM 真正调用前就判断清楚，而不是让模型在失败里反复试探？
- **共享与隔离**：多个 agent 同时运行时，哪些 skill 状态可共享、哪些必须隔离？目前完全靠 prompt 层的软约束。
- **故障恢复**：tool call 失败时能否按 skill 的步骤结构回滚到最近的安全点，而不是重跑整段（可能带副作用的）前序？
- **权限与审计**：每个 skill / 每个 agent / 每次 session 对底层工具的访问，能否用显式策略约束并留痕？Atlas 当前的 trust-on-first-use 在多机器人场景不够用。
- **并发控制**：多 sub-agent 并行跑 skill 时，对共享资源（机械臂、地图、摄像头流）的竞争能否由运行时保证？

这些方向的共同点是：把 skill 从"喂给 VLM 的文本"提升成**可缓存、可调度、可隔离、可审计的 OS 资源**。相关条目概括登记在 [roadmap](roadmap.md) 的长期方向里。
