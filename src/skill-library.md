# 技能库（Skill Library）

## Skill 的定位

原语（prm）对应硬件驱动，服务（srv）对应系统中间件，而 **Skill** 是 Robonix 操作系统上的应用层。

Skill 描述的不是接口契约，而是 Agent 能用工具做什么——它向 VLM 声明：在何种场景下应使用哪些工具、以何种节奏交替感知与行动、需遵守哪些约束。每个 Skill 对应一类具有明确语义的行为能力（如"用 Nav2 导航""视觉搜索目标"），可独立部署、按需加载，并在 Agent 运行时动态组合。

```
原语（prm）  ←  硬件/仿真
服务（srv）  ←  框架中间件
技能（skill）←  Agent 可调用的行为应用  ← 本页
任务（task） ←  运行时 TaskGraph 实例
```

## Skill 的两种形式

Skill 有两种互补形式，可独立使用，亦可组合使用：

| 形式 | 载体 | 作用 |
|------|------|------|
| **`SKILL.md` playbook** | 独立存放的 Markdown 文件，显式注册到 Atlas | 向 VLM 描述行为规范：用哪些工具、按什么步骤、有哪些约束 |
| **Skill Node** | 运行中的进程，注册到 Atlas | 通过 MCP 暴露工具实现，供 Executor 调用 |

> **不再从包内自动扫描**：历史上 `rbnx start` 会扫描 `<package>/skills/<name>/SKILL.md` 自动注册，现已取消。`SKILL.md` 现在由用户 / 部署方**显式**放到 skill 注册路径下（例如 `~/.robonix/skills/` 或由 `ROBONIX_SKILLS_EXTRA_DIRS` 指定的目录），或由 Skill Node 进程在启动时自行调用 Atlas RPC 注册。详见下方 [Skill 与包的关系](#skill-与包的关系)。

## SKILL.md 格式

每个 Skill 是一个目录，其中必须包含一个 `SKILL.md` 文件。文件由两部分组成：YAML frontmatter（机器可读元数据）和 Markdown 正文（面向 VLM 的 playbook）。

### Frontmatter 字段

```yaml
---
name: <skill_name>                      # 必需；snake_case，全局唯一
description: <one-line description>     # 必需；注入 skills 索引，VLM 靠此选择 skill
disable-model-invocation: false         # 可选；true 时 skill 对 VLM 不可见（仅用于系统内部）
user-invocable: true                    # 可选；保留字段，供未来 UI 过滤使用
---
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `name` | string | Skill 的唯一标识符，snake_case，在 skills 索引和 CLI 中显示 |
| `description` | string | 一行描述，直接出现在 VLM system prompt 的 `<skills>` 索引中，决定 VLM 是否选择该 skill |
| `disable-model-invocation` | bool | 设为 `true` 时，该 skill 不进入 VLM 可见的 skills 索引（不影响注册存储）；默认 `false` |
| `user-invocable` | bool | 保留字段，默认 `true` |

### Playbook 正文

Frontmatter 之后的正文是面向 VLM 的**行为规范（playbook）**。正文由 `ROBONIX_SKILLS_INJECT_BODIES=1` 时整体注入 system prompt，或仅在需要时由 VLM 通过 `read_file` 按需读取 `path`。

Playbook 的写作目标是让 VLM 明确：
- 该 skill 适用的场景和禁用条件
- 可用的工具及其语义
- 完成任务的核心步骤（带顺序和约束）
- 与其他 skill 的协作或互斥关系

### 示例

**`~/.robonix/skills/navigation/SKILL.md`**（精简）：

```markdown
---
name: navigation
description: Navigate the Tiago robot to a target position in the map using Nav2.
---

# Navigation Skill

Send the robot to any (x, y) coordinate on the map with a specified heading.

## Available Tools

- `navigate_to(x, y, yaw, frame_id)` — queue a navigation goal
- `get_nav_status(goal_id)` — check goal progress (QUEUED → ACCEPTED → SUCCEEDED / ABORTED)
- `cancel_nav_goal(goal_id)` — cancel an in-progress navigation

## Typical Workflow

1. Call `navigate_to(x=2.0, y=1.5, yaw=0.0)` — returns a `goal_id`
2. Poll `get_nav_status(goal_id)` until status is `SUCCEEDED` or `ABORTED`
3. If stuck, call `cancel_nav_goal(goal_id)` and try an alternative route

## When the user forbids Nav2 tools

Use the **`object_search_wander`** skill instead: closed-loop `get_robot_pose` + `get_camera_image` + short `move_base` steps.
```

**`~/.robonix/skills/visual_inspection/SKILL.md`**（精简）：

```markdown
---
name: visual_inspection
description: Capture and understand camera images using the robot's head camera and VLM.
---

# Visual Inspection Skill

Use the RGB camera to observe the environment. Preferred when the task requires
understanding scene content without motion planning.
```

## Skill 与包的关系

**Skill 不再随包自动导入。**

早期 `rbnx start` 会自动扫描包内 `<package>/skills/<name>/SKILL.md` 并代表节点向 Atlas 注册这些 skills。这一机制已被**取消**——原因有二：

1. **概念耦合**：Robonix 把 Skill 定位为"Agent 行为应用"（原语 / 服务 / **技能** / 任务 四层之一），是与"包"正交的独立概念。让包自动产出 skills 把两层黏在一起，不同场景下加载同一个包会污染全局 skill 索引。
2. **语义冲突**：`SKILL.md` 是面向 VLM 的行为规范，和"包的开发者说明书"完全不同。早期的 `skills/<name>/SKILL.md` 既被当作注册源，又承担了包级文档职责，两者定位不清。

**当前约定**：

- **包级别的说明文档**：每个包根目录放一份 [`DESCRIPTION.md`](integration-guide/package-and-manifest.md#descriptionmd包说明文件)——介绍该包的接口 / 源码 / 函数 / 用法。面向开发者和集成方，**不注入 VLM prompt**。
- **Skill 注册**：由专门的 skill 注册路径管理（例如 `~/.robonix/skills/` 用户目录、`ROBONIX_SKILLS_EXTRA_DIRS` 环境变量、或 Skill Node 进程启动时主动调 `RegisterNode`），与 `rbnx start` 解耦。

换句话说：**一个包就是一个部署单元，负责进程 / 节点 / 接口；Skills 是独立的 Agent 能力库，按需挂到 Atlas**。如果想让某个包伴随注册 skill，由包自己在启动时显式调用 Atlas RPC（见下方 [Skill Node 部分](#skill-node)），而不是靠目录扫描副作用。

## Atlas Skill Library

Atlas 维护一个全局 skill 库，所有已注册节点的 skills 汇聚于此。

### `QueryAllSkills` RPC

Pilot 通过 `QueryAllSkills` 拉取全部 skill 信息：

```protobuf
rpc QueryAllSkills(QueryAllSkillsRequest) returns (QueryAllSkillsResponse);

message SkillInfo {
  string name = 1;          // snake_case 技能名
  string description = 2;   // 一行描述
  string path = 3;          // SKILL.md 在文件系统上的绝对路径（供 read_file 按需读取）
  string metadata_json = 4; // {"disable_model_invocation": false, "user_invocable": true}
}
```

响应中包含所有节点各自的 `SkillEntry`（含 `node_id`、`namespace`、`kind`、`skill_md` 遗留字段、`skills` 列表）。

### 查看已注册 Skills

```bash
rbnx describe                    # 展示所有节点的 SKILL.md 内容
rbnx describe --node com.robonix.prm.tiago   # 查看指定节点
rbnx tools                       # 列出 Agent 可见的全部工具（含 skill catalog 与 MCP 工具）
```

## Pilot 的加载与合并策略

Pilot 在每个推理轮次开始时调用 `QueryAllSkills`，并与本地 skills 三层合并：

| 优先级 | 来源 | 说明 |
|--------|------|------|
| 最低 | Atlas Registry（`QueryAllSkills`） | 已注册节点的 skills |
| 中 | `~/.robonix/skills/` | 用户管理的本地 skills，每个子目录一个 `SKILL.md` + 可选资源 |
| 最高 | `ROBONIX_SKILLS_EXTRA_DIRS`（逗号分隔路径） | 工作目录或 CI 环境注入的 skills |

后加载的优先级更高；同名 skill 以最高优先级来源为准。

## 注入 System Prompt

合并后的 skills 以 XML 索引的形式注入 Pilot 的 system prompt：

```xml
<skills>
  <skill name="navigation"
         description="Navigate the Tiago robot to a target position in the map using Nav2."
         path="/home/user/.robonix/skills/navigation/SKILL.md"
         node="com.robonix.prm.tiago"/>
  <skill name="object_search_wander"
         description="Find a visible target (e.g. door) using camera + pose and approach with base_cmd only."
         path="/home/user/.robonix/skills/object_search_wander/SKILL.md"
         node="com.robonix.prm.tiago"/>
  <skill name="visual_inspection"
         description="Capture and understand camera images using the robot's head camera and VLM."
         path="/home/user/.robonix/skills/visual_inspection/SKILL.md"
         node="com.robonix.prm.tiago"/>
</skills>
```

VLM 通过 `path` 字段按需调用内置工具 `read_file` 读取完整 playbook，无需在 system prompt 中预先展开全部内容。

### 完整注入（Playbook 注入模式）

设置 `ROBONIX_SKILLS_INJECT_BODIES=1` 可在 system prompt 中直接展开所有 skill 正文，适用于对话轮次较少、无需按需读取的场景：

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `ROBONIX_SKILLS_INJECT_BODIES` | `0` | `1` 时将所有 skill playbook 直接注入 system prompt |
| `ROBONIX_SKILLS_INJECT_MAX_CHARS` | `16000` | Playbook 注入的总字符预算 |
| `ROBONIX_SKILLS_INJECT_PER_SKILL_CHARS` | `4000` | 单个 skill 的最大注入字符数（超出截断） |

## Skill Node

Skill Node 是以运行进程形式存在的 skill：它向 Atlas 注册为节点，并通过 MCP 暴露工具供 Executor 发现和调用。与 primitive/service 不同，Skill Node 暴露的接口不受 contract 约束——不同场景和语义的多样性决定了不宜对 skill 工具强加统一的接口规范，工具的签名和语义完全由 skill 自身定义。

换言之，自然语言驱动的操作接口、任务级执行入口、策略封装工具，均不应伪装为 `robonix/prm/...` primitive contract。此类能力正是 Skill Node 的适用范畴。

### 注册模式

Skill Node 的注册流程与其他节点相同。能力类别不在 `RegisterNode` 的 `namespace`/`kind` 字段中声明（二者已废弃），而是通过 `DeclareInterface` 的 `contract_id` 体现。Skill Node 的接口应使用 `robonix/skill/...` 前缀，不应冒用基础原语命名空间：

```python
# 注册节点（不再使用已废弃的 namespace / kind 字段）
stub.RegisterNode(pb.RegisterNodeRequest(
    node_id="com.robonix.demo.vla",
))

# 能力类别通过 per-interface 的 contract_id 体现
stub.DeclareInterface(pb.DeclareInterfaceRequest(
    node_id="com.robonix.demo.vla",
    name="mcp_tools",
    supported_transports=["mcp"],
    metadata_json=json.dumps({"tools": [...]}),  # MCP 工具列表
    listen_port=mcp_port,
    contract_id="robonix/skill/manipulation/tools",  # 可选；仅用于发现，不属于 prm 标准面
))
```

Executor 通过 Atlas 发现 MCP 端点并连接获取工具列表，这些工具直接出现在 VLM 可见的工具集中。

### 与 SKILL.md 搭配

Skill Node 可以配一份 `SKILL.md` 放到 skill 注册路径下，向 VLM 说明该 Node 暴露的工具怎么用。`SKILL.md` 不再由包目录自动导入，部署时放到用户 skill 目录即可：

```
~/.robonix/skills/
  vla_manipulation/
    SKILL.md           ← 描述 execute_instruction / move_base 的用法和约束

<package_root>/        ← 包只负责进程 + MCP 工具实现
  robonix_manifest.yaml
  DESCRIPTION.md
  src/
    vla_node.py        ← 暴露 execute_instruction、move_base 两个 MCP 工具
```

或者 Skill Node 进程自己在 `RegisterNode` 时提交 `SkillInfo`，跳过文件系统中间环节（最紧凑的做法，适合一次性验证）。

对应的 `SKILL.md` playbook 示例：

```markdown
---
name: vla_manipulation
description: Execute closed-loop manipulation tasks using a VLA policy (Octo).
---

# VLA Manipulation Skill

Use this skill for pick-and-place or reach tasks in the ManiSkill3 simulation.

## Available Tools

- `execute_instruction(instruction, max_cycles, steps_per_cycle, sim_rate_hz)`
  — Run a closed-loop VLA predict→step loop. Returns `status: done/timeout`.
- `move_base(linear, angular, hold_steps)`
  — Directly command the mobile base (Fetch). Use for coarse repositioning.

## Typical Workflow

1. Call `execute_instruction("pick the red cup")` — VLA policy runs until done or timeout
2. If `status == "timeout"`, try repositioning with `move_base` then retry
3. Check `reward` in the result to confirm task success
```

### 注意事项

- Skill Node 不需要提供 SKILL.md，SKILL.md 也不要求对应一个运行节点
- `contract_id` 在 Skill Node 的 `DeclareInterface` 中是可选的，即使填写也仅作为标注用途，Atlas 不做接口 schema 的合规性校验
- Skill Node 的工具在 Executor 层被发现后，VLM 可以直接调用，无需额外配置

### 相似能力的动态选择

语义相似的 Skill Node（例如两个视觉导航 skill，分别适用于室内与室外场景）应在 SKILL.md 的 `description` 和 playbook 正文中明确各自的适用条件与限制。Pilot 将所有已注册 SKILL.md 以 `<skills>` 索引形式注入 system prompt，VLM 在推理时根据当前任务上下文自主选择最合适的 skill，无需额外路由机制。

### 典型案例：预训练 RL/VLA 模型作为 Skill Node

预训练的强化学习（RL）或视觉-语言-动作（VLA）模型在 Robonix 中以 Skill Node 形态运行。模型训练完成后封装为独立进程，通过 MCP 暴露执行入口（如 `execute_instruction`），并在 SKILL.md 中声明其能力与调用方式。

运行时，该 Skill Node 可以：

- 调用系统服务获取上下文信息（如定位服务获取机器人在环境中的位姿）
- 调用硬件原语（如关节角度控制、底盘运动）驱动物理执行
- 通过 SKILL.md playbook 指导 Pilot/VLM 在适当场景下选择并调用自身

这一"模型即 Skill Node"的模式使 Robonix 能够承载各类预训练策略，无需为每种模型定制框架层的集成方案。

## 典型开发流程

### 仅 `SKILL.md`

1. 在 skill 注册路径（`~/.robonix/skills/<name>/` 或 `ROBONIX_SKILLS_EXTRA_DIRS` 指定的目录）下创建 `SKILL.md`
2. 填写 frontmatter（`name` + `description` 至少两字段）
3. 编写 playbook 正文：场景、工具列表、核心步骤、约束
4. Pilot 下次启动（或重载 skill 索引）时读取到 → 自动出现在 `<skills>` 索引中
5. `rbnx describe` 确认已注册；`rbnx tools` 确认 Agent 可见
6. `rbnx chat` 测试 Agent 能否正确调用对应行为

### Skill Node（+ 可选 `SKILL.md`）

1. 实现 Skill Node 进程：`RegisterNode` → `DeclareInterface`（MCP）→ 启动 MCP HTTP server
2. 通过 `rbnx start <package>` 启动 Skill Node
3. 如需 VLM 行为提示：把 `SKILL.md` 放到 skill 注册路径，**或**让 Skill Node 在 `RegisterNode` 时附带 `SkillInfo`
4. `rbnx tools` 确认 MCP 工具已出现在 Agent 可见工具集中
5. `rbnx chat` 验证 VLM 能依据 playbook 正确调用 skill 工具

## Agent Skills 规范兼容

Robonix 的 SKILL.md 格式**刻意对齐** Anthropic/Claude 的 [Agent Skills 规范](https://docs.claude.com/en/docs/agents-and-tools/agent-skills)：同一份 frontmatter 字段（`name` / `description` / `disable-model-invocation` / `user-invocable`）、同一种"目录 + SKILL.md + 可选脚本/资源"的组织方式。这带来两个直接好处：

- **从 Claude Skills 生态直接导入**：`clawhub_skills` 等社区 skill 库可以不经改造注册进 Atlas（实际上 `rbnx` 已经集成了 OpenClaw 的导入能力，见 roadmap）。
- **反向导出**：Robonix package 里的 skills 可以直接作为 Claude Code / Claude Desktop 的 skill 使用，playbook 里的 `read_file` / tool 引用在两端语义一致。

与上游规范的差异仅在**扩展**而非**冲突**：

| 维度 | Claude Agent Skills | Robonix 扩展 |
|---|---|---|
| 目录结构 | `<skill_root>/<name>/SKILL.md` + 资源 | 同上（skill 注册路径，独立于包） |
| Frontmatter | `name` / `description` / `disable-model-invocation` / `user-invocable` | 同上 |
| 分发 | 用户目录 / Skill Marketplace | Atlas Registry 按节点注册 + `~/.robonix/skills/` + `ROBONIX_SKILLS_EXTRA_DIRS` |
| 工具实现 | Claude 侧内置 / MCP | MCP / gRPC / Built-in，由 Executor 三路派发 |
| 跨进程发现 | 本地文件 | `QueryAllSkills` RPC，多节点汇聚 |
| 与运行进程绑定 | 不绑定 | 可选绑定 Skill Node（见上节） |

换言之：**规范层沿用，分发/发现/执行层是 Robonix 自己的实现**。这样既享受生态的 skill 内容，又能把 skill 嵌到机器人分布式运行时里。

## 未来设想：把 Skill 当成 OS 对象管理

SKILL.md 和 Skill Node 解决了"skill 怎么写、怎么注册、怎么被 VLM 选中"，但 skill 作为**一等操作系统对象**还有更大空间。以下是一些长期方向，具体方案都未定，先把问题列在这里：

- **复用**：同一个 skill 在不同 agent / 不同机器人上反复执行，中间产物（子查询结果、轨迹、嵌入向量）有没有办法缓存复用，而不是每次从零跑？
- **运行前检查**：skill 依赖的 service、硬件、外部 API 是否就绪，能否在 VLM 真正调用前就判断清楚，而不是让模型在失败里反复试探？
- **共享与隔离**：多个 agent 同时运行时，哪些 skill 状态可共享、哪些必须隔离？目前完全靠 prompt 层的软约束。
- **故障恢复**：tool call 失败时能否按 skill 的步骤结构回滚到最近的安全点，而不是重跑整段（可能带副作用的）前序？
- **权限与审计**：每个 skill / 每个 agent / 每次 session 对底层工具的访问，能否用显式策略约束并留痕？Atlas 当前的 trust-on-first-use 在多机器人场景不够用。
- **并发控制**：多 sub-agent 并行跑 skill 时，对共享资源（机械臂、地图、摄像头流）的竞争能否由运行时保证？

这些方向的共同点是：把 skill 从"喂给 VLM 的文本"提升成**可缓存、可调度、可隔离、可审计的 OS 资源**。SKILL.md 规范只是入口，真正的工作量在下面这套 skill 运行时。相关条目概括登记在 [roadmap](roadmap.md) 的长期方向里。
