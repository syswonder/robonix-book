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

Skill 有两种互补形式，可独立使用，亦可在同一个包内共存：

| 形式 | 载体 | 作用 |
|------|------|------|
| **SKILL.md playbook** | 文本文件，随包注册到 Atlas | 向 VLM 描述行为规范：用哪些工具、按什么步骤、有哪些约束 |
| **Skill Node** | 运行中的进程，注册到 Atlas | 通过 MCP 暴露工具实现，供 Executor 调用 |

两者无强制绑定关系：SKILL.md 可单独存在（描述 primitive/service 已有工具的用法），Skill Node 亦可独立运行（不附带 playbook）。当两者共存时，SKILL.md 是描述 Skill Node 所暴露工具的最佳载体，可直接指导 VLM 正确使用这些工具。

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

**`tiago_sim_stack/skills/navigation/SKILL.md`**（精简）：

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

**`tiago_sim_stack/skills/visual_inspection/SKILL.md`**（精简）：

```markdown
---
name: visual_inspection
description: Capture and understand camera images using the robot's head camera and VLM.
---

# Visual Inspection Skill

Use the RGB camera to observe the environment. Preferred when the task requires
understanding scene content without motion planning.
```

## 包目录结构

Skills 以目录形式组织在包的 `skills/` 子目录下。每个子目录对应一个 skill，目录名不作约束，以 `SKILL.md` 中的 `name` 字段为准。

```
<package_root>/
  robonix_manifest.yaml
  skills/
    navigation/
      SKILL.md
    object_search_wander/
      SKILL.md
    visual_inspection/
      SKILL.md
```

`rbnx start` 会自动扫描 `skills/` 下的所有子目录——只要包含 `SKILL.md`，就会被解析并注册到 Atlas。

## 注册流程

Skills 在包启动时通过 `rbnx start` 注册到 Atlas，亦可由进程在 `RegisterNode` 时附带提交。

```
rbnx start <package>
    │
    ├── 扫描 <package_root>/skills/ 目录
    ├── 解析每个 SKILL.md frontmatter → SkillInfo { name, description, path, metadata_json }
    │
    ├── 连接 robonix-atlas（ROBONIX_ATLAS，默认 localhost:50051）
    └── RegisterNode(node_id, skills=[...])   ← 一次调用注册所有 skills
```

注册成功后，`rbnx start` 会打印：

```
  › Discovered 3 skill(s):
    - navigation : Navigate the Tiago robot to a target position...
    - object_search_wander : Find a visible target (e.g. door)...
    - visual_inspection : Capture and understand camera images...
  › Skills registered with robonix-atlas
```

Atlas 仅在 `skills` 字段非空时覆盖已注册的 skill 列表。节点自身随后发送的 `RegisterNode`（不含 skills）不会清除 `rbnx start` 预注册的 skills，二者互不干扰。

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
| 中 | `~/.robonix/skills/` | 用户管理的本地 skills，目录结构与包 `skills/` 相同 |
| 最高 | `ROBONIX_SKILLS_EXTRA_DIRS`（逗号分隔路径） | 工作目录或 CI 环境注入的 skills |

后加载的优先级更高；同名 skill 以最高优先级来源为准。

## 注入 System Prompt

合并后的 skills 以 XML 索引的形式注入 Pilot 的 system prompt：

```xml
<skills>
  <skill name="navigation"
         description="Navigate the Tiago robot to a target position in the map using Nav2."
         path="/opt/robonix/tiago/skills/navigation/SKILL.md"
         node="com.robonix.prm.tiago"/>
  <skill name="object_search_wander"
         description="Find a visible target (e.g. door) using camera + pose and approach with base_cmd only."
         path="/opt/robonix/tiago/skills/object_search_wander/SKILL.md"
         node="com.robonix.prm.tiago"/>
  <skill name="visual_inspection"
         description="Capture and understand camera images using the robot's head camera and VLM."
         path="/opt/robonix/tiago/skills/visual_inspection/SKILL.md"
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

### 与 SKILL.md 共存

Skill Node 和 SKILL.md 可在同一个包内共存。当 Skill Node 暴露自定义工具时，配套的 SKILL.md 是向 VLM 说明工具用法的最佳位置：

```
<package_root>/
  robonix_manifest.yaml
  skills/
    vla_manipulation/
      SKILL.md          # 描述 execute_instruction / move_base 的用法和约束
  maniskill_vla_demo/
    vla_node.py         # 暴露 execute_instruction、move_base 两个 MCP 工具
```

对应的 SKILL.md playbook 示例：

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

### 仅 SKILL.md

1. 在包的 `skills/<name>/` 目录下创建 `SKILL.md`
2. 填写 frontmatter（`name` + `description` 至少两字段）
3. 编写 playbook 正文：场景、工具列表、核心步骤、约束
4. `rbnx start <package>`（或重启包）→ skills 自动注册到 Atlas
5. `rbnx describe` 确认已注册；`rbnx tools` 确认 Agent 可见
6. `rbnx chat` 测试 Agent 能否正确调用对应行为

### Skill Node + SKILL.md

1. 实现 Skill Node 进程：`RegisterNode` → `DeclareInterface`（MCP）→ 启动 MCP HTTP server
2. 在 `skills/<name>/SKILL.md` 中描述节点暴露的工具及用法
3. `rbnx start <package>` 注册 SKILL.md；同时启动 Skill Node 进程
4. `rbnx tools` 确认 MCP 工具已出现在 Agent 可见工具集中
5. `rbnx chat` 验证 VLM 能依据 SKILL.md playbook 正确调用 skill 工具
