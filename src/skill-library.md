# 技能库（Skill Library）

## Skill 的定位

如果说**原语（prm）** 是硬件驱动、**服务（sys）** 是系统中间件，那么 **Skill** 就是 Robonix 这个操作系统上的**应用**。

Skill 描述的不是接口契约，而是 **Agent 能用工具做什么**——它告诉 VLM：在什么场景下应该使用哪些工具、按什么节奏交替感知和行动、有哪些限制和约定。一个 Skill 对应一类**有具体语义的行为能力**（如"用 Nav2 导航"、"视觉搜索目标"），可以独立部署、按需加载、在 Agent 运行时动态组合。

```
原语（prm）  ←  硬件/仿真
服务（sys）  ←  框架中间件
技能（skill）←  Agent 可调用的行为"应用"  ← 本页
任务（task） ←  运行时 TaskGraph 实例
```

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

Skills 以目录形式组织在包的 `skills/` 子目录下。每个子目录对应一个 skill，目录名不限，以 `SKILL.md` 中的 `name` 字段为准。

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

Skills 通过 `rbnx start` 在包启动时注册到 Atlas，也可以由进程自行在 `RegisterNode` 时附带提交。

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

**关键**：Atlas 仅在 `skills` 字段非空时才覆盖已注册的 skill 列表。节点自身随后发送的 `RegisterNode`（不含 skills）不会清除 `rbnx start` 预注册的 skills，两者互不干扰。

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

**后加载的优先级更高**；同名 skill 以最高优先级来源为准。

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

VLM 通过 `path` 字段按需调用内置工具 `read_file` 读取完整的 playbook 文档，无需在 system prompt 中预先展开所有内容。

### 完整注入（Playbook 注入模式）

设置 `ROBONIX_SKILLS_INJECT_BODIES=1` 可在 system prompt 中直接展开所有 skill 的正文，适用于对话轮次较少、不依赖按需读取的场景：

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `ROBONIX_SKILLS_INJECT_BODIES` | `0` | `1` 时将所有 skill playbook 直接注入 system prompt |
| `ROBONIX_SKILLS_INJECT_MAX_CHARS` | `16000` | Playbook 注入的总字符预算 |
| `ROBONIX_SKILLS_INJECT_PER_SKILL_CHARS` | `4000` | 单个 skill 的最大注入字符数（超出截断） |

## 典型开发流程

1. 在包的 `skills/<name>/` 目录下创建 `SKILL.md`
2. 填写 frontmatter（`name` + `description` 至少两字段）
3. 编写 playbook 正文：场景、工具列表、核心步骤、约束
4. `rbnx start <package>`（或重启包）→ skills 自动注册到 Atlas
5. `rbnx describe` 确认已注册；`rbnx tools` 确认 Agent 可见
6. `rbnx chat` 测试 Agent 能否正确调用对应行为
