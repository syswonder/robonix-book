# RFC 005: SKILL.md 格式规范

| 版本 | 日期 |
|------|------|
| 0.1 | 2026-03-24 |

## 1. 目标与范围

约定 Robonix 中 SKILL.md 的文件格式，使节点技能可被 LLM 理解、被 MCP tool 的 `inputSchema` 直接使用，并与控制平面注册流程一致。

适用范围：primitive / service / skill 等逻辑单元均可附带一份 SKILL.md；一份文档描述一个可被发现、可被调用的技能单元。

不在本 RFC 内：技能的具体执行实现、RIDL 消息体与 SKILL 参数的一一编码规则（由实现与代码生成约定）。


## 2. 标准格式（必选结构）

SKILL.md 为 Markdown，顶层使用固定小节标题，便于解析与展示。

| 小节 | 含义 |
|------|------|
| Name | 技能名称（短名，建议稳定、唯一性由命名空间/节点上下文保证）。 |
| Description | 功能与使用场景描述，面向 LLM，说明何时应调用、前置条件与副作用（简述）。 |
| Parameters | 调用参数的 JSON Schema 定义（对象类型描述 `properties` / `required` 等）；须兼容 MCP 对 tool `inputSchema` 的用法。 |
| Returns | 返回值语义说明：成功时字段含义、错误时约定（可与应用错误码对齐，文本描述即可）。 |
| Example | 调用示例：可为 JSON 参数示例、伪代码或自然语言步骤，便于文档与 prompt 拼装。 |

解析器应容忍小节内多段 Markdown（列表、代码块）；Parameters 段内须包含可抽取的单一 JSON Schema 对象（如置于 ` ```json ` 代码块中），供服务器生成 MCP `inputSchema`。


## 3. 注册与存储

- 节点在 `RegisterNode`（见 RFC003）时提交 SKILL.md 全文或受控引用（实现可选：内联字符串 vs. 内容寻址存储，对外语义均为「该节点绑定该 SKILL」）。
- `robonix-server` 持久化或索引所有已注册 SKILL.md，并提供：
  - `QueryAllSkills` 等 gRPC 能力（聚合查询）；
  - 供 RFC004 MCP 映射使用的同一数据源。

更新技能时，应通过重新注册或官方规定的更新 API 刷新内容，避免客户端与 MCP 缓存长期不一致。


## 4. 与 MCP、RIDL 的边界

- MCP `inputSchema` 直接来自 Parameters 的 JSON Schema；Name / Description 映射到 tool 的 name / description（具体拼接规则由实现定义，须确定性、可测试）。
- RIDL 描述接口契约与传输语义；SKILL.md 描述「agent 如何理解并传参调用」。同一技能可同时有 RIDL 接口定义与 SKILL.md，二者应语义一致，冲突以注册校验或文档约定解决。


## 5. 示例骨架（非规范性附录）

小节顺序为 Name → Description → Parameters（JSON Schema 建议置于独立 `json` 围栏代码块中）→ Returns → Example。Parameters 示例：

```json
{
  "type": "object",
  "properties": {
    "object_id": { "type": "string" },
    "max_wait_s": { "type": "number", "default": 30 }
  },
  "required": ["object_id"]
}
```


## 6. 小结

SKILL.md 用 Name / Description / Parameters（JSON Schema）/ Returns / Example 五段统一描述技能；注册时经 `RegisterNode` 提交，由 `robonix-server` 集中维护，供 agent 与 MCP 使用。
