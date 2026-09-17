---
title: 使用和开发验证器（Verifier）
---

# 使用和开发验证器（Verifier）

本页面面向配置验证器或开发验证器（Verifier）的开发者。完成配置后，Executor 会在指定能力成功结束后调用验证器，并根据验证结果决定该 RTDL 节点的最终状态。

`verify` 是 Executor 提供的结果验证框架。Scene Verifier 和 VLM Verifier 是当前已经实现、可以使用的两个验证器；它们不是整个 `verify` 体系，开发者也可以按照公共能力约定实现自定义验证器。

## 使用验证器

在机器人部署清单的 `system.executor.verification` 中添加规则。下面是使用 Scene Verifier 验证导航结果的最小配置：

```yaml title="robonix_manifest.yaml"
service:
  - name: scene_verifier
    path: ${ROBONIX_SOURCE_PATH}/services/verifiers/scene_verifier

system:
  executor:
    verification:
      overlap: false
      rules:
        - target_contract_id: robonix/service/navigation/navigate
          target_provider_id: nav2
          verifier_provider_id: scene_verifier
          verifier_args:
            scene_provider_id: scene
```

`ROBONIX_SOURCE_PATH` 是 Robonix 源码目录。`service[].name` 必须与规则中的 `verifier_provider_id` 一致。仅启动 verifier 不会启用验证，目标调用还必须匹配一条规则。

每条规则包含以下字段：

| 字段 | 是否必填 | 含义 |
|---|---|---|
| `target_contract_id` | 是 | 要验证的能力约定 ID |
| `target_provider_id` | 否 | 只匹配指定能力提供方；省略时匹配实现该能力约定的任意提供方 |
| `verifier_provider_id` | 是 | 要调用的 verifier 的能力提供方 ID |
| `verifier_args` | 否 | 传给 verifier 的 JSON 对象；字段由具体 verifier 定义，默认值为 `{}` |

当提供方专用规则和只含 `target_contract_id` 的规则都能匹配时，Executor 优先使用提供方专用规则。相同匹配范围不能配置多条规则。没有匹配规则，或者目标能力本身失败、取消或超时时，Executor 不调用 verifier。

`overlap` 是整个 `verification` 配置的开关，默认为 `false`：

| 配置 | 可观察结果 |
|---|---|
| `overlap: false` | Executor 等待 verifier，随后只发出最终 `SUCCEEDED` 或 `FAILED`；后续顺序节点在验证完成后开始 |
| `overlap: true` | Executor 先发出非终态 `VERIFYING` 并继续 RTDL 树，验证完成后再发出唯一最终 `SUCCEEDED` 或 `FAILED` |

两种模式都会等待所有节点得到最终状态后再发出 `plan_complete`。验证不通过或 verifier 不可用都会使目标节点最终失败；验证通过时保留目标能力原来的成功输出。

### 使用当前实现的 verifier

Robonix 当前提供以下两个 verifier：

- [Scene Verifier](https://github.com/syswonder/robonix/tree/62afaf6cfc6a40b0cfec62acc0d09537bcecfd86/services/verifiers/scene_verifier) 验证 `map` 坐标系中的平面导航结果。规则需要提供 `verifier_args.scene_provider_id`；距离、朝向和观察超时可在服务配置中调整。
- [VLM Verifier](https://github.com/syswonder/robonix/tree/62afaf6cfc6a40b0cfec62acc0d09537bcecfd86/services/verifiers/vlm_verifier) 验证能通过 RGB 图像观察的抓取、放置等结果。规则需要提供 `verifier_args.camera_provider_id`，服务配置需要提供 VLM 地址、凭据和模型 ID。

使用 VLM Verifier 时，把 `verifier_provider_id` 改成 `vlm_verifier`，并在 `service` 中配置该软件包：

```yaml title="robonix_manifest.yaml"
service:
  - name: vlm_verifier
    path: ${ROBONIX_SOURCE_PATH}/services/verifiers/vlm_verifier
    config:
      vlm:
        base_url: ${VERIFY_VLM_BASE_URL}
        api_key: ${VERIFY_VLM_API_KEY}
        model: ${VERIFY_VLM_MODEL}
```

`VERIFY_VLM_BASE_URL`、`VERIFY_VLM_API_KEY` 和 `VERIFY_VLM_MODEL` 分别表示 VLM 服务地址、凭据和模型 ID。再把规则的 `verifier_args` 设置为 `camera_provider_id: CAMERA_PROVIDER_ID`；`CAMERA_PROVIDER_ID` 表示提供 `robonix/primitive/camera/rgb` 的准确能力提供方 ID。完整参数和限制以两个软件包各自的 README 为准。

## 开发自定义 verifier

自定义 verifier 只需作为能力提供方实现 [`robonix/service/verifier/verify`](../interface-catalog/service/verifier.md)。Executor 负责匹配规则和发起调用，verifier 负责读取上下文、取得独立证据并返回判断。

该能力约定使用以下请求和响应：

```text
string call_id
string args_json
---
bool passed
string detail
```

`args_json` 是由 Executor 构造的 JSON envelope：

```json
{
  "target_provider_id": "TARGET_PROVIDER_ID",
  "target_contract_id": "TARGET_CONTRACT_ID",
  "target_description": "预期结果",
  "target_args": {},
  "target_output": {},
  "verifier_args": {}
}
```

在软件包的 `package_manifest.yaml` 中声明能力约定：

```yaml
capabilities:
  - name: robonix/service/verifier/verify
```

下面的伪代码展示与 Scene/VLM 无关的最小 MCP 注册和返回方式：

```python
import json

from robonix_api import Service
from verifier_mcp import Verify_Request, Verify_Response

verifier = Service(id="custom_verifier", namespace="robonix/service/verifier")


@verifier.mcp("robonix/service/verifier/verify")
async def verify(req: Verify_Request) -> Verify_Response:
    envelope = json.loads(req.args_json)
    evidence = await observe_independent_evidence(envelope)
    passed, reason = evaluate_expected_result(envelope, evidence)
    return Verify_Response(passed=passed, detail=reason)
```

实现时应遵守以下要求：

- `passed=true` 表示独立证据足以确认预期结果，`passed=false` 表示证据确认结果不成立；两种情况都应返回非空 `detail`。
- `target_output` 只能作为上下文，不能单独作为结果成立的证据。
- 请求无效、证据来源不可用或无法形成有效响应时，应返回调用错误，而不是把未验证的结果标记为通过。
- 自定义 `verifier_args` 的字段、默认值和约束应在该软件包文档中说明。

`Verify_Request` 和 `Verify_Response` 由软件包构建时根据能力约定生成。确切字段和 envelope 语义见[结果验证接口](../interface-catalog/service/verifier.md)。

## 理解 verifier 原理

目标能力成功结束后，Executor 先按 `target_contract_id` 和可选的 `target_provider_id` 选择规则，再通过 MCP 调用该规则指定的 `verifier_provider_id`。调用中包含原始调用 ID、目标描述、参数、输出和规则配置：

```text
目标能力成功
  → Executor 匹配 verification rule
  → 调用 robonix/service/verifier/verify
  → verifier 取得独立证据并返回 passed/detail
  → Executor 生成目标节点的最终状态
```

Executor 使用失败关闭（fail-closed）语义：`passed=true` 保留原始成功结果；`passed=false` 产生以 `result verification failed:` 开头的失败；连接失败、超时、调用错误或响应格式错误产生以 `result verification unavailable:` 开头的失败。

同步模式在 verifier 返回后才继续执行。`overlap=true` 时，节点状态按 `VERIFYING → SUCCEEDED/FAILED` 收敛，RTDL 树可在验证期间继续；`plan_complete` 仍要等待所有验证得到最终状态，并把验证失败计入 `any_failed`。重叠验证不会取消或回滚已经开始的后续节点，因此存在结果依赖时应使用同步模式。

叶子进入 `VERIFYING` 后，原能力调用已经结束。此时取消方案不会取消 verifier，也不会把该叶子的最终状态改为 `CANCELED`；verifier 仍会使它收敛到 `SUCCEEDED` 或 `FAILED`，但尚未开始或仍在运行的其他节点以及祖先操作符仍可被取消。
