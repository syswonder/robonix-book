# 机械臂 robonix/prm/arm

## 已入库契约

### 操作执行（`manipulation/exec`）

接收自然语言或结构化字符串指令，输入输出均为 `std_msgs/String`，与上层策略/VLA 对齐。

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/prm/manipulation/exec` | `rpc` | `std_msgs/String` → `std_msgs/String` | `prm/manipulation_exec.v1.toml` |

> 契约 TOML 路径省略 `rust/contracts/` 前缀。MCP 工具实现使用 `mcp_contract` 装饰器，见 [接口目录首页](../index.md) 中「MCP 与 Python 工具线格式」。

## 规划中的接口（尚无 TOML）

以下接口为设计意图，IDL 包 `prm_arm` 尚未完整入库，暂无对应 TOML。新增时按 `rust/contracts/README.md` 规范创建版本化文件（如 `arm_move_ee.v1.toml`）。

| 契约 ID（`contract_id`） | 模式 | 载荷（IDL） | 契约 TOML |
|--------------------------|------|-------------|-----------|
| `robonix/prm/arm/move_ee` | `rpc` | `geometry_msgs/PoseStamped` → `std_msgs/String` | — |
| `robonix/prm/arm/move_joint` | `rpc` | `prm_arm/JointTarget` → `std_msgs/String` | — |
| `robonix/prm/arm/joint_trajectory` | `rpc` | `trajectory_msgs/JointTrajectory` → `std_msgs/String` | — |
| `robonix/prm/arm/state_joint` | `pub-sub (out)` | `sensor_msgs/JointState` | — |

## 典型组合

已有硬件实现 `manipulation/exec` 即可接入上层 Agent。完整关节控制的 provider 通常还需实现 `move_ee`（或 `move_joint`）+ `state_joint`，有轨迹跟踪需求时加 `joint_trajectory`。
