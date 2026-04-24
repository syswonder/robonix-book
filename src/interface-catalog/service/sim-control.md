# 仿真器控制服务（设计稿 / TODO）

Status：设计中，未实现。

## 做什么

给 Robonix 一层统一的仿真器控制接口。上层（Pilot、Agent、benchmark runner）只认这一组 RPC，底下换成 MuJoCo / Isaac Sim / Webots / Gazebo 不需要改代码。具体做四件事：

一，统一的物理时间控制（reset / step / pause / resume），让任何算法节点能以同一套语义推进仿真。

二，统一的场景构造操作（spawn / remove / set_pose / apply_wrench），让 Pilot 能根据任务需要往活动仿真里加物体、移动物体、施加外力。

三，统一的状态回读（`get_state`），把仿真当前的位姿/关节/接触信息抓出来转成 `SemanticScene`，直接喂进 `srv/semantic_map/*`。

四，能力自描述（`capabilities`），让上层查到当前后端支持哪些操作，避开不支持的。

这套接口和语义地图服务有很清晰的职责边界：语义地图服务只管语义信息本身的存取，仿真控制服务管"怎么让这份语义在某个仿真器里真的动起来"。

## 核心 RPC

命名空间 `srv/sim/*`，IDL 在 `rust/contracts/srv/sim_*.v1.toml`。

| Contract | 行为 |
|---|---|
| `srv/sim/reset` | 把仿真回到初始状态或某个命名快照 |
| `srv/sim/step` | 推进 n 个子步（参数：`n_substeps`，可选 `dt`） |
| `srv/sim/pause` | 暂停物理 |
| `srv/sim/resume` | 继续物理 |
| `srv/sim/get_state` | 抓当前仿真的完整姿态/关节/接触，返回 `SemanticScene` |
| `srv/sim/spawn_object` | 把一个 `Object`（可选 asset_uri）加进活动仿真 |
| `srv/sim/remove_object` | 从活动仿真拿掉一个 Object |
| `srv/sim/set_pose` | 强制一个 Object 的位姿（teleport，忽略物理约束） |
| `srv/sim/apply_wrench` | 在一个 Object 上施加力 + 力矩，持续指定时长 |
| `srv/sim/query_contacts` | 返回当前所有接触对 |
| `srv/sim/capabilities` | 返回当前后端实际支持的 RPC 子集和版本 |

所有 Object、Pose、Wrench 都是 Robonix 内部定义的 IDL 类型（和语义地图同一套），不暴露任何 `MjData` / `UsdStage` / `Supervisor` / `ROS2 Service` 的底层句柄。驱动内部把这些 handle 藏起来。

## 每个仿真器的落地示意

同一条 RPC，不同驱动翻译成原生 API：

| RPC | MuJoCo | Isaac Sim | Webots | Gazebo |
|---|---|---|---|---|
| `reset` | `mj_resetData` | `simulation_context.reset()` | `simulationReset()` | `/world/default/reset` service |
| `step` | `mj_step` 循环 | `simulation_context.step()` 循环 | `supervisor.step(ms)` | `/world/default/control` step |
| `get_state` | 遍历 `mjData.body` / `qpos` | USD stage traversal + PhysX query | `supervisor.getRoot()` 递归 | `/world/default/state` service |
| `spawn_object` | 编辑 `MjSpec` → `mj_recompile` | `stage.DefinePrim(...)` + Xform | `supervisor.getRoot().importFromString(...)` | `/world/default/create` service |
| `remove_object` | `MjSpec.detach(body)` + recompile | `stage.RemovePrim(...)` | `Node.remove()` | `/world/default/remove` service |
| `set_pose` | 写 `qpos` 或 `body_pos` | `UsdGeom.Xformable.Set(...)` | `Node.getField("translation").setSFVec3f(...)` | `/world/default/set_pose` |
| `apply_wrench` | `xfrc_applied` | OmniPhysics `applyForce` | `Node.addForce(...)` | `ApplyBodyWrench` service |
| `query_contacts` | `mjData.contact` 数组 | PhysX contact view | `supervisor.getContactPoints(...)` | `/world/default/contacts` topic |

驱动只要实现这套翻译层就能被 Robonix 挂进来。上层代码完全不感知底下是哪家。

## 能做到的边界

抽象到这一层成本：所有仿真器独有的能力进不来。举例：
- Isaac Sim 的 Replicator 随机化 pipeline
- MuJoCo 在线改 `mjModel` 约束或 contact filter
- Webots 的 Proto 模板实例化
- Gazebo 的 ROS2 插件自定义 topic

这些要暴露就走扩展通道（下节），核心 RPC 不尝试覆盖。

## 扩展通道

每个仿真驱动在 `srv/sim/ext/<backend>/*` 下登记自己专属 RPC：

| 命名空间 | 归属 |
|---|---|
| `srv/sim/ext/mujoco/*` | MuJoCo 驱动专属（MjSpec 编辑、mjcb 回调注册等） |
| `srv/sim/ext/isaac/*` | Isaac Sim 驱动专属（Replicator、Materials API 等） |
| `srv/sim/ext/webots/*` | Webots 驱动专属（Proto 实例化、Receiver/Emitter 等） |
| `srv/sim/ext/gazebo/*` | Gazebo 驱动专属 |

这些不是 Robonix 的统一抽象，只是允许调用方在必要时访问后端原生能力。调用方显式知道自己在用某个后端，并承担代码不可移植的代价。`capabilities()` 的返回值会列出当前活动驱动注册了哪些 ext RPC。

## 和语义地图的关系

两个服务之间是单向数据流加反向回写：

- 初始化：`srv/semantic_map/load` 加载一份 `SemanticScene` → 调用 `srv/sim/spawn_object` 把它铺进活动仿真。
- 执行期：上层动作让仿真里物体位姿变化 → 感知节点定期调 `srv/sim/get_state` 或直接观察 → 调 `srv/semantic_map/update_pose` 回灌到语义地图。
- 预演：Pilot 想"假设先把杯子挪过来看会怎样" → 克隆一份 `SemanticScene` → 在预演仿真实例上 `spawn_object` + `step` → 读结果 → 丢弃这份克隆，不污染主语义地图。

仿真控制服务不持有权威的语义信息，语义地图才是。仿真只是"场景变活动"和"预演"的执行者。

## 性能注意

每条 RPC 是跨进程 gRPC，单次调用有几百微秒的开销。适用于：
- Pilot 粒度的场景构造（reset / spawn / set_pose，每个任务几次到几十次）
- benchmark runner 节奏控制（reset / step 到 done / 回读）
- 预演一个 plan 评估可行性（step 百来下）

不适用于：
- RL 训练里每 step 都 RPC（1 kHz 级别）
- 仿真内部的紧循环控制（放 in-process skill/plugin 里）

高频/紧循环的场景走 in-process skill 而不是跨 service RPC。

## 当前状态 · TODO

- core RPC 的 IDL 定稿（`rust/contracts/srv/sim_*.v1.toml`）
- `capabilities()` 的字符串列表规范
- 先写一个 MuJoCo 驱动作为参考实现（`srv/sim/mujoco_driver`）
- 写一个 Isaac Sim 驱动打通第二家
- `get_state()` → `SemanticScene` 的字段映射约定（哪些仿真字段对应 Robonix 的哪些 Object 字段）
- 预演克隆语义：快照、克隆仿真实例、废弃副本的回收
