# 服务开发指南

<!-- toc -->

本文档说明如何将算法能力以标准服务（Service）形式接入 Robonix。接入流程可以概括为：实现符合系统规范（Spec）的 ROS2 服务、在包的 `manifest` 中声明该服务、通过 `recipe` 注册到 robonix-core，并在部署时通过 `rbnx` 启动服务进程。完成后，任务规划与执行即可通过标准服务名发现并调用你的实现。

## 服务在系统中的角色

标准服务的名称与 `srv_type` 由 robonix-core 的 Spec 定义，你的实现必须与规范一致。部署时在 `recipe` 中选择要注册的服务；执行 `rbnx deploy register` 时，会将服务信息（名称、`srv_type`、实际 ROS2 服务地址 `entry`、`provider`、`metadata` 等）写入 core。服务进程由你在包内实现（任意语言或框架），通过 ROS2 提供一个或多个 service server，对外暴露的服务名即为 manifest 里填写的 `entry`。`rbnx deploy start` 会根据 manifest 中的 `start_script` 启动该进程；其他节点（如 robonix-core、技能）通过 QueryService 按标准服务名查询，得到 `entry` 后即可调用你的 ROS2 服务。

## 接入步骤

1. 创建 robonix 包：与 [原语开发指南](hardware-primitives-guide.md) 类似，可以直接在 `~/.robonix/packages` 下建立包目录（包含 `rbnx_manifest.yaml`，可选 `rbnx/build.sh`），也可以在任意目录创建一个包含这些文件的工程并推送为 GitHub 仓库，之后通过 `rbnx package install --path` 或 `rbnx package install --github` 安装到本地。

2. 在 manifest 中声明服务：在 `rbnx_manifest.yaml` 的 `services` 列表中，为每个要提供的标准服务写一条配置：`name`（标准名）、`srv_type`（须与 Spec 一致）、`entry`（你的进程对外提供的 ROS2 服务名）、`metadata`、`version`，以及可选的 `start_script`、`stop_script`。

3. 实现服务进程：在代码中创建 ROS2 service server，类型与 Spec 中的 `srv_type` 一致，服务名与 manifest 中的 `entry` 一致；通过 `start_script` 启动该进程。

4. 注册与启动：在 `recipe` 中列出该包及要启用的服务名（如 `srv::semantic_map`），然后执行 `rbnx deploy register`、`rbnx deploy start`。core 与规划器通过 QueryService 按标准名查到 `entry` 并调用。

下面以 `srv::semantic_map` 为例给出 manifest 配置片段与字段说明。

## 在 rbnx_manifest.yaml 中声明服务

```yaml
package:
  name: demo_service_provider
  version: 0.0.1
  description: 提供 semantic_map 与 task_plan 的示例包
  maintainer: your_name
  maintainer_email: your_email@example.com
  license: MulanPSL-2.0

services:
  - name: srv::semantic_map
    srv_type: robonix_sdk/srv/service/semantic_map/QuerySemanticMap
    entry: /demo_service/semantic_map/query
    metadata: '{"model":"qwen3-vl-plus","backend":"vlm"}'
    version: 0.0.1
    start_script: rbnx/start_semantic_map.sh
    stop_script: rbnx/stop_semantic_map.sh
```

- `name`：必须与 robonix-core 中登记的标准服务名一致（如 `srv::semantic_map`、`srv::task_plan`），否则注册时校验会失败。
- `srv_type`：ROS2 服务类型，必须与 Spec 完全一致，见下文规范表。
- `entry`：你的进程实际对外提供的 ROS2 服务名；调用方通过 QueryService 得到该字符串后即可调用该服务。
- `metadata`：JSON 字符串，用于筛选实现（如模型名、后端类型）。
- `start_script` / `stop_script`：相对包根的脚本路径；`rbnx deploy start` 会在包根目录执行 `start_script` 以启动进程。

同一包可为同一标准服务提供多个实现（通过不同 `version` 或 `metadata` 区分），注册后都会写入 core，查询时可通过 `filter` 筛选。

## 服务进程如何暴露服务

在代码中使用 ROS2 创建 service server，将服务名设为 manifest 中的 `entry`（例如 `/demo_service/semantic_map/query`），类型设为 `srv_type`（例如 `robonix_sdk/srv/service/semantic_map/QuerySemanticMap`）。进程在 ROS2 网络中运行并创建该 server 即可，无需再向别处“注册”；向 robonix-core 的注册由 `rbnx deploy register` 完成（写入标准名到 `entry` 的映射），调用方通过 QueryService 取得 `entry` 后直接调用你的服务。若服务依赖某些原语（例如相机、位姿），可在进程内通过 QueryPrimitive 查询到对应 Topic 后再订阅或发布。可参考仓库中的 `robonix/rust/provider/demo_service` 实现。

## 服务规范表

下表为 robonix-core 中已定义的标准服务（名称及 srv 类型）。实现时须使用表中的 `name` 与 `srv_type`，`entry` 由你自行决定。

| 标准服务名 | 含义 | ROS2 服务类型（srv_type） |
|------------|------|---------------------------|
| `srv::semantic_map` | 语义地图：物体级环境表示（如 VLM+深度） | `robonix_sdk/srv/service/semantic_map/QuerySemanticMap` |
| `srv::task_plan` | 任务规划：自然语言转为 RTDL | `robonix_sdk/srv/service/task_plan/PlanTask` |
| `srv::spatial_map` | 空间地图：几何结构 | `robonix_sdk/srv/service/spatial_map/GetSpatialMap` |
| `srv::transform.scan` | 点云转 LaserScan（CONVERT/流式） | `robonix_sdk/srv/service/transform_scan/TransformScan` |
| `srv::plan_simulate` | 计划仿真与可行性检查 | `robonix_sdk/srv/service/plan_simulate/SimulatePlan` |
| `srv::result_feedback` | 执行结果反馈与验证 | `robonix_sdk/srv/service/result_feedback/ResultFeedback` |
| `srv::memory` | 认知记忆（TODO） | `robonix_sdk/srv/service/memory/QueryMemory` |
| `srv::control` | 躯体控制与安全（TODO） | `robonix_sdk/srv/service/control/Control` |

当前任务执行流程中，core 实际会调用的标准服务为 `srv::semantic_map`（构建物体图）和 `srv::task_plan`（规划）；其余服务已在 Spec 中预留，供后续扩展。

## Recipe 中启用服务

在 recipe 的 `packages` 中，为每个包指定要注册的 `services` 列表（填写标准服务名）：

```yaml
packages:
  - name: demo_service_provider
    services:
      - srv::semantic_map
      - srv::task_plan
```

若不写 `services` 字段，则会注册该包在 manifest 中列出的全部服务。

## LLM / API Key

若服务使用大模型 API（例如语义地图、任务规划），推荐在包根或进程工作目录下使用 `.env` 存放 API Key，并在代码中读取；不强制，具体方式由实现自行决定。常用变量名为 `DASHSCOPE_API_KEY` 或 `QWEN3_VL_API_KEY`。API Key 的申请与配置见 [快速开始 - LLM 与 API Key](../chapter1-getting-started/quickstart.md)。

参考实现：`robonix/rust/provider/demo_service` 提供 `srv::semantic_map`、`srv::task_plan`、`srv::transform.scan` 的完整示例，包括 start/stop 脚本与 manifest，可对照上述规范与 [robonix-sdk](robonix-sdk.md) 中的 srv 定义阅读。
