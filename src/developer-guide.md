# 开发者指南

[toc]

本指南面向 Primitive、Service 和 Skill Package 开发者。完成后，你将能从官方模板启动一个最小部署，创建自己的 Package，声明标准或自定义 contract，接收 deployment config，并用 Atlas 与 CLI 验证运行结果。

<div class="procedure-meta">
  <div><strong>源码</strong>syswonder/robonix dev-next</div>
  <div><strong>模板</strong>syswonder/template-rbnx main</div>
  <div><strong>交付物</strong>独立 Package 仓库或 robot deployment 子目录</div>
</div>

## 1. 选择 Package 类型

| 类型 | 何时使用 | 典型例子 |
|---|---|---|
| Primitive | 直接连接一项硬件或硬件数据源 | chassis、camera、lidar、IMU、arm、audio |
| Service | 提供可替换的系统算法或共享功能 | mapping、navigation、speech、memory |
| Skill | 提供模型可调用的语义行为 | explore、greet、pick、transport_things |

不要把整台机器人写成一个 Package。完整机器人由 robot deployment 组合多个 Package、URDF、Soma 和本体参数；Package 本身应保持可替换和可复用。

## 2. 跑通官方模板

先按[快速上手](getting-started/quickstart.md#2-取得-dev-next-并安装-robonix)安装 `dev-next`。然后克隆模板：

```bash
git clone https://github.com/syswonder/template-rbnx.git
cd template-rbnx
```

模板包含一个 Primitive、一个 Service 和一个 Skill。先验证三个 Package manifest；`rbnx validate` 使用位置参数，不使用 `-p`：

```bash
rbnx validate primitives/mock_chassis
rbnx validate services/my_navigate
rbnx validate skills/say_hello
```

每条命令都应以以下一行结束：

```text
✓ Manifest validation passed
```

配置 VLM 后构建、启动：

```bash
export VLM_API_KEY='sk-...'
export VLM_BASE_URL='https://api.example.com/v1'
export VLM_MODEL='your-model-name'

rbnx build
rbnx boot
```

另开终端：

```bash
rbnx caps -v
rbnx tools
rbnx ask 'Say hello to Alice.'
```

<div class="expected-result">
Atlas 中出现 <code>mock_chassis</code>、<code>my_navigate</code> 和 <code>say_hello</code>；<code>rbnx tools</code> 列出可供模型调用的 MCP 工具；问候任务完成后返回自然语言结果。
</div>

## 3. 理解 Package 与运行实例

一个 Package 是静态构建与分发单元。一个 deployment entry 是该 Package 的一次运行实例。

```yaml
primitive:
  - name: front_camera
    url: https://github.com/example/primitive-camera-rbnx
    branch: main
    config:
      serial: '123456'
```

- `url` / `path` 找到 Package。
- `name` 是本次运行的 provider id，必须与进程向 Atlas 注册的 `id` 一致。
- `config` 是该实例的运行时配置；只有实现生命周期 driver 的 Package 才会通过 `Driver(CMD_INIT, config_json)` 收到它。
- 同一 Package 可以启动多个实例，但每个实例必须使用不同的 `name`。

可复用 Package 不应把 provider id 写死。`rbnx boot` 为进程注入 `RBNX_INSTANCE_NAME`；代码使用它作为 `id`：

```python
import os
from robonix_api import Primitive

provider = Primitive(
    id=os.environ.get("RBNX_INSTANCE_NAME", "demo_camera"),
    namespace="robonix/primitive/camera",
)
```

`namespace` 是 contract 的主分组与诊断信息，不是 provider 唯一身份。共享 contract 或迁移中的不一致会产生提示，但 Atlas 以 `(provider_id, contract_id)` 区分能力。

## 4. 创建 Package 骨架

在一个 deployment 根目录中运行：

```bash
rbnx package-new demo_camera --type primitive
rbnx package-new demo_service --type service
rbnx package-new demo_skill --type skill
```

`--type` 默认为 `service`，创建 Primitive 或 Skill 时必须显式指定。以 Primitive 为例，生成结果是：

```text
primitives/demo_camera/
├── package_manifest.yaml
├── scripts/
│   ├── build.sh
│   └── start.sh
├── demo_camera/
│   ├── __init__.py
│   └── main.py
├── capabilities/
│   └── .gitkeep
└── .gitignore
```

生成器提供的是可编辑骨架，不会替你选择 contract、实现 transport 或连接硬件。

## 5. 编写 package_manifest.yaml

最小新格式：

```yaml
manifestVersion: 1

package:
  name: robonix.primitive.example.camera
  version: 0.1.0
  description: RGB-D camera primitive for Example Camera.
  license: Apache-2.0
  tags: [primitive, camera, rgbd]
  maintainers:
    - Your Name <you@example.com>

build: bash scripts/build.sh
start: bash scripts/start.sh

capabilities:
  - name: robonix/primitive/camera/driver
  - name: robonix/primitive/camera/rgb
  - name: robonix/primitive/camera/depth
  - name: robonix/primitive/camera/intrinsics

depends: []
```

运行时校验要求 `manifestVersion`、`package.name`、`package.version`、`package.description`、`package.license` 和非空 `start`。`build` 可省略，适用于已经交付可执行产物的 Package。准备提交 Catalog 时还应填写 `tags` 与 `maintainers`。

旧的 `package.vendor`、`package.id`、`nodes[]`、`build.script` 和 Package 内 `robonix_manifest.yaml` 继续向后兼容，但会显示迁移 warning；新 Package 使用上面的结构。

### Target manifest

同一 Package 可以为不同平台提供不同 manifest，例如：

```text
package_manifest.yaml
package_manifest.jetson-native.yaml
package_manifest.jetson-docker.yaml
```

Robot deployment 在对应条目上选择：

```yaml
- name: front_camera
  url: https://github.com/example/primitive-camera-rbnx
  branch: main
  manifest: package_manifest.jetson-native.yaml
  config: {}
```

Target manifest 决定该平台使用的 `build`、`start`、`stop`、依赖与能力列表；运行时 `config` 仍由 robot deployment 提供。多平台约定见[多平台与 Target Manifest](architecture/multiplatform-deployment.md)。

## 6. 选择 Contract

先查[接口目录](interface-catalog/index.md)和[自动生成的 Contract 参考](reference/contracts.md)。标准硬件与系统能力直接复用主仓库 contract，不在 Package 中复制一份：

- RGB-D camera：`robonix/primitive/camera/{driver,rgb,depth,intrinsics,extrinsics,snapshot}`
- 3D lidar：`robonix/primitive/lidar/{driver,lidar3d}`，载荷是 `sensor_msgs/PointCloud2`
- 2D lidar：`robonix/primitive/lidar/{driver,lidar}`，载荷是 `sensor_msgs/LaserScan`
- Chassis：`robonix/primitive/chassis/{driver,move,twist_in,odom}`
- Arm：以当前[自动生成参考](reference/contracts.md#primitive)中的 `robonix/primitive/arm/*` 为准

只有新语义能力没有标准 contract 时，才在 Package 的 `capabilities/` 中加入 TOML 与 ROS IDL，并在 manifest 的 capability 项上写 `path`。Skill 往往属于这种情况；`template-rbnx/skills/say_hello` 是完整示例。

构建脚本通过当前已登记的 Robonix 源码树解析标准 contract：

```bash
rbnx codegen -p "$RBNX_PACKAGE_ROOT"
```

默认产物位于：

```text
rbnx-build/codegen/
├── proto_gen/
├── robonix_mcp_types/
└── ros2_idl/          # 仅请求 ROS 2 codegen 时出现
```

## 7. 实现 Provider 与生命周期

Python Package 使用 `Primitive`、`Service` 或 `Skill`。共同流程是：

1. 以 deployment instance name 注册 provider。
2. 在 `on_init(cfg)` 中解析并验证配置、连接冷资源、声明 capability。
3. 在 `on_activate` 中启动需要持续运行的热资源。
4. 在 `on_deactivate` / `on_shutdown` 中停止线程、设备、子进程和 transport。
5. 调用 `provider.run()` 进入运行循环。

生命周期 driver 不是业务能力，但它决定 `rbnx boot` 能否下发 INIT / ACTIVATE / SHUTDOWN。Primitive 和 Service 在 boot 后进入 `ACTIVE`；Skill 完成 INIT 后保持 `INACTIVE`，第一次被 Executor 调用时激活。

业务能力的 transport 按数据形态选择：

| 数据形态 | 推荐 transport | 例子 |
|---|---|---|
| 高频连续传感器或控制 | ROS 2 topic | RGB、PointCloud2、Odometry、Twist |
| 系统间请求/响应或流 | gRPC | lifecycle、状态查询、流式健康信息 |
| 模型直接调用的工具 | MCP | navigate、goal_room、greet |

完整装饰器签名以 [`robonix_api`](https://github.com/syswonder/robonix/tree/dev-next/pylib/robonix-api) 源码和[代码 API](reference/api.md)为准。不要在教程里复制一份与 IDL 分叉的手写 JSON schema；MCP request / response 类型由 codegen 生成。

## 8. 文档化运行时配置

如果 Package 接受 `config`，在 `package_manifest.yaml` 旁增加 `config.spec`。它是给集成人员和工具阅读的说明，不参与运行时解析；实际默认值与校验仍由 Package 代码负责。

```yaml
config:
  device_serial:
    type: string
    required: true
    description: Camera serial number selected by this instance.

  publish_rate_hz:
    type: float
    unit: Hz
    default: 15.0
    constraints: 0 < value <= 30
    description: RGB and depth publish frequency.
```

每个字段至少写明类型、是否必需、单位、默认值、约束和用途。没有运行时配置时可以省略文件，或明确写 `config: {}`。

`CAPABILITY.md` 也是推荐项而非运行必需项。它解释模型应如何使用整个 Package、前置条件、长任务行为、安全限制和取消方式。Provider 注册时把正文上传到 Atlas，Pilot 需要时通过 `read_capability_doc(provider_id)` 获取；不要让模型读取 provider 本机文件路径。

## 9. 加入 robot deployment

本地 Package：

```yaml
primitive:
  - name: front_camera
    path: ./primitives/front_camera
    config:
      device_serial: '123456'
      publish_rate_hz: 15.0
```

独立仓库 Package：

```yaml
primitive:
  - name: front_camera
    url: https://github.com/example/primitive-camera-rbnx
    branch: main
    manifest: package_manifest.jetson-native.yaml
    config:
      device_serial: '123456'
```

远程仓库 clone 到 `<deployment>/rbnx-boot/cache/<repository-name>/`。Cache 目录按 URL 的仓库名命名，不按 provider instance `name` 命名；同一仓库的多个实例共享一个 checkout。

开发期间可使用移动分支；可复现实验或发布应记录并固定实际 commit。更新已有 cache 使用 `rbnx update`，不要手动修改一份未记录的 checkout 后仍把它当作上游版本。

## 10. 验证、构建与单包调试

在 Package 根或 deployment 根执行：

```bash
# 只检查 package manifest
rbnx validate ./primitives/front_camera

# 构建一个 Package
rbnx build -p ./primitives/front_camera

# 构建 deployment 中全部 Package
rbnx build -f ./robonix_manifest.yaml

# 已有 Atlas 时单独启动 Package
rbnx start -p ./primitives/front_camera \
  --endpoint 127.0.0.1:50051 \
  --config ./front_camera.local.yaml

# 启动完整 deployment
rbnx boot -f ./robonix_manifest.yaml
```

`rbnx validate` 只验证单个 `package_manifest.yaml`，不会验证 deployment 根；完整 deployment 由 `rbnx build -f` 和 `rbnx boot -f` 解析。

运行时检查：

```bash
rbnx caps -v
rbnx contracts
rbnx describe --provider front_camera
rbnx channels
rbnx inspect
rbnx logs -t front_camera
```

<div class="expected-result">
Provider id 与 deployment entry 的 <code>name</code> 相同；生命周期达到该类型的目标状态；每条实际暴露的 capability 都能在 <code>rbnx caps -v</code> 查到；日志文件名为 <code>rbnx-boot/logs/&lt;provider_id&gt;.log</code>。
</div>

## 11. 测试要求

### Primitive

- 配置缺失或单位错误时，INIT 返回可定位的错误。
- 硬件不存在、断开或恢复时，生命周期与日志符合预期。
- topic / RPC 的类型、frame、时间戳、单位和频率与 contract 一致。
- 运动硬件具有限速、watchdog、stop 和进程退出后的安全状态。

### Service

- 上游 provider 缺失时返回 `Deferred` 或结构化错误，不假装成功。
- 长任务具有稳定 run id、状态查询和取消语义。
- 参数来自 deployment，本体专属参数不硬编码在共享 Service 仓库。

### Skill

- 初始为 `INACTIVE`，首次调用激活，取消后资源可回收。
- `CAPABILITY.md` 写清前置条件、阶段、限制和失败恢复。
- 并发调用与重复调用不会产生不受控的物理动作。

真实硬件必须经过仿真、架空/台架、低速空场和目标场景四级验证；测试人员能触达 E-stop，并在启动车轮或机械臂前确认安全边界。

## 12. 发布与协作

提交社区前完成：

```text
[ ] package_manifest.yaml 可由 rbnx validate 通过
[ ] README 给出安装、构建、启动、验证和故障日志路径
[ ] LICENSE 与 package.license 一致
[ ] config.spec 覆盖所有 deployment-facing 字段
[ ] 标准 contract 没有被私自复制或改名
[ ] 目标平台的 build / boot 记录包含源码 commit 与 target manifest
[ ] 真实运动能力有安全验收记录
```

发布流程见 [Package Catalog](integration-guide/package-catalog.md)。仓库创建、Catalog 索引、整机集成验证和首页“已支持”是不同状态，不能用一次 manifest lint 代替本体验收。
