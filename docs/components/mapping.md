---
title: 建图与定位服务使用指南
---

# 建图与定位服务使用指南

本页面向要在一台机器人上建出第一张地图、并让后续任务在这张地图上稳定运行的工程师。阅读前需要机器人已经有可用的雷达或 RGB-D 相机和底盘里程计，并且能用 `rbnx boot` 启动部署。读完本页可以独立完成：选对传感器绑定与引擎、建图并保存、以定位模式启动、在运行中切换地图，以及判断一次“地图不见了”到底发生了什么。

接口清单与载荷定义在[空间地图接口页](../interface-catalog/service/map.md)，本页不重复。

本页依据上游 `service-map-rbnx` 提交 `898432ef` 编写。

## 服务负责什么

建图服务把传感器数据变成两样东西：一张二维占据栅格（occupancy grid），和一个可以反复回到的坐标系。前者交给导航服务规划路径，后者是[场景服务](./scene.md)里所有对象位姿的参照。

它同时负责地图的生命周期：保存、加载、删除，以及向全系统广播“当前是哪张地图、第几代”。场景服务据此判断自己的对象该不该失效。

## 三个数据库，必须分清

RTAB-Map 从不直接写入已保存的地图。下面所有规则都源于这一点。

| 数据库 | 路径 | 谁写它 |
|---|---|---|
| 已保存地图 | `{MAPPING_MAPS_DIR}/{map_id}/rtabmap.db` | 只有 `save_map` 写一次，此后不可变 |
| 运行时数据库 | `{MAPPING_RUNTIME_DB_DIR}/…`（默认 `/tmp/robonix-mapping-runtime`） | RTAB-Map 持续写。建图会话拿到一个全新的空库，加载则拿到已保存地图的副本 |
| 遗留默认库 | `~/.ros/rtabmap.db` | 正常部署里没人写，只作为 `save_map` 的最后兜底 |

一个已保存地图目录里除数据库外还有配套产物：

```text
maps/lab_3f/rtabmap.db  occupancy.pgm  occupancy.yaml  occupancy.png  cloud.pcd  meta.yaml
```

## 传感器绑定

`sensor_providers` 是唯一必填项。它把传感器角色映射到 Atlas 上的提供方 ID，写了哪个角色就启用哪个输入。

```yaml
service:
  mapping:
    config:
      sensor_providers:
        lidar3d: roof_lidar
        rgb: front_camera
        depth: front_camera
        odom: base_chassis
```

支持的角色是 `lidar2d`、`lidar3d`、`rgb`、`depth`、`imu`、`odom`。用 RGB-D 时 `rgb` 和 `depth` 必须同时给出。

## 启动配置

| 字段 | 默认值 | 含义 |
|---|---|---|
| `algo` | `rtabmap` | 建图引擎。**只用 `rtabmap`**，在跑的机器人部署全是它。`config.spec` 里另外列出的取值没有部署在用，不要选 |
| `occupancy_sources` | 所有可用输入 | 参与构建二维占据栅格的输入，取 `lidar`、`depth` |
| `params_file` | 无 | 部署自己拥有的 RTAB-Map 参数 YAML，相对路径从 `robonix_manifest.yaml` 所在目录解析。从上游 `config/rtabmap_params.template.yaml` 复制一份到部署仓库作为起点，上游模板运行时不会被加载 |
| `rtabmap_params` | 无 | 在 `params_file` 之上的最终覆盖，键用 RTAB-Map 的名字例如 `Grid/FootprintLength`，值必须是标量 |
| `base_frame` | `base_link` | 机器人本体坐标系，必须与完整 URDF/TF 树以及导航服务用的一致 |
| `odom_frame` | `odom` | 连续运动坐标系，选定的里程计提供方必须在这个坐标系里发布 |
| `deskew_lidar` | `false` | 对 `lidar3d` 点云做运动畸变校正，需要逐点时间戳 |
| `use_sim_time` | `false` | 用 ROS `/clock`。只有当所有传感器、TF 发布者和消费者都用同一个仿真时钟时才打开 |
| `map_mode` | `mapping` | 启动模式，见下一节 |
| `map_id` | 无 | 定位模式下要加载的地图标识，定位模式必填 |
| `webui_port` | `8091` | 建图 Web 界面端口，设为 `0` 或空字符串关闭 |
| `webui_host` | `127.0.0.1` | Web 界面绑定地址。**该界面没有鉴权**，除非有带认证的部署覆盖层保护，否则保持回环地址 |

### 分离定位与导航里程计

有些机器人用于建图的高精度里程计延迟太高，直接给导航用会导致控制震荡。这时打开 `navigation_odom_bridge`：RTAB-Map 内部里程计变成只发消息的私有轨迹，另有一座桥用底盘位姿发布 map 到导航 odom 的变换。

```yaml
config:
  navigation_odom_bridge: true
  navigation_odom_topic: /odom
  navigation_odom_frame: odom      # 打开桥时必须与私有的 odom_frame 不同
```

默认关闭，保持原有 TF 行为。

## RTAB-Map 参数系统

这是建图服务配置里真正花时间的地方。下面讲 RTAB-Map 自己的参数体系，以及现有机器人部署从中总结出来的经验。

### 两类参数，两套规则

RTAB-Map 的 ROS 包装层里有两类参数，行为不同，混淆它们会让节点起不来：

- **斜杠命名的 RTAB-Map 参数**，例如 `Grid/CellSize`、`Reg/Strategy`。包装层把它们统一声明为**字符串**。
- **原生 ROS 参数**，例如 `deskewing`、`publish_null_when_lost`。它们有各自的声明类型，必须保持布尔或数值。

服务已经按这个区分处理：`params_file` 与 `rtabmap_params` 里斜杠命名的键会被转成字符串，其余保持原类型。这条规则的存在是因为曾经全部字符串化，导致一个声明为布尔的原生参数被换成字符串，节点直接拒绝启动。

### 两层覆盖

`params_file` 是部署自有的完整参数文件，`rtabmap_params` 是在它之上的最终覆盖。后者适合放几条与机器人硬件强相关、且希望在清单里一眼看见的值：

```yaml
config:
  params_file: config/rtabmap_params.yaml
  rtabmap_params:
    Grid/FootprintLength: 0.84
    Grid/FootprintWidth: 0.60
```

### 实际需要调的几组

**`Grid/*` —— 占据栅格怎么来的。** 这一组直接决定导航看到的那张图。

| 参数 | 现有部署的取值 | 说明 |
|---|---|---|
| `Grid/CellSize` | `0.05` | 栅格分辨率，与 Nav2 代价地图的 `resolution` 保持一致 |
| `Grid/RangeMax` | `6.0` | 参与建栅格的最大距离。给得比雷达量程小是有意的，远处回波角分辨率差，写进栅格只会糊 |
| `Grid/RangeMin` | `0.25`（Lite3） | 近场截止。四足机器人靠它同时挡掉行走时的腿部回波 |
| `Grid/RayTracing` | `true` | 用射线清空走过的自由空间，否则地图上全是没被清掉的旧障碍 |
| `Grid/3D` | `false` | 二维导航用二维栅格 |
| `Grid/MaxObstacleHeight` | `1.0` 到 `1.5` | 高于此高度的回波不算障碍。按机器人真实高度定 |
| `Grid/MaxGroundHeight` | `0.1` | 低于此高度算地面 |
| `Grid/FootprintLength` / `Width` | 按 Soma 声明 | 写进栅格之前先去掉机身自身的回波。Lite3 用的是 soma.yaml 里考虑步态的包络 `0.68 × 0.46`，而不是官方静态尺寸 `0.61 × 0.37` |
| `Grid/Sensor` / `Grid/FromDepth` | Lite3 设 `0` / `false` | 只用雷达建栅格。RGB-D 仍供 RTAB-Map 和场景服务使用，但不会把同一堵墙用受外参误差影响的方式再写一遍 |

**`Reg/*` 与 `Optimizer/*` —— 位姿配准。** `Reg/Strategy: 1` 选 ICP，`Reg/Force3DoF: true` 把配准限制在平面内。后者在四足机器人上是必需的：不加的话步态带来的横滚和俯仰会把地图掰弯。

**`Icp/*` —— 匹配的松紧。** `Icp/MaxCorrespondenceDistance`、`Icp/MaxTranslation`、`Icp/MaxRotation` 三者共同决定一次匹配能接受多大的偏差。**调得太紧的表现不是报错，而是静默放弃修正**：Benben 的注释记录了这个现象——二维激光特征少，阈值过紧导致定位匹配不上，给了 initialpose 之后修正量恒为 0，旋转持续漂移。

**`Vis/*` —— 视觉特征门槛。** `Vis/MinInliers` 默认 20，两台用二维激光的机器人都放宽到 12，理由同样是特征少。

**`RGBD/*` —— 更新频率与回环。** `RGBD/LinearUpdate` 和 `RGBD/AngularUpdate` 决定移动多少才生成新节点（现有取值 `0.05` 到 `0.1`）。`RGBD/ProximityBySpace` 打开空间邻近检测，`RGBD/ProximityPathMaxNeighbors: 1` 把搜索限制在最近的路径邻居，Lite3 用它在保留走廊回环的同时减少平行墙造成的错误连接。

**`Rtabmap/DetectionRate`** 是每秒处理几帧。Lite3 取 `1.0` 并写明了依据：手动建图推荐速度 0.10 到 0.15 m/s，1 Hz 意味着每次更新间隔 10 到 15 厘米，同时给 Jetson 留下足够算力做 RGB-D 与点云同步和 ICP 里程计。**这是定这个参数的正确方式——从建图时的移动速度倒推。**

### 三条踩过的坑

这三条都写在真实部署的注释里，不是推测。

**`RGBD/MaxOdomCacheSize: 0`。** 默认值大于 0 时，定位的回环修正要等第二次确认匹配才生效。机器人静止时这次确认永远不会来，于是 map 到 odom 的修正一直挂起，`/map` 显示的是一个局部窗口而不是加载进来的整张图。

**`Mem/InitWMWithAllNodes: true`。** 不设它的话，运行时加载地图后工作内存不会载入全部已保存节点，`/map` 只从实时传感器数据重建，表现为大约 6 米的局部窗口。对全新的临时建图库没有影响。

**`RGBD/ProximityGlobalScanMap` 必须保持 `false`。** 设为 `true` 会触发 RTAB-Map 0.22 的断言崩溃（`Rtabmap.cpp:2953`，`Pose of N not found in global scan poses`），定位和建图的邻近检测直接中止。

### 一个不要覆盖的参数

`Mem/IncrementalMemory` 由服务按模式接管：建图时为真，定位时为假。**不要在共享参数文件里覆盖它**，否则模式切换的语义会被破坏。

## 两种启动模式

`map_mode` 只有两种有意义的组合。

建新图是默认形态，两个键都不写就是它：

```yaml
service:
  mapping:
    config: {}
```

以定位模式起来，是跑任务时用的稳定坐标系形态：

```yaml
service:
  mapping:
    config:
      map_mode: localization
      map_id: lab_3f
```

`mapping` 模式**总是打开一个全新的空运行时库**。同时写了 `map_id` 也会被忽略：它指向的是一个已保存产物，不是一个活着的会话。**没有“启动后继续扩展地图 X”这种配置。**

`localization` 模式必须给 `map_id`，地图不存在就启动失败——这是有意的，好过悄悄从开机位姿开始重新建图。它把已保存的数据库复制一份并在副本上定位，因此**地图坐标系跨重启稳定**，场景服务才能为同一个 ID 恢复语义状态。

## 运行时操作

| 操作 | 改数据库 | 改模式 | 改坐标系 |
|---|---|---|---|
| `save_map(map_id)` | 否，把当前活动库快照成一张新的已保存地图 | 否 | 否 |
| `load_map(map_id)` | 是，复制已保存地图并切到副本上 | 是，切到定位 | 是，切到该地图的坐标系 |
| `switch_mode(mode)` | 否 | 是 | 否 |
| `reset_map` | 否，只清工作内存，文件还在 | 回到建图 | **是**，原点变成机器人当前位姿 |
| `pose_estimate(x, y, θ)` | 否 | 否 | 否 |
| `delete_map(map_id)` | 从磁盘删除一张已保存地图 | 否 | 否 |

现场需要记住的四条：

- **`save_map` 只发布一次。** 用已存在的 `map_id` 保存会被拒绝，修正后的地图存成新 ID。
- **`load_map` 一定进入定位模式。** 传 `mapping` 参数会被接受并强制改写，因为 RTAB-Map 只有在以定位模式打开数据库时才会恢复已保存的占据栅格。
- **`reset_map` 会让坐标失效。** 重建出来的地图不和旧的共享坐标系，之前记录的位置全部作废。生命周期广播会提升 generation 来说明这一点。
- **加载会替换当前会话。** 上次保存之后建的部分全部丢失，先保存再加载。

## 常用流程

1. **建第一张图** —— 不写 `map_id` 和 `map_mode` 启动，把空间走一遍，`save_map("lab_3f")`。
2. **再建一张图** —— 重启服务，然后走图并存成新 ID。**不要**先加载一张已有地图。
3. **在已保存地图上跑任务** —— 用 `map_mode: localization` 加 `map_id` 启动，或者对运行中的服务调 `load_map(id)`。
4. **修正一张已保存地图** —— 重新建一个会话并存成新 ID。已发布的地图是不可变的。

## 运行中切换模式，以及“地图不见了”

配置里的 `map_mode` 只是启动默认值。`switch_mode` 能在不动数据库和坐标系的前提下翻转运行中的 RTAB-Map。

**优先重启，而不是运行时切换。** 从定位切回建图是危险的那个方向，而且切过去之后加载的那张地图通常会从实时视图里消失。Web 界面在定位状态下会一直显示警告，并在这次切换前要求确认。要建新图，请用 `map_mode: mapping` 重启服务。

地图消失时没有任何东西被删除。它变成了一个图的连通分量，而当前发布的地图不是从这个分量组装的。RTAB-Map 0.23.x 里分四步发生：

1. 进入定位会调用 `Memory::incrementMapId()`，开启新的会话 ID 并清空短期记忆。每次加载都会这样，因为加载总是进入定位。
2. 定位期间每个新节点都被丢弃而不是保留，会话 ID 保持不变。
3. 切回建图只是把 `Mem/IncrementalMemory` 翻成真。`Memory::addSignatureToStm` **只在会话 ID 相同时**才把新节点连到上一个节点，所以切换后建的第一个节点没有回到旧地图的里程计连接。图从此有两个互不相连的分量。
4. 发布的地图来自 `Rtabmap::optimizeCurrentMap`，它优化的是当前节点所在的连通分量。旧地图在另一个分量里，因此不在 `/map` 中。

当 RTAB-Map 在两个会话之间检测到一次回环时地图会回来：那条连接把两个分量合并。所以这次切换只在重定位确实能成功的场合才安全。磁盘上的数据库两种情况下都不受影响。

## 定位常见问题

**定位模式启动失败。** 这是设计行为：`map_id` 指向的地图不存在时服务拒绝启动，而不是从开机位姿开始建图。先确认地图目录存在且包含 `rtabmap.db`。

**加载后机器人在地图上的位置不对。** 用 `pose_estimate` 给一个初值。它不改数据库也不改模式。

**建的图漂移或者尺度不对。** 先确认 `base_frame` 和 `odom_frame` 与 URDF/TF 树一致，再确认里程计提供方确实在声明的坐标系里发布。这两项不一致时地图看起来仍然像地图，但所有坐标都是错的。

**Web 界面打不开。** 默认只绑回环地址。要从别的机器访问必须先有带认证的覆盖层，不要直接把 `webui_host` 改成 `0.0.0.0`。
