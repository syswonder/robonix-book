---
title: 场景服务使用指南
---

# 场景服务使用指南

本页面向把场景服务（Scene）部署到一台真实机器人或仿真环境上的工程师。阅读前需要已经能用 `rbnx boot` 启动一个最小部署，并且这台机器人已经有可用的定位与占据栅格（occupancy grid）。读完本页可以独立完成三件事：判断当前硬件能跑到哪一档语义能力、写出一份可用的场景服务配置、在对象没有出现或者出现重复时定位到具体原因。

接口清单、能力约定（Contract）ID 与载荷定义在[场景理解接口页](../interface-catalog/system/scene.md)，本页不重复。

## 场景服务负责什么

场景服务订阅机器人的彩色图、深度图、位姿和占据栅格，维护一份对象级的环境模型，并通过能力约定回答关于它的查询。规划器问“椅子在哪里”，由场景服务给出地图坐标系下的位姿。

三类数据只存在于场景服务里：

- 对象注册表（object registry），它是“环境里有哪些对象”的唯一权威答案。
- 关系层（relation layer），描述这些对象彼此之间的空间与语义关系。
- 用户标注（user annotation），也就是人在地图上画出的房间和兴趣点。没有任何感知算法会产生这类信息。

场景服务不做同步定位与建图（SLAM）。度量定位和占据栅格来自[空间地图服务](../interface-catalog/service/map.md)，场景服务只消费。它也不做路径规划：`goal_near` 和 `goal_room` 把语义引用换算成规划器可以接受的目标位姿，到此为止。

## 第一次把场景服务跑起来

场景服务是部署的一部分，由 `rbnx boot` 随整栈拉起，不单独启动。以仓库自带的 Webots 演示为例：

```bash
# 构建镜像。这一步会拉 torch 与感知模型权重，第一次比较久
cd system/scene && bash scripts/build.sh

# 起仿真（Webots + 底盘 + 相机 + 雷达）
cd ../../examples/webots && bash sim/start.sh

# 起整栈：atlas、executor、pilot、三个原语、场景、建图、导航、探索
export DISPLAY=:0
rbnx boot
```

启动之后按这个顺序确认它真的活着：

```bash
# 1. 场景服务在能力注册表里，且是 ACTIVE
rbnx caps | grep scene

# 2. 操作界面能打开
curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:50107/

# 3. 智能体能看到它的工具
rbnx tools | grep scene
```

日志在部署目录的 `rbnx-boot/logs/scene.log`。启动时有两行必看：`perception plan:` 说明定档结果与接上的输入，`ConceptGraphsDetector started` 说明检测器就绪。两行之间的时间差就是激活耗时，x86 上实测约 12.6 秒。

对象要靠机器人走动才会出现。让规划器跑探索技能：

```bash
rbnx ask "thoroughly explore the entire room, and wait for explore to finish"
```

## 硬件决定能跑到哪一档

场景服务把硬件分成三档，只看相机类输入是否接上，不看性能。

| 档位 | 输入 | 检测器 | 智能体可以问什么 |
|---|---|---|---|
| `metric` | 彩色图 + 深度图 | ConceptGraphs | 对象级三维语义、空间关系、开放词表查询 |
| `visual` | 只有彩色图 | 视觉语言模型（VLM） | 区域级语义，对象位置粗略 |
| `geometric` | 都没有 | 无 | 只有占据栅格与 `goal_near` |

内参（intrinsics）、位姿和外参（extrinsics）不参与档位判定。它们缺失时降低的是接地质量而不是档位：没有内参时，度量档会等待而不是猜一个相机矩阵。启动日志里有一行 `tier=... detector=... grounding=... inputs=[...]`，它是判断当前档位的唯一依据。

## 一次感知循环做了什么

度量档默认每 0.6 秒执行一次，每次都完整跑完下列步骤，没有关键帧门限：

1. YOLO-World 在配置的开放词表上给出候选框。
2. 先剔除忽略类，再做分割，因为分割是最贵的一步。
3. MobileSAM 为每个保留的框生成掩码。
4. CLIP 把每个掩码裁剪编码成特征向量。
5. 掩码结合深度图反投影成地图坐标系下的点云和有向包围盒，所用的相机到地图变换优先取自 TF2，其次由位姿加校验过的外参组合。
6. 背景类（地板、墙、天花板、地毯）和只落在地面上的检测被过滤掉。
7. 剩下的检测与持久对象地图做关联并融合。
8. 周期性清理：合并重叠对象，删除“旧位置已经被明确观测为空”的对象。
9. 把该对象地图的当前状态投射回对象注册表。

## 对象如何建立、合并与消失

这一节决定了实际效果，调参也集中在这里。

**关联**按类别与坐标系分桶，在类内做匈牙利最小代价匹配，代价是三维欧氏距离加上置信度惩罚，超出逐类门限半径的配对直接判为不可匹配。匹配上的对象用指数滑动平均更新位姿，未匹配的检测建立新对象。

这里有一个必须知道的后果：**关联是类别严格的**。同一把椅子这一帧被认成 `chair`、下一帧被认成 `couch`，就会产生两个对象。这是重复对象的主要来源，也是 `SCENE_CG_MERGE_CLASS_GROUPS` 存在的原因。

**合并**有两条通路。逐帧合并处理同一物体被拆成多条记录的情况；周期性合并有三个独立判据，可以分别开关：类无关的几何塌缩（只看包围盒交并比与包含关系）、点云重叠加视觉相似度、同类近邻距离。三者的阈值都是环境变量，见下面的配置表。

**消失**分两级。软驱逐把记录标记为 `missing` 并释放它与后端的绑定，`SCENE_OBJECT_TTL_SEC` 之内重新看到可以复用原来的对象 ID 和观测计数；超过之后硬删除。另有一条独立通路：当一个对象的旧位置当前就在视野里而且是空的，它会被直接移除。

## 关系分两层

这两层是分开的，部署时可以只用第一层。

几何关系循环以 3 Hz 运行，只依据几何产生接触、包含与可达性边。它不需要模型、不需要网络、不需要任何凭据，规划器依赖它在启动后数秒内可用。

场景图构建器（scene graph builder）额外产生图像接地的语义边，需要视觉语言模型的接口地址与密钥。没有密钥时它会记录一行日志并让关系返回 `unknown`，几何层照常工作。一个新部署不需要这一层就能跑通。

## 部署配置

场景服务的配置分两处：部署清单（manifest）里的 `config` 块，和环境变量。清单字段优先于同名环境变量。

### 清单字段

```yaml
system:
  scene:
    manifest: package_manifest.yaml
    config:
      map_id: office
      observations:
        - kind: rgb
          topic: /camera/color/image_raw
          type: sensor_msgs/msg/Image
        - kind: depth
          topic: /camera/depth/image_rect_raw
          type: sensor_msgs/msg/Image
      camera_provider_id: com.robonix.example.tiago_camera
      camera_frame: camera_color_optical_frame
      base_frame: base_link
      pose_max_age_s: 2.0
      web_port: 50107
```

| 字段 | 含义 |
|---|---|
| `observations` | 逻辑输入种类到 ROS 2 话题与消息类型的映射。`kind` 取 `rgb`、`depth`、`lidar2d`、`pose`、`odom` 等 |
| `camera_provider_id` | 彩色图、深度图、内参、外参必须来自同一台物理相机，这个字段把它们钉在同一个提供方上 |
| `camera_frame` | 相机光学坐标系名。不配置时按 TF 树解析，解析不出来就停止输出而不是猜 |
| `base_frame` | 机身坐标系名，与本体模型（Soma）声明的值核对 |
| `pose_max_age_s` | 用于相机到世界投影的位姿最大接收年龄，超龄样本会让检测被扣住不发布 |
| `map_id` | 地图身份的静态回退值。建图服务广播 `robonix/service/map/lifecycle` 时以广播为准 |
| `intrinsics_fallback` | 仿真部署显式选用的内参回退，需要经过评审。不配置时缺内参就等待 |
| `web_port` | 操作界面端口，设为 `0` 关闭界面 |

### 环境变量

按用途分组，只列部署时真正会调的。完整清单在 `system/scene/README.md`。

**感知模型与词表**

| 变量 | 默认值 | 含义 |
|---|---|---|
| `SCENE_OPEN_VOCAB_CLASSES` | 55 项默认表 | 逗号分隔的开放词表。检测器只能用给定的名字作答，把词表换成部署现场真实存在的类别，是目前实测收益最大的一处改动 |
| `SCENE_DETECT_PERIOD_S` | `0.6` | 检测周期。与语音等其他 GPU 任务共用一张卡时调大它 |
| `SCENE_CG_FORCE_CPU` | 空 | 置 `1` 强制 CPU，约慢三倍 |
| `SCENE_CLIP_MODEL` / `SCENE_CLIP_PRETRAINED` | `ViT-B-32` / 本地权重 | 特征编码模型 |
| `SCENE_PERCEPTION_WAIT_S` | `30` | 等待相机提供方的秒数，超时后按当时可见的输入定档 |

**合并与去重**

| 变量 | 默认值 | 含义 |
|---|---|---|
| `SCENE_CG_SAME_CLASS_MERGE_DIST_M` | `0.4` | 同类近邻合并距离，不看视觉相似度。对付“一个键盘变三个”。设 `0` 关闭 |
| `SCENE_CG_MERGE_CLASS_GROUPS` | 空 | 易混类归组，例如 `chair,table,desk;sofa,couch`。同组内的标签抖动会被折叠，组外互不影响。空值表示永不跨类改名 |
| `SCENE_CG_CROSS_CLASS_IOU_THRESH` | `0.30` | 周期性类无关塌缩的包围盒交并比阈值，调低更激进 |
| `SCENE_CG_CROSS_CLASS_OVERLAP_THRESH` | `0.50` | 同上，一个框被另一个包含的比例阈值 |
| `SCENE_CG_MERGE_OVERLAP_THRESH` / `SCENE_CG_MERGE_VISUAL_SIM_THRESH` | `0.50` / `0.65` | 点云重叠且 CLIP 余弦相似度同时达标才合并 |
| `SCENE_CG_OBJ_MIN_POINTS` | `20` | 周期清理的最小点数。键盘这类薄物体反投影后点很稀，调高会把它们一起删掉 |
| `SCENE_OBJECT_TTL_SEC` | `30` | 软驱逐对象的保留时长，决定对象 ID 在短暂遮挡后能否复用 |

**关系层**

| 变量 | 默认值 | 含义 |
|---|---|---|
| `SCENE_GRAPH_IMAGE_RELATIONS` | `true` | 用一次图像接地的模型调用产生关系边。置 `false` 或拿不到相机帧时回退到逐对文本推理 |
| `SCENE_GRAPH_IMAGE_MAX_DIM` | `960` | 送给模型的标注图长边像素上限，直接决定图像 token 开销 |
| `VLM_REASONING_EFFORT` | 未设置 | 设置后转发给所有场景模型调用。未设置时字段整个不出现，不影响非推理模型 |

**界面、持久化与地图绑定**

| 变量 | 默认值 | 含义 |
|---|---|---|
| `SCENE_WEB_HOST` | `0.0.0.0` | 操作界面绑定地址。界面没有鉴权，机器人在共享网络上时设为 `127.0.0.1` |
| `SCENE_WEB_PORT` | `50107` | 操作界面端口 |
| `SCENE_OBJECT_MEMORY_DB` | `/data/robonix/scene_memory/objects.db` | 对象快照数据库路径，容器内路径由宿主挂载 |
| `SCENE_ANNOTATIONS_DIR` | `/data/robonix/scene_annotations` | 用户标注按地图分文件存放的目录 |
| `SCENE_RESTORE_ON_START` | 未设置 | 显式选择启动时热恢复对象。默认每次启动都是新的实时会话 |
| `SCENE_MAP_ID` | `default` | 地图身份回退值，优先级低于清单 `map_id`，两者都低于建图服务的广播 |
| `SCENE_MAP_BINDING_WAIT_S` | `3.0` | 启动探测等待生命周期约定出现的秒数，`0` 关闭探测 |

## 操作界面怎么用

界面在 `SCENE_WEB_PORT`（默认 50107），四个页面各自回答一个问题。它们都不需要智能体在运行，是排查感知问题最快的入口。

| 页面 | 路径 | 回答什么 |
|---|---|---|
| 二维地图 | `/2d` | 对象和机器人画在占据栅格上，判断对象有没有落在它该在的房间里 |
| 三维视图 | `/3d` | 对象点云与包围盒，判断几何是否合理 |
| 相机 | `/cam` | 感知流水线**实际收到**的彩色图与深度图，带编码与时间戳 |
| 标注与地图 | `/user` | 画房间、保存与加载地图、纠正定位 |

新部署一个对象都没有时，先开相机页。它把“相机没接上”和“检测器什么都没找到”分开，这两者在日志里长得很像。

### 保存一张地图

保存的是两样东西：建图服务的空间制品（spatial artifact），以及场景服务这一侧的房间标注与对象快照。

1. 打开 `/user`。顶部状态区会显示当前地图身份；没保存过的实时会话显示 `unsaved live`。
2. 在 **Map ID** 里填一个名字，例如 `office_3f`。这个名字之后就是地图身份，`goal_room` 之类的查询都以它为界。
3. 点 **Save current**。成功后状态行会给出写入结果与房间数量。

对同一个 Map ID 再次点 Save current 不会覆盖空间制品：界面会提示该地图已存在，本次只更新该 ID 下的房间与对象。要重建空间制品，只能先删除这张地图再重新保存。这条规则是有意的，避免机器人在错误定位下把一张好地图写坏。

### 加载一张地图并纠正定位

1. 在 `/user` 的地图列表里点目标地图的 **Load**。
2. 界面会依次执行四步并显示进度：校验空间制品、把建图服务切到定位模式、等待一张新的占据栅格、恢复房间与对象。
3. 加载完成后如果机器人在地图上的位置明显不对，点 **Pose estimate**，按钮会变成 `Click pose on map`，再在地图上点机器人实际所在的位置。

加载之后房间与对象都归属于同一个 Map ID，不需要额外操作。

### 手工标注房间

房间只能人来画，没有任何感知算法会产生它。

1. 在 `/user` 点 **✏ Annotate room**，按钮变成 `✕ Cancel drawing`。
2. 在地图上依次点击房间的各个角点，至少三个。
3. **双击**或按 **Enter** 结束，按 **Esc** 取消。
4. 在右侧列表里给它命名。

地图重建（`generation` 提升）之后房间不会被删除，而是标为过期并显示成黄色，同时出现提示横幅。逐个确认仍然有效点 **Still valid**，位置变了就重画。房间是用户资产，系统只标记、不替你删除。

完整的界面语义规则，包括空间制品不可变与过期标记的处理，在[场景理解接口页](../interface-catalog/system/scene.md)。

## 硬件要求

**尚未确定。** 场景服务目前只在一台开发机上有完整实测数据，还没有在多种平台上系统测过，因此本页不给出最低配置。要在 Debian 与 syswonder 小车上补测之后再填。

现有的唯一一组实测数据，仅供参考它的量级，不要当作要求：

| 项目 | 实测值 | 测量条件 |
|---|---|---|
| 显存 | 3.9 GB | RTX 2060 6 GB，`lite` 档（YOLO-World + MobileSAM + CLIP ViT-B-32） |
| 显卡占用 | 约 37% | 同上，仿真办公室场景巡场 |
| CPU | 约一个核 | 每 0.6 秒一拍，每拍都跑完检测、分割、编码 |
| 容器常驻内存 | 启动 1.2 GB，五分钟后 2.8 GB | 同上 |

需要特别注意一处**尚未定位的内存异常**：同一条巡场路线上，有一次容器常驻内存在一个采样周期（5 秒）内从 2.8 GB 冲到 22.0 GB，维持约两分钟后随容器重启回落。它是突发而不是缓慢增长，原因还没定位，周期性合并（会把两团点云拼接后重新处理）是目前的首要怀疑对象。在共享内存的嵌入式板子上这不是一次异常而是一次终止，所以移植前应当先复现并修掉它。

## 三个部署目标

场景服务是唯一与平台相关的 Robonix 系统组件。Rust 系统二进制是一份静态构建，场景服务带着一整套 CUDA 感知栈。

| 目标 | 清单 | torch 来源 | 运行方式 |
|---|---|---|---|
| x86 容器（默认） | `package_manifest.yaml` | 镜像内置 cu128 wheel | `docker run --gpus all` |
| Jetson 容器 | `package_manifest.jetson-docker.yaml` | jetson-ai-lab wheel | `docker run --runtime nvidia` |
| Jetson 原生 | `package_manifest.jetson-native.yaml` | 宿主 JetPack torch | 宿主 `python3 -m scene_service.service` |

多数 Jetson 上推荐原生路径，它复用 JetPack 的 CUDA 栈，不必构建数 GB 的 L4T 镜像。宿主依赖的安装命令在 `system/scene/README.md`。

## 定位常见问题

**启动后界面打不开，进程却还活着。** 生命周期激活（`CMD_ACTIVATE`）有 90 秒上限。x86 上从感知计划打印到检测器就绪实测 12.6 秒，其中 YOLO-World 约 5 秒、CLIP 约 1 秒，余量很大。超时几乎都是被某个外部资源阻塞，而不是模型加载慢：用日志里 `perception plan:` 到 `ConceptGraphsDetector started` 两行的时间差判断。上一次部署没有完全退出、端口仍被占用，是最常见的一种。调大 `ROBONIX_DRIVER_INIT_TIMEOUT_S` 只会把问题推后，不要用它绕过。

**一个对象都没有。** 按顺序看三处：启动日志的 `tier=` 行是不是 `geometric`；相机页有没有图；日志里有没有周期性的检测计数行。度量档缺内参时会一直等待，这是设计行为，日志里会说明。

**对象重复。** 先看重复的两条记录类别是否不同。不同就属于标签抖动，用 `SCENE_CG_MERGE_CLASS_GROUPS` 把这两个类归组；相同就调 `SCENE_CG_SAME_CLASS_MERGE_DIST_M`。两者都不要调过头，合并过激会把两个相邻的真实物体折叠成一个。

**对象位置整体偏移。** 检查相机到地图的变换来源。TF 树完整时走 TF；只有位姿加外参时，任何一处标定误差都会整体搬运所有对象。启动日志的 `grounding=` 字段是 `degraded` 就说明走的是回退路径。

**建图回环后对象留在原地。** 这是已知缺陷，当前没有实现修复。重建地图后用标注页确认或重画房间，对象会随重新观测重建。
