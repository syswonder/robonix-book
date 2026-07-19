---
title: 建图参数与里程计接入
---

# 建图参数与里程计接入

本页说明新机器人如何把底盘里程计、激光雷达和 RGB-D 相机接入 Robonix Mapping，并把 RTAB-Map 参数保存在机器人部署仓库。完成后，系统只有一条连通的 `map → odom → base_link → sensor frames` 变换链。

## 里程计数据由谁提供

移动底盘原语是外部里程计的唯一提供方。它通常从轮速、轮子编码器或底盘 SDK 反馈积分出位置和速度，然后同时发布：

- `robonix/primitive/chassis/odom` 能力，载荷为 `nav_msgs/Odometry`；
- 同一份位姿对应的 TF `odom → base_link`。

`Odometry.header.frame_id` 应为 `odom`，`child_frame_id` 应为 `base_link`。`pose` 表示 `base_link` 在 `odom` 坐标系中的位姿，`twist` 表示在 `base_link` 坐标系中表达的底盘速度。Odometry 消息和 TF 必须来自同一个状态估计，使用可比较的时间戳；不要让驱动、仿真器和 Mapping 各自发布一份 `odom → base_link`。

当部署绑定了外部里程计时，当前 RTAB-Map 路径设置 `odom_frame_id=odom` 和 `odom_sensor_sync=false`。RTAB-Map 不再启动第二个里程计节点，也不依赖把 `/odom` 消息与每一帧传感器数据做额外同步；它在每个传感器时间戳通过 TF2 查询 `odom → base_link`。`nav_msgs/Odometry` 仍然是 Navigation、Soma 和其他状态消费者的标准能力。

Mapping 根据回环与图优化结果发布 `map → odom`。机器人在地图中的位姿由两段变换组成：

```text
map → odom        Mapping / RTAB-Map 发布，表示全局优化修正
odom → base_link  底盘原语发布，表示局部连续运动
```

不要由底盘发布 `map → odom`，也不要让 Mapping 重发外部底盘里程计。没有绑定外部里程计时，Mapping 才会根据已绑定的激光雷达或 RGB-D 输入启动 RTAB-Map 里程计，并声明 `robonix/service/map/odom`。

## 在部署清单中绑定输入

下面的值是 Atlas 提供方 ID，不是 ROS 2 话题名。Mapping 通过标准能力查找每个提供方的实际话题和坐标系。

```yaml title="robonix_manifest.yaml"
service:
  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    branch: main
    config:
      use_sim_time: false
      occupancy_sources: [lidar, depth]
      params_file: config/rtabmap_params.yaml
      sensor_providers:
        lidar2d: front_lidar
        rgb: front_camera
        depth: front_camera
        odom: base_chassis
```

使用三维雷达时将 `lidar2d` 改为 `lidar3d`。RGB-D 必须同时绑定 `rgb` 和 `depth`；只配其中一个不会形成 RGB-D 输入。`occupancy_sources` 决定二维占据格栅使用激光、深度或两者；上例中的 RGB-D 深度可以补充二维雷达高度之外的桌椅等障碍。

仿真才把 `use_sim_time` 设为 `true`，且每个传感器、TF 发布方和消费者都必须使用同一 `/clock`。真机使用墙上时钟时保持 `false`。

## 创建机器人自己的 RTAB-Map 参数文件

Mapping 上游仓库只提供模板，运行时不会自动加载它。在机器人部署仓库根目录执行：

```bash
mkdir -p config
curl --fail --location \
  --output config/rtabmap_params.yaml \
  https://raw.githubusercontent.com/syswonder/service-map-rbnx/main/config/rtabmap_params.template.yaml
```

`params_file` 的相对路径以 `robonix_manifest.yaml` 所在目录为基准。完整参数基线保存在这份文件中；只有少量部署环境差异才使用清单的 `rtabmap_params` 做最终覆盖。

### 模板参数

下表是当前 Mapping 模板的初始值，不是所有机器人的推荐终值。参数的完整定义以 RTAB-Map 上游 [`Parameters.h`](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h) 为准。

| 参数 | 模板值 | 含义与调整方向 |
|---|---:|---|
| `RGBD/CreateOccupancyGrid` | `true` | 为每个图节点创建局部占据格栅 |
| `Grid/RangeMax` | `6.0` | 建格时使用的最大传感器距离，单位米；`0` 表示不限制 |
| `Grid/CellSize` | `0.05` | 占据格栅分辨率，单位米；值越小，地图越细且内存与计算开销越高 |
| `Grid/RayTracing` | `true` | 从传感器到障碍格做射线更新并填充自由空间；关闭后更容易保留过期障碍点 |
| `Grid/3D` | `false` | 只生成二维投影；三维 OctoMap 需要更多内存和时间 |
| `Grid/NormalsSegmentation` | `false` | 不通过点云法向分离地面，改用高度通过滤波 |
| `Grid/MaxObstacleHeight` | `1.0` | 障碍点的最大高度；`0` 表示不限制 |
| `Grid/MaxGroundHeight` | `0.1` | 关闭法向分割时，低于该高度的点按地面处理 |
| `Reg/Strategy` | `1` | 配准策略：`0` 视觉、`1` ICP、`2` 视觉加 ICP |
| `Reg/Force3DoF` | `true` | 将位姿限定为平面 `x`、`y`、yaw，轮式室内机器人通常保持开启 |
| `Optimizer/Strategy` | `1` | 图优化器：`0` TORO、`1` g2o、`2` GTSAM、`3` Ceres；实际可用项取决于 RTAB-Map 构建 |
| `RGBD/NeighborLinkRefining` | `true` | 新节点入图时，用选定配准方法精化与前一节点的相对变换 |
| `RGBD/ProximityBySpace` | `true` | 在工作记忆中对空间上邻近的位置进行闭环候选检测 |
| `RGBD/AngularUpdate` | `0.1` | 新地图节点所需的最小角位移，单位弧度 |
| `RGBD/LinearUpdate` | `0.1` | 新地图节点所需的最小直线位移，单位米 |
| `Vis/MinInliers` | `12` | 计算或接受视觉变换所需的最小内点数 |
| `Rtabmap/DetectionRate` | `1.0` | RTAB-Map 处理输入的目标频率，单位 Hz；提高前先确认 CPU/GPU 和 TF 缓冲能跟上 |
| `Icp/MaxCorrespondenceDistance` | `0.2` | ICP 点对对应允许的最大距离，单位米 |
| `Icp/MaxTranslation` | `0.5` | 单次 ICP 修正可接受的最大平移，单位米 |
| `Icp/MaxRotation` | `0.78` | 单次 ICP 修正可接受的最大旋转，单位弧度 |

Webots Tiago 示例的部署文件将 `RGBD/LinearUpdate` 和 `RGBD/AngularUpdate` 设为 `0.05`，将 `Rtabmap/DetectionRate` 设为 `5.0`，并增加 `Mem/NotLinkedNodesKept: false`。这些是针对该仿真传感器频率和算力的部署值，不是新机器人的通用默认值。可执行示例见 [`examples/webots/config/rtabmap_params.yaml`](https://github.com/syswonder/robonix/blob/dev/examples/webots/config/rtabmap_params.yaml)。

## 验证顺序

### 1. 先验证里程计和 TF

启动底盘原语后，先不启动自主探索。确认 Atlas 能找到里程计能力，再检查 ROS 2 数据。以下示例使用 `/odom`；如果 Atlas 显示的实际话题不同，将命令中的话题替换为该值。

```bash
rbnx caps -v | rg -A 12 -B 2 'base_chassis|robonix/primitive/chassis/odom'
ros2 topic info /odom --verbose
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

机器人静止时位姿不应连续跳变；缓慢直行时平移方向应与实际运动一致；原地旋转时 yaw 方向应正确，位置不应产生大幅圆周运动。任一项失败时，先修正轮径、轮距、坐标系、时间戳或 TF 唯一性，不要用 RTAB-Map 参数掩盖底盘里程计错误。

### 2. 检查传感器坐标和时间

```bash
ros2 run tf2_ros tf2_echo base_link front_lidar_link
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 topic hz /scan
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

框架名和话题名必须替换为当前部署的实际值。转动机器人时如果墙面立即形成扇形、斜墙或多重边界，优先检查里程计旋转量、传感器外参和时间同步。

### 3. 启动 Mapping 并检查唯一发布方

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /map
ros2 node info /rtabmap
rbnx logs -t mapping -l info
```

外部里程计模式下，`/rtabmap` 不直接订阅 `/odom` 是预期行为：它从 TF 查询 `odom → base_link`。检查 TF 树时应只有一个 `odom → base_link` 发布方和一个 `map → odom` 发布方。

### 4. 再做建图调参

使用一段包含慢速直行、原地旋转和回到已知区域的固定路线。每次只改一组参数，并比较墙体重影、闭环后全图变形、障碍保留情况和处理频率。

- 地图细节不足时，先检查 `Grid/CellSize` 和输入分辨率。
- 节点过稀时，逐步降低 `RGBD/LinearUpdate` 和 `RGBD/AngularUpdate`；节点过密导致计算堆积时则提高。
- 处理跟不上输入时，降低 `Rtabmap/DetectionRate`，不要仅增大队列。
- 旋转时配准失败时，先检查 TF/时间/里程计，再调整 ICP 门限。
- `Grid/RayTracing=false` 会减少自由空间清理，容易留下幽灵障碍。当前 Webots 基线保持 `true`；混合激光与深度时，仍要验证低矮障碍不会被错误清除。

地图稳定后再运行 Explore、保存空间地图、标注房间并测试导航。保存、加载和位姿重定位接口见[空间地图](../interface-catalog/service/map.md)。
