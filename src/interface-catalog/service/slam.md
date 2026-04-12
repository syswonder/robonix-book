# SLAM 服务（`robonix/srv/slam/*` + `robonix/srv/common/map/*`）

SLAM（**S**imultaneous **L**ocalization **A**nd **M**apping）服务负责把传感器数据变成两件东西：
**机器人实时位姿** 和 **空间地图**。Robonix 把"SLAM 控制面"（状态查询、模式切换、地图存取）放在
`robonix/srv/slam/*`，把"SLAM 数据面"（生成的地图/点云/激光扫描）放在
`robonix/srv/common/map/*`——控制与数据分离，下游节点可以只关心自己要的那部分。

参考实现：[`enkerewpo/mapping_rbnx`](https://github.com/enkerewpo/mapping_rbnx)，基于
[FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2)：
LiDAR-Inertial 里程计（ESKF + ikd-Tree）+ GTSAM 位姿图优化回环检测 + 两阶段 coarse-to-fine ICP
重定位 + 层次化 Bundle Adjustment 全局优化。

## 控制面：`robonix/srv/slam/*`

所有接口都是 `rpc`（单次请求单次响应），语义轻、状态改动受控。

| Contract | 请求 → 响应 | 典型调用场景 |
|----------|---|---|
| `slam/status` | Empty → `SlamStatus{mode, odom_hz, cloud_alive, map_file, total_odom_frames, ...}` | 可视化/监控面板轮询"SLAM 还活着吗、跑多少 Hz" |
| `slam/switch_mode` | `{mode: "mapping" \| "localization" \| "idle"}` → ack | 建好地图后切到 localization；维护期切 idle 省算力 |
| `slam/save_map` | `{filename}` → `{path, success, message}` | mapping 结束持久化 3D PCD |
| `slam/load_map` | `{path}` → ack | 上电 / 复位时加载已有地图，进入定位模式 |
| `slam/set_initial_pose` | `geometry_msgs/PoseStamped` → ack | ICP 重定位的初始猜测（例如 RViz `2D Pose Estimate`、上次关机位姿、UWB 粗定位） |

### 为什么这 5 个就"合理且充分"？

SLAM 运行期的**所有用户级状态机**可以用这 5 个原子动作表达：

```
idle ──(switch_mode=mapping)──► mapping ─(save_map)─► (disk)
                                   │
                                   └──(switch_mode=localization)──► localization
                                                                    │   ▲
                                                                    │   │ (set_initial_pose, load_map)
                                                                    ▼   │
                                                                  idle  │
                                                                    │   │
                                                                    └───┘
```

- **状态机本身**：`switch_mode` 覆盖所有节点迁移。
- **持久化**：`save_map` / `load_map` 各管一头，不合并成 `manage_map`——保存失败和加载失败语义差别大，分开便于错误处理。
- **重定位初值**：`set_initial_pose` 单独抽出来是因为"我有地图但机器人不知道自己在哪"是 ICP 的必要输入，和 `load_map` 解耦（可以中途重新设置，不必重新加载地图）。
- **观测**：`status` 返回 `odom_hz`、`cloud_alive`、`total_odom_frames` 等运行时计数，下游不用自己订阅 ROS topic 做存活判断。
- **不加的东西**：不把回环检测、优化开关暴露成 contract——那是实现细节，不同 SLAM 后端的参数完全不同，强放进通用契约会把契约污染成"FASTLIO2 专用"。真要调参，用 package 自己的 YAML 或 ROS param。

同样的 5 个 RPC 可以接 MOLA、LIO-SAM、Cartographer、ORB-SLAM3 等——它们的内部算法差异很大但外部状态机都一样。

## 数据面：`robonix/srv/common/map/*`

所有接口都是 `topic_out`（单向流），下游订阅即可，语义和 ROS topic 一对一。

| Contract | 载荷 | 来源 | 下游典型消费者 |
|----------|------|------|---|
| `common/map/pointcloud` | `sensor_msgs/PointCloud2` | SLAM 的 world-frame 注册点云 | RViz、3D 可视化、3D costmap（voxel / octomap）、碰撞检测 |
| `common/map/occupancy_grid` | `nav_msgs/OccupancyGrid` | 3D→2D 高度投影 | Nav2 `static_layer`、map_server、2D 路径规划 |
| `common/map/scan_2d` | `sensor_msgs/LaserScan` | 3D 点云投影到单层扫描 | Nav2 `obstacle_layer`、AMCL、经典 2D 导航栈 |

### 为什么是这 3 条？

3D 点云、2D 占据栅格、2D 激光扫描是**导航/定位/感知栈的三种事实标准表示**，不同下游选不同抽象层：

- **3D 点云** 原始、无损，适合 3D 规划、深度学习、碰撞检测。但 2D 规划器用不了——它不关心高度。
- **2D 占据栅格** 是 map_server / Nav2 静态层的标准输入，适合"大地图远期规划"（A\*、Dijkstra）。SLAM 负责选一个高度切片（比如 0.1 ~ 1.5 m）投影成栅格。
- **2D LaserScan** 是 Nav2 障碍物层（`obstacle_layer`、`voxel_layer`）、AMCL 等传统 2D 栈的实时输入。即使实际硬件是 3D LiDAR，投成单层 scan 也能直接接上经典栈。

三条流全部由 mapping_rbnx 通过 `pointcloud_to_laserscan` + 高度切片自动生成，下游什么都不用改。这样 Nav2、RViz、旧 2D 栈都能无缝接入同一套 SLAM 后端。

### 为什么不是"一个大 map"？

ROS 生态里 `map_server` 服务的 `OccupancyGrid` 是静态一次性发布的，SLAM 的地图是**运行时持续更新的**。合起来会引入两种矛盾需求：

- 静态 consumer（Nav2 静态层）只想订阅一次初值；
- 动态 consumer（RViz live view）要连续更新。

拆成三条 topic_out 后，各自选 QoS（例如 `occupancy_grid` latched、`pointcloud` best-effort），下游不会互相干扰。

## 输入侧：SLAM 消费了什么

SLAM 服务作为 consumer 需要两个 primitive：

| Contract | 载荷 | 谁来 provide |
|---|---|---|
| `robonix/prm/sensor/lidar3d` | `sensor_msgs/PointCloud2` 或 `livox_ros_driver2/CustomMsg` | MID-360、Velodyne、Ouster 等 3D LiDAR 驱动包 |
| `robonix/prm/sensor/imu` | `sensor_msgs/Imu` | LiDAR 内置 IMU 或独立 IMU |

注意：`robonix/prm/sensor/lidar` 是 2D `LaserScan`，3D LiDAR 用 `lidar3d`。

## 完整数据流

```
硬件层                 原语层                SLAM 服务                  数据面                     下游
MID-360 ──► sensor/lidar3d ──┐
                             ├──► mapping_rbnx ──┬──► common/map/pointcloud ────► RViz / 3D costmap
IMU     ──► sensor/imu    ───┘     (FASTLIO2)    ├──► common/map/occupancy_grid ─► Nav2 static_layer
                                                 ├──► common/map/scan_2d ────────► Nav2 obstacle_layer / AMCL
                                                 └──► prm/base/odom ─────────────► 控制 / 状态估计
                                                         ▲
                                                         │  srv/slam/* 控制面
                                                         │
                                                      Pilot / Agent
```

## TODO

- 语义地图（`srv/common/map/semantic`）：物体级标注的地图，供 Pilot 做"厨房、书房"这种命名空间导航；当前只有几何信息。
- 多机器人融合：mapping_rbnx 目前单机；多机 SLAM 的协作/合并未定。
- 动态障碍物层：`common/map/scan_2d` 是静态投影；行人/移动物体追踪需要额外 contract。
