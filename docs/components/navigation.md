---
title: 导航服务使用指南
---

# 导航服务使用指南

本页面向要在一台机器人上把导航跑通并调到可用的工程师。阅读前需要机器人已经有可用的建图服务、底盘里程计和雷达，并且能用 `rbnx boot` 启动部署。读完本页可以独立完成：绑定导航的三路输入、写出一份属于本机器人的 Nav2 参数文件、把速度上限与机器人真实能力对齐、以及在目标被反复取消时找到是哪一道守卫在动作。

接口清单与载荷定义在[导航接口页](../interface-catalog/service/navigation.md)，本页不重复。

本页依据上游 `service-navigation-rbnx` 提交 `cb4f2177` 编写。

## 服务负责什么

导航服务是系统安装的 ROS 2 Nav2 栈的包装层。它通过 Atlas 发现地图、里程计和雷达输入，把它们接到 Nav2 上，暴露 `robonix/service/navigation/*` 能力，并在 Nav2 的输出之后再加一道最终速度守卫。

**导航行为属于机器人部署，不属于这个软件包。** 每台机器人必须自带一份完整的 Nav2 参数 YAML，用 `params_file` 指过去。上游的 `config/nav2_params.example.yml` 是中性示例，不是任何一台机器人的档案。

## 最小配置

```yaml
service:
  - name: nav2
    url: https://github.com/syswonder/service-navigation-rbnx
    branch: main
    config:
      params_file: config/nav2_params.yaml
      provider_ids:
        map: mapping
        odom: chassis
        scan: lidar
      dynamic_speed:
        max_linear_speed_mps: 0.3
        default_percentage: 75
        step_percentage: 20
        min_percentage: 20
```

相对路径从 `robonix_manifest.yaml` 所在目录解析。

`provider_ids` 支持四个角色：`map`、`odom`、`scan`、`scan_cloud`。它给的是 Atlas 上的提供方 ID，不是 ROS 话题名——服务在 `ConnectCapability` 拿到真实端点之后才订阅。

### 三维雷达

没有原生 LaserScan 的机器人绑 `scan_cloud` 并显式声明投影适配器：

```yaml
      provider_ids:
        map: mapping
        odom: chassis
        scan_cloud: lidar3d
      scan_projection:
        enabled: true
        target_frame: base_link
        min_height_m: 0.10
        max_height_m: 1.50
        range_max_m: 12.0
```

高度窗口是逐机器人的物理量，不能照抄。轮式底盘上 `base_link` 通常在地面附近，取 `0.30` 到 `1.40` 一类的正区间；四足机器人的 `base_link` 在机身中心，Unitree Go2 的部署取的是 `-0.25` 到 `0.50`，因为矮障碍物在 `base_link` 之下。抄错方向的结果是把地面当障碍或者把矮障碍全部丢掉。

`self_filter_margin_m` 是在 Soma 的外接圆半径之外再留的余量。Go2 的部署把它压到 `0.02` 并写明了理由：Soma 给出的外接圆本来就偏保守，再加一圈大余量会在机器人周围造成一个近距离盲环，比留不住几个自身回波危险得多。

## 关键配置字段

| 字段 | 默认值 | 含义 |
|---|---|---|
| `params_file` | 必填 | 部署自有的完整 Nav2 参数 YAML |
| `provider_ids` | 必填 | 输入角色到 Atlas 提供方 ID 的映射 |
| `dynamic_speed` | 必填 | 运行时调速策略，见下一节 |
| `bt_xml_file` | 无 | 部署自有的行为树 XML |
| `action_wait_s` | `45.0` | `CMD_INIT` 等待 `navigate_to_pose` 动作服务器就绪的上限。超时即初始化失败并拆掉已拉起的 Nav2 与守卫进程。真机部署普遍调到 `80` 到 `90` |
| `use_sim_time` | `false` | 用 ROS `/clock` |
| `velocity_output_topic` | `/cmd_vel` | 最终速度守卫的发布话题。接入实体机器人期间设成 `/robonix/nomotion/cmd_vel` 之类的空转汇聚点。空值、相对路径和畸形话题名会让启动直接失败 |
| `guard_terminal_xy_m` | `0.45` | 距全局路径终点多近时，原地旋转被当作终点对准并适用更严的限制 |
| `guard_terminal_timeout_s` | `15.0` | 终点对准阶段的总时限 |
| `guard_no_progress_s` | `3.0` | 终点旋转期间偏航误差没有实质减少的最长时间 |
| `guard_global_spin_timeout_s` | `25.0` | 路线上任意位置连续原地旋转的最长时间，包含规划器与控制器的恢复循环 |
| `guard_global_spin_limit_rad` | `6.783` | 连续原地旋转的累计角度上限 |

`params_profile` 是已废弃的选择器，新部署不要用，现有部署会在启动时收到迁移警告。

## 调速策略要和机器人的真实能力对齐

`dynamic_speed.max_linear_speed_mps` 是部署给出的平面速度硬上限，单位 m/s，语义是 `sqrt(vx² + vy²)`。它必须等于所选控制器实际形成的平面天花板——同时考虑 `max_speed_xy` 和更严的逐轴 `max_vel_x` / `max_vel_y`。最终速度守卫会独立地再执行一次这个限制。

`default_percentage` 是这个上限的起始百分比。现有部署普遍取 75，于是 `0.3 × 75% = 0.225 m/s` 就是启动速度。`step_percentage` 是加减的百分点数，不是倍数。

角速度约束留在部署自有的 Nav2 YAML 里（例如 DWB 的 `max_vel_theta`）。Robonix 在这一层不提供独立的角速度策略；控制器在处理 Nav2 速度限制时可能按比例调整自己的内部运动学。

`adjust_speed` 接受 `faster`、`slower`、`normal`；`set_speed_limit` 给明确百分比；`get_speed_limit` 读当前值与配置值。默认情况下一次变更属于当前这一次导航运行，运行结束自动恢复会话限制；`persist=true` 才会跨运行改变提供方会话的限制。这三个操作都不会重启导航、取消目标或重新提交目标。

## Nav2 参数系统

这是配置导航时真正花时间的地方。下面讲的是 Nav2 自己的参数体系，以及 Robonix 在它之上加的两件事。

### 文件结构

Nav2 的参数 YAML 按**节点名**分节，每个节点下面是一个 `ros__parameters` 映射：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.3
```

一份完整的机器人参数文件通常包含这些节点：`planner_server`、`controller_server`、`bt_navigator`、`behavior_server`、`velocity_smoother`、`collision_monitor`、`local_costmap`、`global_costmap`、`waypoint_follower`、`lifecycle_manager`。缺任何一个都会让对应能力不可用，而不是用默认值。

### 插件选择决定了哪些参数存在

Nav2 的大部分行为来自插件，而**插件的参数只在选了它之后才存在**。`FollowPath` 下写 `plugin: "dwb_core::DWBLocalPlanner"`，那一节里的 `max_vel_x`、`vx_samples`、`critics` 才有意义；换成别的控制器，这些键会被静默忽略。同理适用于规划器、目标检查器（goal checker）、进度检查器（progress checker）和代价地图的每一层。

调参前先确认自己改的那一节对应的插件确实被选中了。改了半天没反应，多数是改在了没启用的插件下面。

### Robonix 注入的六个占位符

参数文件里可以写占位符，服务在 `CMD_INIT` 时替换成运行时解析出来的真值。这样同一份文件不必硬编码话题名，也不必重复声明机器人尺寸。

| 占位符 | 替换成 |
|---|---|
| `__ROBONIX_MAP_TOPIC__` | 绑定的地图话题 |
| `__ROBONIX_ODOM_TOPIC__` | 绑定的里程计话题 |
| `__ROBONIX_SCAN_TOPIC__` | 绑定的 LaserScan 话题 |
| `__ROBONIX_SCAN_CLOUD_TOPIC__` | 绑定的点云话题 |
| `__ROBONIX_BT_XML__` | 解析后的行为树路径 |
| `__ROBONIX_FOOTPRINT__` | Soma 声明的足迹多边形 |

占位符出现在文件里但解析不出值时，启动直接失败，不会带着空字符串继续跑。

**足迹只写一次。** 两张代价地图都用 `footprint: "__ROBONIX_FOOTPRINT__"`，真值来自 Soma 的 `robonix/system/soma/footprint`。不要在 Nav2 文件里手写多边形：那样机器人尺寸就有了两个来源，而它们迟早会不一致。`footprint_padding` 仍然由部署自己决定。

### 实际需要调的几组

**速度与加速度**（`controller_server` 下控制器插件一节）。`max_vel_x`、`max_vel_theta`、`max_speed_xy` 是平面能力的来源，`dynamic_speed.max_linear_speed_mps` 必须和它们算出来的天花板一致。`velocity_smoother` 节点还有自己一套限制，比控制器更严时它才是真正生效的那个。

**目标容差**（目标检查器一节）。`xy_goal_tolerance` 和 `yaw_goal_tolerance` 直接决定“到了没有”。Benben 用的是 `0.30` 和 `0.52` rad（约 30 度）。容差给得比机器人实际能收敛的精度还小，目标会永远完不成。

**代价地图**（`local_costmap` / `global_costmap`）。`resolution` 通常与地图服务的 `Grid/CellSize` 一致，现有部署都是 `0.05`。`inflation_radius` 和 `cost_scaling_factor` 决定机器人离墙多远，Benben 用 `0.60` 和 `2.2`。`observation_sources` 必须与实际绑定的传感器对上，`expected_update_rate` 设成 `0.0` 表示不因传感器超时而清层。

**层的顺序**。全局代价地图通常是静态层、障碍层、膨胀层；局部代价地图没有静态层。膨胀层必须在最后。

**行为树**。`bt_xml_file` 指向部署自有的 XML，它决定失败时执行哪些恢复动作。Lite3 的部署刻意用了一棵保守的树：只做重规划、清代价地图和等待，从不在四足机器人上调用 Nav2 通用的 `Spin` 和 `BackUp`。这是一个好例子——恢复动作是否安全取决于机器人形态。

## 四道守卫，以及目标被无故取消时查哪一个

Nav2 之后还有一道最终速度守卫，`guard_*` 字段配置它。它解决的是机器人原地空转、或者在终点附近反复微调却永不结束的情况。

| 守卫 | 触发条件 |
|---|---|
| 终点对准 | 进入 `guard_terminal_xy_m` 范围后适用更严限制 |
| 终点超时 | 对准阶段超过 `guard_terminal_timeout_s` |
| 无进展 | 对准期间 `guard_no_progress_s` 内偏航误差没有实质减少 |
| 全局自转 | 任意位置连续原地旋转超过 `guard_global_spin_timeout_s` 或累计超过 `guard_global_spin_limit_rad` |

**默认值是给理想底盘的。** Hantewin Benben 的部署把 `guard_no_progress_s` 调到 `8.0`、`guard_terminal_timeout_s` 调到 `25.0`，理由写在清单注释里而且是量出来的：这台滑移转向底盘在 `0.15 rad/s` 的终点旋转指令下只能实现约 `0.12 rad/s` 的有效转速，收掉最后约 `0.8 rad` 的偏航误差要 6 到 7 秒，而默认的 3 秒无进展窗口会把正在收敛的目标取消掉。

这是调这组参数的正确方法：先量出机器人在终点旋转指令下的**实际**角速度，再算出收掉典型偏航误差需要多久，然后把窗口设得比它宽。凭感觉往大调只会让真正卡住的情况也拖很久才被发现。

## 启动时序

`Driver(CMD_INIT)` 依次做六件事：

1. 解析选定的 Atlas 提供方；
2. 解析并实例化部署自有的 Nav2 YAML（替换占位符）；
3. 按需启动点云到 LaserScan 的适配器；
4. 启动 Nav2 并等待 `navigate_to_pose` 动作服务器；
5. 连上 Nav2 的 `speed_limit` 订阅端；
6. 暴露导航、状态、取消与调速能力。

必需的提供方缺失时返回 `deferred`；配置非法或 Nav2 启动失败返回 `error`，并拆掉所有已拉起的子进程。

## 定位常见问题

**启动时 `action_wait_s` 超时。** 真机上 45 秒常常不够，现有部署普遍用 80 到 90。但先确认不是 Nav2 自己起不来：日志里会有对应节点的报错，盲目调大只是推迟失败。

**目标在快到终点时被取消。** 看守卫日志判断是哪一道。多数是 `guard_no_progress_s` 对这台底盘太紧，按上面那段的方法量一遍再调。

**机器人贴着墙走或者离墙太远。** 调 `inflation_radius` 和 `cost_scaling_factor`，不要改足迹。足迹来自 Soma，改它会让系统里其他依赖机器人尺寸的部分跟着错。

**改了参数没有任何反应。** 确认那一节对应的插件确实被选中，以及改的是不是被 `velocity_smoother` 或最终速度守卫在更下游重新限制掉了。

**接入实体机器人时不想让它动。** 把 `velocity_output_topic` 指向 `/robonix/nomotion/cmd_vel`，整条链路照常跑，只是没有人订阅那个话题。
