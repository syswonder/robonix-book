# 硬件原语接入指南

<!-- toc -->

本文档说明如何将硬件能力以原语（Primitive）形式接入 Robonix。原语是系统对硬件能力的标准化抽象，符合规范后即可被服务、技能和任务调度使用。下面以动作原语 `prm::base.move`（底盘速度控制）为例，从创建包、声明原语到注册与查询，完整走通接入流程。

## 前置条件

- 已安装 Robonix 命令行工具 `rbnx`，参见 [rbnx 命令行工具](../chapter2-user-guide/rbnx-cli.md)。首次运行 rbnx 时会自动创建 `~/.robonix` 目录。

## 创建 robonix 包

在 `~/.robonix/packages` 下新建一个包目录，将底盘驱动或相关 SDK 代码放在该目录下（例如放在 `src/` 中），并创建 `rbnx/` 目录。此时目录结构示例为：

```bash
my_demo_prim_package/
   src/ # 你的代码和其他资源
   rbnx/ # 空文件夹
```

在包根目录创建 `rbnx_manifest.yaml`，填写包的基本信息与编译脚本路径，例如：

```yaml
package:
  name: my_demo_prim_package
  version: 0.0.1
  description: you can write some description here
  maintainer: your_name
  maintainer_email: your_email@example.com
  license: MulanPSL-2.0
  build_script: rbnx/build.sh
```

在 `rbnx/` 下创建 `build.sh`，内容为包内源码的编译命令（例如 `colcon build`、`pip install` 等）。脚本需具有可执行权限（`chmod +x`），且会在包根目录下被 rbnx 执行。若包无需编译，可以不提供 `build_script` 或留空，`rbnx deploy build` 会跳过该包。完成后执行 `rbnx package list` 确认包已出现在列表中，再执行 `rbnx package build my_demo_prim_package` 或 `rbnx deploy build` 进行编译。

## 声明硬件原语

在 manifest 中声明该包能提供的原语。以 `prm::base.move` 为例，其在系统规范中的定义为：

| 标准原语 | 含义 | 输入参数以及 ROS2 类型 | 输出参数以及 ROS2 类型 |
| -------- | ---- | ---------------------- | ---------------------- |
| `prm::base.move` | Move mobile base with velocity command | 1. `cmd_vel: geometry_msgs/msg/Twist` | 1. `odom: nav_msgs/msg/Odometry` | 

在 `rbnx/` 下创建 `start_prm_base_move.sh` 和 `stop_prm_base_move.sh`。启动脚本会在执行 `rbnx deploy start` 时由 rbnx 在包根目录下执行，用于拉起 ROS2 节点或驱动进程；停止脚本在 `rbnx deploy stop` 时执行，用于优雅退出（rbnx 同时会按 PID 结束子进程）。建议在启动脚本内先 source ROS2 与 robonix-sdk（若需要），并用 `exec` 启动最终进程，以便 rbnx 正确管理进程。示例：

```bash
#!/bin/bash
# start_prm_base_move.sh
set -e
[ -n "$ROBONIX_SDK_PATH" ] && [ -f "$ROBONIX_SDK_PATH/install/setup.bash" ] && source "$ROBONIX_SDK_PATH/install/setup.bash"
source /opt/ros/humble/setup.bash
source ~/my_demo_prim_package/install/setup.bash  # 若已 colcon build
exec ros2 run my_demo_prim_package prm_base_move  # 你的节点
```

目录结构示例：
```bash
my_demo_prim_package/
   src/
   rbnx/
      build.sh
      start_prm_base_move.sh
      stop_prm_base_move.sh
   rbnx_manifest.yaml
```

若你的包内程序启动后会订阅 `/ranger/cmd_vel`、发布 `/ranger/odom`，且消息类型与规范一致，则在 `rbnx_manifest.yaml` 中添加如下内容，将标准原语与你的 Topic 映射起来：

```yaml
primitives:
  - name: prm::base.move
    input_schema: '{"cmd_vel":"/ranger/cmd_vel"}'
    output_schema: '{"odom":"/ranger/odom"}'
    metadata: '{"robot":"ranger"}'
    version: 0.0.1
    start_script: rbnx/start_prm_base_move.sh
    stop_script: rbnx/stop_prm_base_move.sh
```

其中 `input_schema` 和 `output_schema` 用于声明原语的输入、输出参数与 Topic 的对应关系，例如将规范中的 `cmd_vel` 映射到实际 Topic `/ranger/cmd_vel`，`odom` 映射到 `/ranger/odom`。`metadata` 用于声明额外信息（如机器人型号、底盘型号），供 robonix 及服务、技能筛选时使用；`version` 用于区分同一包内对同一标准原语的不同实现。

## 注册硬件原语

创建一个 recipe 文件（如 `boot_recipe.yaml`，位置不限），指定要从哪些包中启用哪些能力：

```yaml
name: my_recipe
description: Demo recipe for my robot with base move primitive
packages:
  - name: my_demo_prim_package
    primitives:
      - prm::base.move
```

Recipe 表示“从已安装的包中选取要启用的原语、服务、技能，形成一套运行配置”。注意：`rbnx deploy register` 只会把能力注册到 robonix-core（写入名称、schema、provider 等），不会启动任何进程；进程由 `rbnx deploy start` 根据 manifest 中的 `start_script` 启动。

操作顺序建议如下：

1. 确保 robonix-core 已运行（若使用 Web UI，需设置 `ROBONIX_WEB_ASSETS_DIR` 与 `ROBONIX_WEB_PORT`）。
2. 在 recipe 所在目录执行：

```bash
rbnx deploy register boot_recipe.yaml   # 仅注册到 core，不启动进程
rbnx deploy status                      # 查看注册状态
rbnx deploy build                       # 可选：编译 recipe 中的包
rbnx deploy start                       # 按 start_script 启动原语/服务/技能进程
rbnx deploy status                      # 确认各项为运行状态
```

3. 打开 Robonix Console（Web UI），可查看原语是否已注册且处于运行状态；点击原语卡片中的 View Log 可查看该能力对应进程的日志（完整日志存放在 `~/.robonix/packages/logs`）。

## 查询硬件原语

原语注册后，系统内的服务、技能等即可通过 robonix-core 的原语查询接口请求该原语，根据返回的 `input_schema`、`output_schema` 获知 Topic 映射，再与硬件进行数据交互。

### 通过 ROS2 查询

先 source robonix-sdk 环境，再调用 `/rbnx/prm/query` 服务（类型 `robonix_sdk/srv/QueryPrimitive`）：

```bash
source $ROBONIX_SDK_PATH/install/setup.bash   # 或 rbnx config --show 中的 Robonix SDK path
ros2 service call /rbnx/prm/query robonix_sdk/srv/QueryPrimitive "{name: 'prm::base.move', filter: '{\"robot\":\"ranger\"}'}"
```

- `name`：标准原语名称（如 `prm::base.move`）。
- `filter`：JSON 字符串，按 metadata 过滤（如 `{"robot":"ranger"}`）；空字符串 `""` 表示不过滤。

返回的 `instances` 中每一项包含 `provider`、`version`、`input_schema`、`output_schema`、`metadata` 等，调用方据此订阅/发布对应 Topic 即可与硬件交互。

在 Python 节点中可使用 robonix-sdk 的 `RobonixClient` 进行查询，详见 [robonix-sdk 使用说明](robonix-sdk.md)。

---

## 系统原语规范表

下表为 robonix-core 中已定义的标准原语。硬件厂商实现时，`name` 以及输入、输出的参数名和 ROS2 消息类型必须与下表一致；具体的 Topic 名称由厂商在 `input_schema`、`output_schema` 中自行映射。

| 标准原语名称 | 含义 | 输入参数（名称及 ROS2 类型） | 输出参数（名称及 ROS2 类型） |
| -------------|------|------------------------------|------------------------------|
| 动作与底盘 |||
| `prm::arm.move.ee` | 末端执行器运动到目标位姿 | `pose`: `geometry_msgs/msg/PoseStamped` | `status`: `std_msgs/msg/Bool` |
| `prm::gripper.close` | 闭合夹爪 | （无） | `status`: `std_msgs/msg/Bool` |
| `prm::base.move` | 底盘速度控制 | `cmd_vel`: `geometry_msgs/msg/Twist` | `odom`: `nav_msgs/msg/Odometry` |
| `prm::base.navigate` | 在 map 系下导航到目标位姿 | `goal`: `geometry_msgs/msg/PoseStamped` | （无） |
| `prm::speech.tts` | 文本转语音并播放 | `text`: `std_msgs/msg/String` | `status`: `std_msgs/msg/Bool` |
| 感知与传感器 |||
| `prm::camera.rgb` | 相机 RGB 图像 | （无） | `image`: `sensor_msgs/msg/Image` |
| `prm::camera.depth` | 深度图 | （无） | `depth`: `sensor_msgs/msg/Image` |
| `prm::camera.rgbd` | RGB + 深度 | （无） | `rgb`: `sensor_msgs/msg/Image`, `depth`: `sensor_msgs/msg/Image` |
| `prm::lidar.scan` | 激光雷达扫描 | （无） | `scan`: `sensor_msgs/msg/LaserScan` |
| `prm::sensor.pointcloud` | 3D 点云 | （无） | `pointcloud`: `sensor_msgs/msg/PointCloud2` |
| `prm::trasform.laserscan` | 点云转 LaserScan | `pointcloud`: `sensor_msgs/msg/PointCloud2` | `scan`: `sensor_msgs/msg/LaserScan` |
| `prm::description.urdf` | 机器人 URDF 描述 | （无） | `urdf`: `std_msgs/msg/String` |
| `prm::slam.vision` | 视觉 SLAM（地图与定位） | （无） | `map`: `nav_msgs/msg/OccupancyGrid` |
| 定位与认知 |||
| `prm::base.pose.cov` | AMCL 等提供的带协方差的位姿 | （无） | `pose`: `geometry_msgs/msg/PoseWithCovarianceStamped` |

说明：表中列出的是参数名与 ROS2 类型；在 manifest 的 `input_schema`、`output_schema` 中，需将这些参数名映射到你实际使用的 Topic（例如 `'{"cmd_vel":"/ranger/cmd_vel"}'`、`'{"odom":"/ranger/odom"}'`）。同一标准原语可以有多个实现（不同包或同一包的不同 `version`、`metadata`），查询时通过 `filter` 或 metadata 区分。若需新增标准原语，必须在 robonix-core 的 perception、cognition、action 的 specs 模块中登记，否则注册时校验会失败。