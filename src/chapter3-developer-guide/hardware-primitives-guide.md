# 硬件原语接入指南

2026.1.31

作者：`wheatfox <wheatfox17@icloud.com>`

## 准备工作

本文档将以动作原语 `prm::base.move` （底盘速度控制）为例，介绍如何将硬件能力接入 robonix。

首先确保你已安装命令行工具 `rbnx`，并且 `~/.robonix` 目录存在。

## 创建 robonix 包

首先，在 `~/.robonix/packages` 目录下创建一个新的目录，并将底盘硬件的 SDK 或者相关驱动代码复制到该目录下，例如 `src` 内，之后创建一个新的目录 `rbnx`，目前的目录结构如下：

```bash
my_demo_prim_package/
   src/ # 你的代码和其他资源
   rbnx/ # 空文件夹
```

此处的整个目录称为一个 robonix 包（Package），用于组织接入 robonix 的代码和数据资源，并通过相关文件和脚本让 robonix 能够识别和使用该包。之后在包的根目录创建一个名为 `rbnx_manifest.yaml` 的文件，并填写以下内容：

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

此时的目录结构如下：

```bash
my_demo_prim_package/
   src/ # 你的代码和其他资源
   rbnx/
   rbnx_manifest.yaml # 包的元信息文件
```

接下来在 `rbnx` 目录下创建一个名为 `build.sh` 的脚本，里面应当包含对包内源码的编译脚本命令（例如运行 ROS2 的 `colcon build` 命令等），该脚本用于让包管理器（`rbnx` 程序）能够编译包。你也可以修改 `build_script` 字段，来指定其他脚本文件作为编译脚本，若不写该字段，包管理器默认尝试 `rbnx/build.sh` 文件作为编译脚本。

接下来运行 `rbnx package list` 命令，你应当能看到你刚刚创建的包已经出现在列表中。然后运行 `rbnx package build my_demo_prim_package` 命令编译你的包。

## 声明硬件原语

接下来我们将该包能够提供的能力在 `rbnx_manifest.yaml` 文件中进行声明。对于 `prm::base.move` 硬件原语，其定义如下：

| 标准原语 | 含义 | 输入参数以及 ROS2 类型 | 输出参数以及 ROS2 类型 |
| -------- | ---- | ---------------------- | ---------------------- |
| `prm::base.move` | Move mobile base with velocity command | 1. `cmd_vel: geometry_msgs/msg/Twist` | 1. `odom: nav_msgs/msg/Odometry` | 

首先，在 `rbnx\` 目录内创建两个脚本 `start_prm_base_move.sh` 和 `stop_prm_base_move.sh`，分别用于 robonix 启动和停止 `prm::base.move` 硬件原语，例如对于 ROS2 源码来说，`start_prm_base_move.sh` 脚本可以如下：

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/my_demo_prim_package/install/setup.bash
ros2 run my_demo_prim_package prm_base_move # example
```

此时目前目录结构如下：
```bash
my_demo_prim_package/
   src/
   rbnx/
      build.sh
      start_prm_base_move.sh
      stop_prm_base_move.sh
   rbnx_manifest.yaml
```

接下来，我们可以在 `rbnx_manifest.yaml` 文件中我们的包支持提供 `prm::base.move` 原语，假设包内的程序启动后，会持续接受路径为 `/ranger/cmd_vel` 的 Topic，并发布路径为 `/ranger/odom` 的 Topic，且对应的 Topic 类型与标准原语定义一致，则可以在文件内部添加以下内容：

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

其中 `input_schema` 和 `output_schema` 字段用于声明原语的输入和输出参数，在这里我们完成最重要的一步，就是将标准原语的输入和输出参数与包内程序的输入和输出参数进行映射，例如 `cmd_vel` 参数对应 `/ranger/cmd_vel` Topic，`odom` 参数对应 `/ranger/odom` Topic。

对于 `metadata` 字段，用于声明原语的额外数据，例如机器人型号、底盘型号等，这些数据供 robonix 管理，服务、技能可以根据 metadata 筛选需要的原语实现。对于 `version` 字段，用于区分包内部同一个原语的不同实现，例如同一个底盘实现了两套控制算法，但是推荐采用 metadata 来作为区分标识，version 仅用于版本选择（例如保留了旧版本的原语实现）。

## 注册硬件原语

接下来我们将该硬件原语注册到 robonix 中，在 robonix 中，除了通过原始 API（ROS2 service）直接调用 robonix-core 的注册接口外，更推荐的方法是通过命令行工具 `rbnx` 进行注册和管理。

首先创建一个文件 `boot_recipe.yaml`（位置不限），并填写以下内容：

```yaml
name: my_recipe
description: Demo recipe for my robot with base move primitive
packages:
  - name: my_demo_prim_package
    primitives:
      - prm::base.move
```

一个 recipe 文件代表“我希望从哪些已安装的包里面、选择每个包里的一部分能力（原语、服务、技能），形成一套运行配置加载并注册到 robonix 系统中”，首先确保 robonix-core 正在运行（`/robonix-core` ROS2 node 存在），然后在 recipe 所在目录中运行：

```bash
rbnx deploy register boot_recipe.yaml
rbnx deploy status # 查看注册状态
rbnx deploy build # 对于 recipe 中声明到的 package，执行一次编译
rbnx deploy start # 启动相关 package 的对应能力（原语、服务、技能），并维护其生命周期（进程组）
rbnx deploy status # 查看运行状态（正常所有能力项均应处于运行状态）
```

此时打开 robonix 的 Web UI 界面（Robonix Console），你应当能看到 `prm::base.move` 硬件原语已经注册到 robonix 中，并且处于运行状态，点击对应的原语卡片后点击 View Log，即可看到你的相关程序（例如硬件 SDK）的日志输出（由 `rbnx` 程序自动捕获，完整 log 保存于对应运行 `rbnx` 的主机的 `~/.robonix/packages/logs` 目录下）。

## 查询硬件原语

在原语注册之后，系统内的服务、技能以及系统本身的子系统（如语义地图子系统）就能够请求对应的原语并最终进行数据交互。首先给通过命令行 `ros2` 程序进行查询的方式（参考 [QueryPrimitive.srv](https://github.com/syswonder/robonix/blob/main/rust/robonix-sdk/srv/primitive/QueryPrimitive.srv) 文件）：

```bash
# string name                    # Standard primitive name
# string filter                  # JSON: filter by metadata (e.g., {"resolution":">=720p"}, {"index":0}). Empty string means no filter
# ---
# robonix_sdk/PrimitiveInstance[] instances

source $PATH_TO_ROBONIX_SDK/install/setup.bash # 替换为你的 robonix-sdk 文件夹路径
ros2 service call /rbnx/prm/query robonix_sdk/srv/QueryPrimitive "{name: 'prm::base.move', filter: '{'robot':'ranger'}'}"
```