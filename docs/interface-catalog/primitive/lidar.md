---
title: 激光雷达
---
<span id="激光雷达-robonixprimitivelidar"></span>
# 激光雷达

激光雷达原语覆盖二维扫描与三维点云。`robonix/primitive/lidar/lidar`（二维 `LaserScan`）和 `robonix/primitive/lidar/lidar3d`（`PointCloud2`）都是不绑定具体传输方式的 `topic_out` 数据面，供建图、避障和场景融合使用；`robonix/primitive/lidar/snapshot` 供大模型智能体按需取一帧二维扫描。

能力约定 TOML 在 `capabilities/primitive/lidar/`，接口定义语言（Interface Definition Language，IDL）文件在 `capabilities/lib/lidar/` 与 `capabilities/lib/common_interfaces/`。

> 表中的命名空间 Driver 是已有软件包的兼容接口。新软件包省略 Driver 条目时由框架自动使用共享的 `robonix/lifecycle/driver`；显式共享仍受支持，两种 Driver 只能选择一条。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。

## 接口

| 能力约定 ID | 模式 | Tiago 参考实现传输 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/primitive/lidar/driver` | `rpc` | gRPC（旧命名空间 Driver） | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/lidar/driver.v1.toml` |
| `robonix/primitive/lidar/lidar` | `topic_out` | ROS 2 | [`sensor_msgs/LaserScan`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-laserscan-msg) | `primitive/lidar/lidar.v1.toml` |
| `robonix/primitive/lidar/lidar3d` | `topic_out` | 未实现 | [`sensor_msgs/PointCloud2`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-pointcloud2-msg) | `primitive/lidar/lidar3d.v1.toml` |
| `robonix/primitive/lidar/snapshot` | `rpc` | MCP | [`lidar/GetLaserScan`](../../reference/idl.md#lidar-srv-getlaserscan-srv) | `primitive/lidar/lidar_snapshot.v1.toml` |

一个雷达软件包可以按硬件实现 `lidar`（二维雷达）、`lidar3d`（三维雷达，如 Livox 或 Velodyne）或两者。原语接口与厂商无关：提供方只暴露标准 `LaserScan` 或 `PointCloud2`，厂商 SDK 的私有格式应在驱动内转换为标准消息。

`snapshot` 的规范 ID 是 `robonix/primitive/lidar/snapshot`。二维、三维数据是否进入建图或场景融合，取决于对应服务的提供方绑定与运行参数；仅声明 `lidar` 或 `lidar3d` 不会自动启用上层处理。

参考实现位于 `examples/webots/primitives/tiago_lidar`，它把 `/scanner` 转换为二维激光雷达数据和按需快照，并输出归一化的 `/scanner_normalized`。实现新雷达包时，应让软件包清单中的 `capabilities` 与 Atlas 中的实际声明一致，并用 `rbnx caps -v` 核对提供方。
