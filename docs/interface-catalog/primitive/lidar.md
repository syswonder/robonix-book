<span id="激光雷达-robonixprimitivelidar"></span>
# 激光雷达

激光雷达原语覆盖二维扫描与三维点云。`lidar`（二维 `LaserScan`）和 `lidar3d`（`PointCloud2`）都是不绑定具体传输方式的 `topic_out` 数据面，供建图（`service/map`）、避障和场景融合使用；`snapshot` 供大模型智能体按需取一帧二维扫描。

能力约定 TOML 在 `capabilities/primitive/lidar/`，IDL 在 `capabilities/lib/lidar/` 与 `capabilities/lib/common_interfaces/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/primitive/lidar/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/lidar/driver.v1.toml` |
| `robonix/primitive/lidar/lidar` | `topic_out` | [`sensor_msgs/LaserScan`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-laserscan-msg) | `primitive/lidar/lidar.v1.toml` |
| `robonix/primitive/lidar/lidar3d` | `topic_out` | [`sensor_msgs/PointCloud2`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-pointcloud2-msg) | `primitive/lidar/lidar3d.v1.toml` |
| `robonix/primitive/lidar/snapshot` | `rpc` | [`lidar/GetLaserScan`](../../reference/idl.md#lidar-srv-getlaserscan-srv) | `primitive/lidar/lidar_snapshot.v1.toml` |

一个雷达包按硬件实现 `lidar`（2D 雷达）、`lidar3d`（3D 雷达，如 Livox / Velodyne）或两者。**vendor 中立**：原语只暴露标准 `LaserScan` / `PointCloud2`，厂家 SDK 的私有点云格式应在驱动内转成标准消息，不外泄到能力约定。

> **当前运行时限制**：Scene 的 3D 自动发现仍查询错误的 `robonix/primitive/lidar/pointcloud`，不会自动接入标准 `robonix/primitive/lidar/lidar3d`。修复 Scene 前，不能把声明了 `lidar3d` 等同于已经启用 Scene 3D 融合。

`snapshot` 的规范 ID 是表中的 `robonix/primitive/lidar/snapshot`。`GetLaserScan.srv` 的生成参考目前仍显示旧名称 `robonix/primitive/lidar/lidar_snapshot`；提供方注册时不要使用旧名称。

参考实现：`examples/webots/primitives/tiago_lidar`（`/scanner` → `lidar` + `snapshot`，输出归一化的 `/scanner_normalized`）。驱动初始化成功后会向 Atlas 声明 `robonix/primitive/lidar/lidar`，但当前 `package_manifest.yaml` 只列出 `snapshot` 和 `driver`；修复清单前，软件包元数据不是完整的运行时能力清单。
