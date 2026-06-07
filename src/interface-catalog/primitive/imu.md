# IMU robonix/primitive/imu

IMU 原语反馈惯性测量（角速度、线加速度、姿态），`topic_out` 数据面，主要给 3D 建图（`service/map`，如 FAST-LIO2 的 IMU + 点云融合）和状态估计用。

能力约定 TOML 在 `capabilities/primitive/imu/`，IDL 在 `capabilities/lib/common_interfaces/sensor_msgs/`。

## 接口

| 能力约定 ID | 模式 | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|
| `robonix/primitive/imu/driver` | `rpc` | [`lifecycle/Driver`](../../reference/idl.md#lifecycle-srv-driver-srv) | `primitive/imu/driver.v1.toml` |
| `robonix/primitive/imu/imu` | `topic_out` | [`sensor_msgs/Imu`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-imu-msg) | `primitive/imu/imu.v1.toml` |

消费方（如建图服务）应通过 atlas 按 `robonix/primitive/imu/imu` 能力约定发现 IMU 数据面，不硬编码 `/livox/imu` 之类的 topic 名。
