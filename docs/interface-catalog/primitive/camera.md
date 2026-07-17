---
title: 相机
---
<span id="相机-robonixprimitivecamera"></span>
# 相机

相机原语覆盖 RGB 与深度图像，两种取图方式并存：**流式**（`rgb` / `depth`）给场景融合、建图等高频消费者使用；**快照**（`snapshot` / `depth_snapshot`，一元 RPC）供大模型智能体按需取一帧。`topic_out` 只描述单向输出流，不绑定具体传输方式；当前 Webots 提供方通过 ROS 2 发布，Scene 当前也只接入 ROS 2 数据面。当前 Scene 首先从 TF2 查询完整的世界帧到相机光学帧变换；只在 TF2 不可用时，才组合 Atlas 发现的地图位姿与 `robonix/primitive/camera/extrinsics`。

能力约定 TOML 在源码树的 [`capabilities/primitive/camera/`](https://github.com/syswonder/robonix/tree/181d3eb974fd495a795ed120a0a4c6e6f342d179/capabilities/primitive/camera)，接口定义语言（Interface Definition Language，IDL）位于 [`capabilities/lib/camera/`](https://github.com/syswonder/robonix/tree/181d3eb974fd495a795ed120a0a4c6e6f342d179/capabilities/lib/camera) 与固定版本的 [`common_interfaces`](https://github.com/enkerewpo/common_interfaces/tree/0ecd0f70791fe200f057b12bfc626beb21bad639) 子模块。下文的帧方向和标定字段以这些源码定义为准。

新软件包省略 Driver 条目，由框架自动注册共享的 `robonix/lifecycle/driver`；显式选择共享 Driver 的行为相同。未实现生命周期回调时，框架记录警告并执行空操作。

:::warning[后向兼容：相机命名空间 Driver]
`robonix/primitive/camera/driver`、`lifecycle/Driver` 和 `primitive/camera/driver.v1.toml` 只用于仍由软件包自行维护 Driver TOML 的旧实现。目前仍可使用，但计划迁移到共享 Driver；两种 Driver 不能同时注册。详见[生命周期兼容流程](../../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。
:::

## 接口

下表是相机 v1 能力约定的接口摘录；字段级线协议以链接的生成参考和上述 IDL 源文件为准。

| 能力约定 ID（`contract_id`） | 模式 | Tiago 参考实现传输 | 载荷（IDL） | 能力约定 TOML |
|--------------------------|------|-------------|-------------|-----------|
| `robonix/primitive/camera/rgb` | `topic_out` | ROS 2 | [`sensor_msgs/Image`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-image-msg) | `primitive/camera/rgb.v1.toml` |
| `robonix/primitive/camera/depth` | `topic_out` | ROS 2 | [`sensor_msgs/Image`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-image-msg) | `primitive/camera/depth.v1.toml` |
| `robonix/primitive/camera/extrinsics` | `topic_out` | ROS 2（兼容回退） | [`geometry_msgs/TransformStamped`](../../reference/idl.md#common-interfaces-geometry-msgs-msg-transformstamped-msg) | `primitive/camera/extrinsics.v1.toml` |
| `robonix/primitive/camera/intrinsics` | `topic_out` | ROS 2 | [`sensor_msgs/CameraInfo`](../../reference/idl.md#common-interfaces-sensor-msgs-msg-camerainfo-msg) | `primitive/camera/intrinsics.v1.toml` |
| `robonix/primitive/camera/snapshot` | `rpc` | MCP | [`camera/GetCameraImage`](../../reference/idl.md#camera-srv-getcameraimage-srv) | `primitive/camera/snapshot.v1.toml` |
| `robonix/primitive/camera/depth_snapshot` | `rpc` | MCP | [`camera/GetCameraImage`](../../reference/idl.md#camera-srv-getcameraimage-srv) | `primitive/camera/depth_snapshot.v1.toml` |

`snapshot` / `depth_snapshot` 共用 `camera/GetCameraImage` 线协议结构，但使用两个不同的能力约定 ID 选择 RGB 或深度图；两者的请求均为空，应答为 `sensor_msgs/Image`。`rgb` / `depth` 是给 Scene、Mapping 等系统消费者的高频数据面。当前 Scene 选择 ROS 2 传输；其它消费者或提供方也可以为同一 `topic_out` 模式注册 gRPC 流。

### `extrinsics` 的父子帧与方向

`extrinsics` 的载荷是 [`geometry_msgs/TransformStamped`](https://github.com/enkerewpo/common_interfaces/blob/0ecd0f70791fe200f057b12bfc626beb21bad639/geometry_msgs/msg/TransformStamped.msg)。提供方必须按以下语义填写，不能只写一个未说明方向的“相机到机身”矩阵：

- `header.frame_id` 是父帧，即机器人机身或安装参考帧，通常为 `base_link`；
- `child_frame_id` 是子帧，即实际成像使用的相机光学帧，例如 `camera_color_optical_frame`；
- `transform` 是 `T(parent ← child)`：它给出子帧原点和坐标轴在父帧中的位姿，并把子帧点变换到父帧，`p_parent = T(parent ← child) · p_child`。这里不能取逆矩阵。

当前 Scene 的主路径是直接用 TF2 取 `T(world ← camera_optical)`。只有该查询失败时，兼容路径才计算 `T(world ← camera_optical) = T(world ← base) · T(base ← camera_optical)`；实现见 [`_build_camera_to_map_transform`](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/system/scene/scene_service/ingest/perception_concept_graphs.py)。Tiago 参考驱动调用 `lookup_transform(base_frame, cam_frame, ...)`，随后明确写入 `header.frame_id = base_frame` 与 `child_frame_id = cam_frame`；实现见 [`camera_driver/driver.py`](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/examples/webots/primitives/tiago_camera/camera_driver/driver.py)。

`extrinsics` 使用“瞬态本地（`TRANSIENT_LOCAL`）+ 可靠（`RELIABLE`）”的服务质量（Quality of Service，QoS），在启动和重新标定后发布。它是当前 Scene 的兼容回退，不是权威主路径。新部署应提供连通世界帧、机身帧与相机光学帧的完整 URDF/TF；如果仍声明该契约，其变换必须与 TF 一致。

### `intrinsics` 的完整 `CameraInfo`

`intrinsics` 的载荷是 [`sensor_msgs/CameraInfo`](https://github.com/enkerewpo/common_interfaces/blob/0ecd0f70791fe200f057b12bfc626beb21bad639/sensor_msgs/msg/CameraInfo.msg)。提供方不能只填四个焦距参数；至少要按所发布图像的标定模型正确填写下列字段：

| 字段 | 必须表达的内容 |
|---|---|
| `header.frame_id` | 图像的光学帧；原点在光心，`+x` 向图像右侧、`+y` 向下、`+z` 指向镜头前方。与关联 `Image.header.frame_id` 冲突时，IDL 将行为定义为未定义。 |
| `width` / `height` | 该组标定参数对应的像素宽度和高度。用于融合的 RGB、已配准深度与内参必须指向同一像素几何；改变分辨率、裁剪或 binning 时同步更新标定。 |
| `distortion_model` | `d` 所采用的畸变模型；常见值 `plumb_bob` 使用径向与切向畸变。 |
| `d`（D） | 与畸变模型匹配的系数数组；`plumb_bob` 的顺序为 `(k1, k2, t1, t2, k3)`。 |
| `k`（K） | 原始畸变图像的 3×3 行主序内参矩阵，`k[0]=fx`、`k[2]=cx`、`k[4]=fy`、`k[5]=cy`。 |
| `r`（R） | 3×3 行主序校正旋转；单目相机通常为单位矩阵。 |
| `p`（P） | 校正图像使用的 3×4 行主序投影矩阵；单目相机通常 `Tx=Ty=0`，左侧 3×3 为校正后的内参。 |

未标定相机应将 D、K、R、P 保持为零；`K[0] == 0` 表示未标定。当前 Scene 的 [`_cam_info_to_intrinsics`](https://github.com/syswonder/robonix/blob/181d3eb974fd495a795ed120a0a4c6e6f342d179/system/scene/scene_service/service.py) 只读取 `width`、`height` 和 K，并拒绝宽高、`fx/fy` 或主点不完整的消息；它尚不使用 `distortion_model`、D、R、P 做去畸变或校正。因此，“Scene 已收到内参”不能替代完整标定检查。

提供方应使用可靠发布，并保证晚启动消费者能取得最新标定；Tiago 参考驱动使用持久发布并周期性重发最后一条有效消息。它从部署参数构造的仿真回退值是 `plumb_bob`、空 D、单位 R 和单目 P，仅适用于已经确认无需畸变修正的仿真图像；真实相机应转发设备标定产生的完整 `CameraInfo`。

### RGB 与深度对齐

`rgb` 与 `depth` 是两条独立的 `sensor_msgs/Image` 流，当前能力约定没有把它们封装成一个原子同步帧。因此，时间戳必须记录各自采集时间，但契约本身不保证两条消息时间同步。进行逐像素 RGB-D 融合时，提供方还必须先把深度配准到 RGB 的光学帧和像素网格，使两条 `Image` 的 `header.frame_id`、`width`、`height` 与 `CameraInfo` 描述同一几何；不能用图像缩放代替相机几何配准。

Tiago 参考包默认选择 `/head_front_camera/depth_registered/image_raw`，但当前 Scene 只读取两条流的最新帧，并在分辨率不同时缩放分割掩码；它不会补做 RGB—深度外参配准，也没有精确时间同步器。不同光学帧、不同投影模型或必须严格同步的设备，需要在相机原语内部完成注册/同步后再声明这组能力，否则不能把输出标为已对齐。

参考实现：`examples/webots/primitives/tiago_camera`。其软件包清单列出表中全部 7 条能力约定，驱动在初始化后注册 RGB、深度和两项标定数据面。

该实现的 `snapshot` 把 JPEG 字节放进 `sensor_msgs/Image.data`，`depth_snapshot` 还会把深度归一化为灰度 JPEG；这是 Tiago 示例的编码特例，不是能力约定保证，也不保留米制深度。需要原始像素布局或度量深度的消费者应使用 `rgb` / `depth` 数据面，并检查 `Image.encoding`。
