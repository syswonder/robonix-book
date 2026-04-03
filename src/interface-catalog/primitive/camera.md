# 相机 robonix/prm/camera

相机原语定义 RGB、深度、红外图像流和内参输出。图像数据的 payload 类型直接使用 `sensor_msgs/msg/Image.msg`（来自 `common_interfaces`），streaming RPC 指令定义在 `rust/crates/robonix-interfaces/lib/prm_camera/srv/`。

## 接口

| 契约 ID（`contract_id`） | 模式 | 方向 | ROS IDL | gRPC 映射 | 契约源码（TOML） |
|-----------------------|------|------|---------|-----------|------------------|
| `robonix/prm/camera/rgb` | pub-sub | output | `sensor_msgs/msg/Image.msg` | server-streaming `PrmCameraService/SubscribeRgb` | `rust/contracts/prm/camera_rgb.v1.toml` |
| `robonix/prm/camera/depth` | pub-sub | output | `sensor_msgs/msg/Image.msg` | 同上模式 | `rust/contracts/prm/camera_depth.v1.toml` |
| `robonix/prm/camera/ir` | pub-sub | output | `sensor_msgs/msg/Image.msg` | 同上模式 | — |
| `robonix/prm/camera/intrinsics` | pub-sub | output | `sensor_msgs/msg/CameraInfo.msg` | `message CameraInfo` | — |
| `robonix/prm/camera/rgbd` | pub-sub | output | `robonix_msg/msg/RGBD.msg` | `robonix_msg.RGBD` | — |

## 按需快照与感知（RPC，常与 MCP 对齐）

与连续 **`rgb` / `depth` 流**互补：单帧查询、VLM 输入、调试抓图等多用下列 **一元 RPC** 契约（`[mode].type = rpc`）。实现 MCP 工具时，请用 **`robonix-codegen --lang mcp`** + **`mcp_contract`**，**`DeclareInterface.metadata_json`** 中 **`input_schema`** 用 **`InputCls.json_schema()`**；**`sensor_msgs/Image`** 在 JSON 线格式中 **`data`** 为 **base64**（见 [接口目录首页](../index.md) 中「MCP 与 Python 工具线格式」）。

| 契约 ID（`contract_id`） | 模式 | 方向 | ROS IDL | 说明 | 契约源码（TOML） |
|-----------------------|------|------|---------|------|------------------|
| `robonix/prm/camera/snapshot` | rpc | — | `Empty` → `sensor_msgs/Image` | 单帧 RGB（或主相机） | `rust/contracts/prm/camera_snapshot.v1.toml` |
| `robonix/prm/camera/depth_snapshot` | rpc | — | `Empty` → `sensor_msgs/Image` | 单帧深度图 | `rust/contracts/prm/camera_depth_snapshot.v1.toml` |
| `robonix/prm/perception/detect` | rpc | — | `sensor_msgs/Image` → `std_msgs/String` | 检测结果 JSON（见 TOML） | `rust/contracts/prm/perception_detect.v1.toml` |

gRPC streaming RPC 通过 `.srv` 文件首行的 `# @robonix.grpc stream_server sensor_msgs/msg/Image` 指令生成。ROS 2 端使用普通 topic。

## 典型组合

最常见的配置是 `rgb`（大多数 VLM 需要 RGB 图像）。带深度传感器的相机再加上 `depth`，立体相机或 RGB-D 传感器可以用 `rgbd` 一次性输出。`intrinsics` 对需要 3D 重建的下游算法有用。

## Tiago 桥接实现

`tiago_bridge/node.py` 为 `rgb` 端口同时注册了 gRPC（server-stream）和 ROS 2（topic republish）两种传输；**`camera_snapshot`** 等 MCP 工具通过 **`mcp_contract`** 与上表契约 ID 对齐。gRPC 端实现 `PrmCameraServicer.SubscribeRgb`，以指定帧率从 ROS 2 缓存的最新帧中推送 `sensor_msgs.Image`。ROS 2 端将控制平面分配的 topic 名作为 republish 目标。
