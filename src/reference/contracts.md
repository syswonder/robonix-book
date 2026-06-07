# 能力约定参考（自动生成）

> 由 robonix v0.1.0 · commit 953df33-dirty · 2026-06-05 自动生成，请勿手改。重新生成：`rbnx docs`。

本页罗列 `capabilities/` 下的所有标准能力约定（共 57 条）。
载荷列链到对应的 [ROS IDL](idl.md)。概念与字段含义见 [接口目录](../interface-catalog/index.md)。

[toc]

## primitive

| 能力约定 ID | kind | mode | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/primitive/audio/driver` | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/audio/driver.v1.toml` |
| `robonix/primitive/audio/list_devices` | primitive | `rpc` | [`audio/srv/ListAudioDevices.srv`](idl.md#audio-srv-listaudiodevices-srv) | `primitive/audio/list_devices.v1.toml` |
| `robonix/primitive/audio/mic` | primitive | `topic_out` | [`audio/msg/AudioChunk.msg`](idl.md#audio-msg-audiochunk-msg) | `primitive/audio/mic.v1.toml` |
| `robonix/primitive/audio/select_device` | primitive | `rpc` | [`audio/srv/SelectAudioDevice.srv`](idl.md#audio-srv-selectaudiodevice-srv) | `primitive/audio/select_device.v1.toml` |
| `robonix/primitive/audio/speaker` | primitive | `topic_in` | [`audio/msg/AudioChunk.msg`](idl.md#audio-msg-audiochunk-msg) | `primitive/audio/speaker.v1.toml` |
| `robonix/primitive/camera/depth` | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/Image.msg`](idl.md#common-interfaces-sensor-msgs-msg-image-msg) | `primitive/camera/depth.v1.toml` |
| `robonix/primitive/camera/depth_snapshot` | primitive | `rpc` | [`camera/srv/GetCameraImage.srv`](idl.md#camera-srv-getcameraimage-srv) | `primitive/camera/depth_snapshot.v1.toml` |
| `robonix/primitive/camera/driver` | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/camera/driver.v1.toml` |
| `robonix/primitive/camera/extrinsics` | primitive | `topic_out` | [`common_interfaces/geometry_msgs/msg/TransformStamped.msg`](idl.md#common-interfaces-geometry-msgs-msg-transformstamped-msg) | `primitive/camera/extrinsics.v1.toml` |
| `robonix/primitive/camera/rgb` | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/Image.msg`](idl.md#common-interfaces-sensor-msgs-msg-image-msg) | `primitive/camera/rgb.v1.toml` |
| `robonix/primitive/camera/snapshot` | primitive | `rpc` | [`camera/srv/GetCameraImage.srv`](idl.md#camera-srv-getcameraimage-srv) | `primitive/camera/snapshot.v1.toml` |
| `robonix/primitive/chassis/driver` | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/chassis/driver.v1.toml` |
| `robonix/primitive/chassis/move` | primitive | `rpc` | [`chassis/srv/ExecuteMoveCommand.srv`](idl.md#chassis-srv-executemovecommand-srv) | `primitive/chassis/move.v1.toml` |
| `robonix/primitive/chassis/odom` | primitive | `topic_out` | [`common_interfaces/nav_msgs/msg/Odometry.msg`](idl.md#common-interfaces-nav-msgs-msg-odometry-msg) | `primitive/chassis/odom.v1.toml` |
| `robonix/primitive/chassis/twist_in` | primitive | `topic_in` | [`common_interfaces/geometry_msgs/msg/Twist.msg`](idl.md#common-interfaces-geometry-msgs-msg-twist-msg) | `primitive/chassis/twist_in.v1.toml` |
| `robonix/primitive/imu/driver` | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/imu/driver.v1.toml` |
| `robonix/primitive/imu/imu` | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/Imu.msg`](idl.md#common-interfaces-sensor-msgs-msg-imu-msg) | `primitive/imu/imu.v1.toml` |
| `robonix/primitive/lidar/driver` | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/lidar/driver.v1.toml` |
| `robonix/primitive/lidar/lidar` | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/LaserScan.msg`](idl.md#common-interfaces-sensor-msgs-msg-laserscan-msg) | `primitive/lidar/lidar.v1.toml` |
| `robonix/primitive/lidar/lidar3d` | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/PointCloud2.msg`](idl.md#common-interfaces-sensor-msgs-msg-pointcloud2-msg) | `primitive/lidar/lidar3d.v1.toml` |
| `robonix/primitive/lidar/snapshot` | primitive | `rpc` | [`lidar/srv/GetLaserScan.srv`](idl.md#lidar-srv-getlaserscan-srv) | `primitive/lidar/lidar_snapshot.v1.toml` |

## service

| 能力约定 ID | kind | mode | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/service/map/driver` | service | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `service/map/driver.v1.toml` |
| `robonix/service/map/occupancy_grid` | service | `topic_out` | [`common_interfaces/nav_msgs/msg/OccupancyGrid.msg`](idl.md#common-interfaces-nav-msgs-msg-occupancygrid-msg) | `service/map/occupancy_grid.v1.toml` |
| `robonix/service/map/odom` | service | `topic_out` | [`common_interfaces/nav_msgs/msg/Odometry.msg`](idl.md#common-interfaces-nav-msgs-msg-odometry-msg) | `service/map/odom.v1.toml` |
| `robonix/service/map/pointcloud` | service | `topic_out` | [`common_interfaces/sensor_msgs/msg/PointCloud2.msg`](idl.md#common-interfaces-sensor-msgs-msg-pointcloud2-msg) | `service/map/pointcloud.v1.toml` |
| `robonix/service/map/pose` | service | `topic_out` | [`common_interfaces/geometry_msgs/msg/PoseWithCovarianceStamped.msg`](idl.md#common-interfaces-geometry-msgs-msg-posewithcovariancestamped-msg) | `service/map/pose.v1.toml` |
| `robonix/service/memory/compact` | service | `rpc` | [`memory/srv/Compact.srv`](idl.md#memory-srv-compact-srv) | `service/memory/compact.v1.toml` |
| `robonix/service/memory/save` | service | `rpc` | [`memory/srv/Save.srv`](idl.md#memory-srv-save-srv) | `service/memory/save.v1.toml` |
| `robonix/service/memory/search` | service | `rpc` | [`memory/srv/Search.srv`](idl.md#memory-srv-search-srv) | `service/memory/search.v1.toml` |
| `robonix/service/navigation/cancel` | service | `rpc` | [`navigation/srv/CancelNavigation.srv`](idl.md#navigation-srv-cancelnavigation-srv) | `service/navigation/cancel.v1.toml` |
| `robonix/service/navigation/driver` | service | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `service/navigation/driver.v1.toml` |
| `robonix/service/navigation/navigate` | service | `rpc` | [`navigation/srv/Navigate.srv`](idl.md#navigation-srv-navigate-srv) | `service/navigation/navigate.v1.toml` |
| `robonix/service/navigation/status` | service | `rpc` | [`navigation/srv/GetNavigationStatus.srv`](idl.md#navigation-srv-getnavigationstatus-srv) | `service/navigation/status.v1.toml` |
| `robonix/service/speech/asr` | service | `rpc` | [`asr/srv/Recognize.srv`](idl.md#asr-srv-recognize-srv) | `service/speech/asr.v1.toml` |
| `robonix/service/speech/asr_stream` | service | `rpc_bidirectional_stream` | [`asr/srv/RecognizeStream.srv`](idl.md#asr-srv-recognizestream-srv) | `service/speech/asr_stream.v1.toml` |
| `robonix/service/speech/dialog` | service | `rpc_server_stream` | [`speech/srv/StartDialog.srv`](idl.md#speech-srv-startdialog-srv) | `service/speech/dialog.v1.toml` |
| `robonix/service/speech/driver` | service | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `service/speech/driver.v1.toml` |
| `robonix/service/speech/list_speakers` | service | `rpc` | [`speech/srv/ListSpeakers.srv`](idl.md#speech-srv-listspeakers-srv) | `service/speech/list_speakers.v1.toml` |
| `robonix/service/speech/speak` | service | `rpc` | [`speech/srv/Speak.srv`](idl.md#speech-srv-speak-srv) | `service/speech/speak.v1.toml` |
| `robonix/service/speech/tts` | service | `rpc` | [`tts/srv/Synthesize.srv`](idl.md#tts-srv-synthesize-srv) | `service/speech/tts.v1.toml` |
| `robonix/service/speech/tts_stream` | service | `rpc_server_stream` | [`tts/srv/SynthesizeStream.srv`](idl.md#tts-srv-synthesizestream-srv) | `service/speech/tts_stream.v1.toml` |
| `robonix/service/voiceprint/delete` | service | `rpc` | [`voiceprint/srv/DeleteEnrolled.srv`](idl.md#voiceprint-srv-deleteenrolled-srv) | `service/voiceprint/delete.v1.toml` |
| `robonix/service/voiceprint/enroll` | service | `rpc` | [`voiceprint/srv/Enroll.srv`](idl.md#voiceprint-srv-enroll-srv) | `service/voiceprint/enroll.v1.toml` |
| `robonix/service/voiceprint/identify` | service | `rpc` | [`voiceprint/srv/Identify.srv`](idl.md#voiceprint-srv-identify-srv) | `service/voiceprint/identify.v1.toml` |
| `robonix/service/voiceprint/list` | service | `rpc` | [`voiceprint/srv/ListEnrolled.srv`](idl.md#voiceprint-srv-listenrolled-srv) | `service/voiceprint/list.v1.toml` |

## system

| 能力约定 ID | kind | mode | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|
| `robonix/system/executor` | service | `rpc_server_stream` | [`executor/srv/Execute.srv`](idl.md#executor-srv-execute-srv) | `system/executor.v1.toml` |
| `robonix/system/liaison/submit` | system | `rpc_server_stream` | [`liaison/srv/SubmitTask.srv`](idl.md#liaison-srv-submittask-srv) | `system/liaison/submit.v1.toml` |
| `robonix/system/liaison/voice` | system | `rpc_server_stream` | [`liaison/srv/StartVoiceSession.srv`](idl.md#liaison-srv-startvoicesession-srv) | `system/liaison/voice.v1.toml` |
| `robonix/system/pilot` | service | `rpc_server_stream` | [`pilot/srv/SubmitTask.srv`](idl.md#pilot-srv-submittask-srv) | `system/pilot.v1.toml` |
| `robonix/system/scene/get_object_context` | service | `rpc` | [`semantic_map/srv/GetObjectContext.srv`](idl.md#semantic-map-srv-getobjectcontext-srv) | `system/scene/get_object_context.v1.toml` |
| `robonix/system/scene/get_scene_graph` | service | `rpc` | [`semantic_map/srv/GetSceneGraph.srv`](idl.md#semantic-map-srv-getscenegraph-srv) | `system/scene/get_scene_graph.v1.toml` |
| `robonix/system/scene/goal_near` | service | `rpc` | [`semantic_map/srv/GoalNear.srv`](idl.md#semantic-map-srv-goalnear-srv) | `system/scene/goal_near.v1.toml` |
| `robonix/system/scene/list_objects` | service | `rpc` | [`semantic_map/srv/ListObjects.srv`](idl.md#semantic-map-srv-listobjects-srv) | `system/scene/list_objects.v1.toml` |
| `robonix/system/scene/list_relations` | service | `rpc` | [`semantic_map/srv/ListRelations.srv`](idl.md#semantic-map-srv-listrelations-srv) | `system/scene/list_relations.v1.toml` |
| `robonix/system/soma/description` | service | `rpc` | [`soma/srv/GetDescription.srv`](idl.md#soma-srv-getdescription-srv) | `system/soma/description.v1.toml` |
| `robonix/system/soma/footprint` | service | `rpc` | [`soma/srv/GetFootprint.srv`](idl.md#soma-srv-getfootprint-srv) | `system/soma/footprint.v1.toml` |
| `robonix/system/soma/sensor_extrinsics` | service | `rpc` | [`soma/srv/GetSensorExtrinsics.srv`](idl.md#soma-srv-getsensorextrinsics-srv) | `system/soma/sensor_extrinsics.v1.toml` |
