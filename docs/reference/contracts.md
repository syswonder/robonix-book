# 能力约定参考（自动生成）

> 由 `rbnx docs` 自动生成，请勿手改。

本页罗列 `capabilities/` 下的所有标准能力约定（共 98 条）。
载荷列链到对应的 [ROS IDL](idl.md)。概念与字段含义见 [接口目录](../interface-catalog/index.md)。

## primitive

| 能力约定 ID | 接口含义 | kind | mode | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|---|
| `robonix/primitive/arm/driver` | Legacy-compatible lifecycle interface for arm primitive providers. | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/arm/driver.v1.toml` |
| `robonix/primitive/arm/end_pose` | Measured end-effector pose in the arm provider's documented base frame. | primitive | `topic_out` | [`common_interfaces/geometry_msgs/msg/Pose.msg`](idl.md#common-interfaces-geometry-msgs-msg-pose-msg) | `primitive/arm/end_pose.v1.toml` |
| `robonix/primitive/arm/joint_command` | Named arm and gripper joint command input using standard JointState units. | primitive | `topic_in` | [`common_interfaces/sensor_msgs/msg/JointState.msg`](idl.md#common-interfaces-sensor-msgs-msg-jointstate-msg) | `primitive/arm/joint_command.v1.toml` |
| `robonix/primitive/arm/joint_states` | Measured arm and gripper joint positions, velocities, and efforts. | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/JointState.msg`](idl.md#common-interfaces-sensor-msgs-msg-jointstate-msg) | `primitive/arm/joint_states.v1.toml` |
| `robonix/primitive/arm/pos_command` | Target end-effector pose input in the arm provider's documented base frame. | primitive | `topic_in` | [`common_interfaces/geometry_msgs/msg/Pose.msg`](idl.md#common-interfaces-geometry-msgs-msg-pose-msg) | `primitive/arm/pos_command.v1.toml` |
| `robonix/primitive/audio/bridge_info` | Discover the reverse client-audio endpoint exposed by an audio bridge provider. | primitive | `rpc` | [`audio/srv/GetAudioBridgeInfo.srv`](idl.md#audio-srv-getaudiobridgeinfo-srv) | `primitive/audio/bridge_info.v1.toml` |
| `robonix/primitive/audio/driver` | Legacy-compatible lifecycle interface for audio primitive providers. | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/audio/driver.v1.toml` |
| `robonix/primitive/audio/list_devices` | List audio input and output devices visible to the audio primitive. | primitive | `rpc` | [`audio/srv/ListAudioDevices.srv`](idl.md#audio-srv-listaudiodevices-srv) | `primitive/audio/list_devices.v1.toml` |
| `robonix/primitive/audio/mic` | Continuous microphone audio stream produced by an audio input primitive. | primitive | `topic_out` | [`audio/msg/AudioChunk.msg`](idl.md#audio-msg-audiochunk-msg) | `primitive/audio/mic.v1.toml` |
| `robonix/primitive/audio/select_device` | Select the active audio input or output device used by an audio primitive. | primitive | `rpc` | [`audio/srv/SelectAudioDevice.srv`](idl.md#audio-srv-selectaudiodevice-srv) | `primitive/audio/select_device.v1.toml` |
| `robonix/primitive/audio/speaker` | Audio playback stream consumed by an audio output primitive. | primitive | `topic_in` | [`audio/msg/AudioChunk.msg`](idl.md#audio-msg-audiochunk-msg) | `primitive/audio/speaker.v1.toml` |
| `robonix/primitive/camera/depth` | Continuous depth image stream from a camera primitive. | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/Image.msg`](idl.md#common-interfaces-sensor-msgs-msg-image-msg) | `primitive/camera/depth.v1.toml` |
| `robonix/primitive/camera/depth_snapshot` | Capture one depth image frame on demand. | primitive | `rpc` | [`camera/srv/GetCameraImage.srv`](idl.md#camera-srv-getcameraimage-srv) | `primitive/camera/depth_snapshot.v1.toml` |
| `robonix/primitive/camera/driver` | Legacy-compatible lifecycle interface for camera primitive providers. | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/camera/driver.v1.toml` |
| `robonix/primitive/camera/extrinsics` | Compatibility static transform from the robot body frame to the camera optical frame when TF is unavailable. | primitive | `topic_out` | [`common_interfaces/geometry_msgs/msg/TransformStamped.msg`](idl.md#common-interfaces-geometry-msgs-msg-transformstamped-msg) | `primitive/camera/extrinsics.v1.toml` |
| `robonix/primitive/camera/intrinsics` | Camera calibration and image geometry used for projection and 3D reconstruction. | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/CameraInfo.msg`](idl.md#common-interfaces-sensor-msgs-msg-camerainfo-msg) | `primitive/camera/intrinsics.v1.toml` |
| `robonix/primitive/camera/rgb` | Continuous RGB image stream from a camera primitive. | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/Image.msg`](idl.md#common-interfaces-sensor-msgs-msg-image-msg) | `primitive/camera/rgb.v1.toml` |
| `robonix/primitive/camera/snapshot` | Capture one RGB image frame on demand. | primitive | `rpc` | [`camera/srv/GetCameraImage.srv`](idl.md#camera-srv-getcameraimage-srv) | `primitive/camera/snapshot.v1.toml` |
| `robonix/primitive/chassis/driver` | Legacy-compatible lifecycle interface for chassis primitive providers. | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/chassis/driver.v1.toml` |
| `robonix/primitive/chassis/move` | Low-level bounded chassis motion command without global path planning. | primitive | `rpc` | [`chassis/srv/ExecuteMoveCommand.srv`](idl.md#chassis-srv-executemovecommand-srv) | `primitive/chassis/move.v1.toml` |
| `robonix/primitive/chassis/odom` | Raw chassis odometry in the local odom frame. | primitive | `topic_out` | [`common_interfaces/nav_msgs/msg/Odometry.msg`](idl.md#common-interfaces-nav-msgs-msg-odometry-msg) | `primitive/chassis/odom.v1.toml` |
| `robonix/primitive/chassis/twist_in` | Velocity command input consumed by a chassis controller. | primitive | `topic_in` | [`common_interfaces/geometry_msgs/msg/Twist.msg`](idl.md#common-interfaces-geometry-msgs-msg-twist-msg) | `primitive/chassis/twist_in.v1.toml` |
| `robonix/primitive/health/driver` | Legacy-compatible lifecycle interface for health primitive providers. | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/health/driver.v1.toml` |
| `robonix/primitive/health/state` | - | primitive | `rpc` | [`health/srv/GetHealthState.srv`](idl.md#health-srv-gethealthstate-srv) | `primitive/health/state.v1.toml` |
| `robonix/primitive/health/stream` | - | primitive | `rpc_server_stream` | [`health/srv/StreamHealthState.srv`](idl.md#health-srv-streamhealthstate-srv) | `primitive/health/stream.v1.toml` |
| `robonix/primitive/imu/driver` | Legacy-compatible lifecycle interface for IMU primitive providers. | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/imu/driver.v1.toml` |
| `robonix/primitive/imu/imu` | Continuous inertial measurement stream from an IMU primitive. | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/Imu.msg`](idl.md#common-interfaces-sensor-msgs-msg-imu-msg) | `primitive/imu/imu.v1.toml` |
| `robonix/primitive/lidar/driver` | Legacy-compatible lifecycle interface for lidar primitive providers. | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/lidar/driver.v1.toml` |
| `robonix/primitive/lidar/lidar` | Continuous 2D laser scan stream from a lidar primitive. | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/LaserScan.msg`](idl.md#common-interfaces-sensor-msgs-msg-laserscan-msg) | `primitive/lidar/lidar.v1.toml` |
| `robonix/primitive/lidar/lidar3d` | Continuous 3D point cloud stream from a lidar primitive. | primitive | `topic_out` | [`common_interfaces/sensor_msgs/msg/PointCloud2.msg`](idl.md#common-interfaces-sensor-msgs-msg-pointcloud2-msg) | `primitive/lidar/lidar3d.v1.toml` |
| `robonix/primitive/lidar/snapshot` | Capture one 2D laser scan on demand. | primitive | `rpc` | [`lidar/srv/GetLaserScan.srv`](idl.md#lidar-srv-getlaserscan-srv) | `primitive/lidar/lidar_snapshot.v1.toml` |
| `robonix/primitive/robot_description/driver` | Legacy-compatible lifecycle interface for robot-description primitive providers. | primitive | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `primitive/robot_description/driver.v1.toml` |

## service

| 能力约定 ID | 接口含义 | kind | mode | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|---|
| `robonix/service/map/delete_map` | Delete a saved map and its persisted artifacts by map id. | service | `rpc` | [`map/srv/DeleteMap.srv`](idl.md#map-srv-deletemap-srv) | `service/map/delete_map.v1.toml` |
| `robonix/service/map/driver` | Legacy-compatible lifecycle interface for map service providers. | service | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `service/map/driver.v1.toml` |
| `robonix/service/map/get_mode` | Read whether the map service is currently mapping or localizing. | service | `rpc` | [`map/srv/GetMode.srv`](idl.md#map-srv-getmode-srv) | `service/map/get_mode.v1.toml` |
| `robonix/service/map/get_pose` | Read the robot pose in the map frame as scalar x, y, and yaw. | service | `rpc` | [`map/srv/GetPose.srv`](idl.md#map-srv-getpose-srv) | `service/map/get_pose.v1.toml` |
| `robonix/service/map/lifecycle` | Latched map identity and lifecycle events (`map_id`, `mode`, `generation`) from the mapping service. | service | `topic_out` | [`map/msg/MapLifecycle.msg`](idl.md#map-msg-maplifecycle-msg) | `service/map/lifecycle.v1.toml` |
| `robonix/service/map/list_maps` | List saved maps and metadata available for load/delete. | service | `rpc` | [`map/srv/ListMaps.srv`](idl.md#map-srv-listmaps-srv) | `service/map/list_maps.v1.toml` |
| `robonix/service/map/load_map` | Load a saved map for localization or continued mapping. | service | `rpc` | [`map/srv/LoadMap.srv`](idl.md#map-srv-loadmap-srv) | `service/map/load_map.v1.toml` |
| `robonix/service/map/occupancy_grid` | 2D occupancy grid output produced by the mapping service. | service | `topic_out` | [`common_interfaces/nav_msgs/msg/OccupancyGrid.msg`](idl.md#common-interfaces-nav-msgs-msg-occupancygrid-msg) | `service/map/occupancy_grid.v1.toml` |
| `robonix/service/map/odom` | Continuous SLAM or localization odometry suitable for controllers. | service | `topic_out` | [`common_interfaces/nav_msgs/msg/Odometry.msg`](idl.md#common-interfaces-nav-msgs-msg-odometry-msg) | `service/map/odom.v1.toml` |
| `robonix/service/map/pointcloud` | Registered world-frame point cloud output from the mapping service. | service | `topic_out` | [`common_interfaces/sensor_msgs/msg/PointCloud2.msg`](idl.md#common-interfaces-sensor-msgs-msg-pointcloud2-msg) | `service/map/pointcloud.v1.toml` |
| `robonix/service/map/pose` | Canonical map-frame robot pose produced by localization or SLAM. | service | `topic_out` | [`common_interfaces/geometry_msgs/msg/PoseWithCovarianceStamped.msg`](idl.md#common-interfaces-geometry-msgs-msg-posewithcovariancestamped-msg) | `service/map/pose.v1.toml` |
| `robonix/service/map/pose_estimate` | Seed or reset localization with a map-frame pose estimate. | service | `rpc` | [`map/srv/PoseEstimate.srv`](idl.md#map-srv-poseestimate-srv) | `service/map/pose_estimate.v1.toml` |
| `robonix/service/map/reset_map` | Clear the live map state without deleting persisted maps. | service | `rpc` | [`map/srv/ResetMap.srv`](idl.md#map-srv-resetmap-srv) | `service/map/reset_map.v1.toml` |
| `robonix/service/map/save_map` | Persist the current map under a stable map id. | service | `rpc` | [`map/srv/SaveMap.srv`](idl.md#map-srv-savemap-srv) | `service/map/save_map.v1.toml` |
| `robonix/service/map/switch_mode` | Switch the running map service between mapping and localization modes. | service | `rpc` | [`map/srv/SwitchMode.srv`](idl.md#map-srv-switchmode-srv) | `service/map/switch_mode.v1.toml` |
| `robonix/service/memory/compact` | Compact long-term memory into a shorter summary. | service | `rpc` | [`memory/srv/Compact.srv`](idl.md#memory-srv-compact-srv) | `service/memory/compact.v1.toml` |
| `robonix/service/memory/driver` | Legacy-compatible lifecycle interface for memory service providers. | service | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `service/memory/driver.v1.toml` |
| `robonix/service/memory/save` | Persist a fact, preference, or note into long-term memory. | service | `rpc` | [`memory/srv/Save.srv`](idl.md#memory-srv-save-srv) | `service/memory/save.v1.toml` |
| `robonix/service/memory/search` | Search long-term memory for entries relevant to a query. | service | `rpc` | [`memory/srv/Search.srv`](idl.md#memory-srv-search-srv) | `service/memory/search.v1.toml` |
| `robonix/service/navigation/driver` | Legacy-compatible lifecycle interface for navigation service providers. | service | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `service/navigation/driver.v1.toml` |
| `robonix/service/navigation/navigate` | Navigate the robot to a target pose using planning and obstacle avoidance. | service | `rpc` | [`navigation/srv/Navigate.srv`](idl.md#navigation-srv-navigate-srv) | `service/navigation/navigate.v1.toml` |
| `robonix/service/navigation/navigate/cancel` | Cancel a previously accepted navigation run. | service | `rpc` | [`navigation/srv/CancelNavigation.srv`](idl.md#navigation-srv-cancelnavigation-srv) | `service/navigation/navigate/cancel.v1.toml` |
| `robonix/service/navigation/navigate/status` | Read status for a previously accepted navigation run. | service | `rpc` | [`navigation/srv/GetNavigationStatus.srv`](idl.md#navigation-srv-getnavigationstatus-srv) | `service/navigation/navigate/status.v1.toml` |
| `robonix/service/speech/asr` | Recognize speech from one audio buffer. | service | `rpc` | [`asr/srv/Recognize.srv`](idl.md#asr-srv-recognize-srv) | `service/speech/asr.v1.toml` |
| `robonix/service/speech/asr_stream` | Streaming speech recognition over an audio chunk stream. | service | `rpc_bidirectional_stream` | [`asr/srv/RecognizeStream.srv`](idl.md#asr-srv-recognizestream-srv) | `service/speech/asr_stream.v1.toml` |
| `robonix/service/speech/dialog` | Run a voice dialog session that coordinates speech input and output. | service | `rpc_server_stream` | [`speech/srv/StartDialog.srv`](idl.md#speech-srv-startdialog-srv) | `service/speech/dialog.v1.toml` |
| `robonix/service/speech/driver` | Legacy-compatible lifecycle interface for speech service providers. | service | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `service/speech/driver.v1.toml` |
| `robonix/service/speech/list_speakers` | List speaker playback targets available for speech output. | service | `rpc` | [`speech/srv/ListSpeakers.srv`](idl.md#speech-srv-listspeakers-srv) | `service/speech/list_speakers.v1.toml` |
| `robonix/service/speech/speak` | Speak text aloud through a selected audio output target. | service | `rpc` | [`speech/srv/Speak.srv`](idl.md#speech-srv-speak-srv) | `service/speech/speak.v1.toml` |
| `robonix/service/speech/tts` | Synthesize one text input into audio. | service | `rpc` | [`tts/srv/Synthesize.srv`](idl.md#tts-srv-synthesize-srv) | `service/speech/tts.v1.toml` |
| `robonix/service/speech/tts_stream` | Streaming text-to-speech synthesis that returns audio chunks. | service | `rpc_server_stream` | [`tts/srv/SynthesizeStream.srv`](idl.md#tts-srv-synthesizestream-srv) | `service/speech/tts_stream.v1.toml` |
| `robonix/service/speech/wake_word` | Detect a configured wake phrase from a client-streamed PCM audio source. | service | `rpc_client_stream` | [`speech/srv/DetectWakeWord.srv`](idl.md#speech-srv-detectwakeword-srv) | `service/speech/wake_word.v1.toml` |
| `robonix/service/voiceprint/delete` | Delete an enrolled speaker identity from the voiceprint database. | service | `rpc` | [`voiceprint/srv/DeleteEnrolled.srv`](idl.md#voiceprint-srv-deleteenrolled-srv) | `service/voiceprint/delete.v1.toml` |
| `robonix/service/voiceprint/driver` | Legacy-compatible lifecycle and configuration interface for Voiceprint providers. | service | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `service/voiceprint/driver.v1.toml` |
| `robonix/service/voiceprint/enroll` | Enroll a speaker identity from voice samples. | service | `rpc` | [`voiceprint/srv/Enroll.srv`](idl.md#voiceprint-srv-enroll-srv) | `service/voiceprint/enroll.v1.toml` |
| `robonix/service/voiceprint/identify` | Identify whether an audio sample matches an enrolled speaker. | service | `rpc` | [`voiceprint/srv/Identify.srv`](idl.md#voiceprint-srv-identify-srv) | `service/voiceprint/identify.v1.toml` |
| `robonix/service/voiceprint/list` | List enrolled speaker identities known to the voiceprint service. | service | `rpc` | [`voiceprint/srv/ListEnrolled.srv`](idl.md#voiceprint-srv-listenrolled-srv) | `service/voiceprint/list.v1.toml` |

## system

| 能力约定 ID | 接口含义 | kind | mode | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|---|
| `robonix/system/executor/cancel_all_plans` | Cancel all currently running executor plans. | service | `rpc` | [`executor/srv/CancelAll.srv`](idl.md#executor-srv-cancelall-srv) | `system/executor/cancel_all_plans.v1.toml` |
| `robonix/system/executor/control_plan` | Apply an out-of-band RTDL meta operation without creating another plan. | service | `rpc` | [`executor/srv/ControlPlan.srv`](idl.md#executor-srv-controlplan-srv) | `system/executor/control_plan.v1.toml` |
| `robonix/system/executor/execute` | Execute an RTDL plan and stream execution events. | service | `rpc_server_stream` | [`executor/srv/Execute.srv`](idl.md#executor-srv-execute-srv) | `system/executor/execute.v1.toml` |
| `robonix/system/executor/get_health` | Return the current health report of the Executor module. | service | `rpc` | [`module_health/srv/GetModuleHealth.srv`](idl.md#module-health-srv-getmodulehealth-srv) | `system/executor/get_health.toml` |
| `robonix/system/executor/list_active_plans` | Read the authoritative set of RTDL plans currently owned by Executor without creating a new plan. | service | `rpc` | [`executor/srv/ListActivePlans.srv`](idl.md#executor-srv-listactiveplans-srv) | `system/executor/list_active_plans.v1.toml` |
| `robonix/system/liaison/handsfree/events` | Subscribe to events from Liaison's robot-local hands-free voice turns. | service | `rpc_server_stream` | [`liaison/srv/WatchHandsfreeEvents.srv`](idl.md#liaison-srv-watchhandsfreeevents-srv) | `system/liaison/handsfree/events.v1.toml` |
| `robonix/system/liaison/handsfree/set_enabled` | Enable or disable Liaison's robot-local wake-word interaction mode. | service | `rpc` | [`liaison/srv/SetHandsfree.srv`](idl.md#liaison-srv-sethandsfree-srv) | `system/liaison/handsfree/set_enabled.v1.toml` |
| `robonix/system/liaison/handsfree/status` | Read Liaison's robot-local wake-word interaction state. | service | `rpc` | [`liaison/srv/GetHandsfreeStatus.srv`](idl.md#liaison-srv-gethandsfreestatus-srv) | `system/liaison/handsfree/status.v1.toml` |
| `robonix/system/liaison/submit` | Submit a user task through Liaison and stream Pilot events back. | service | `rpc_server_stream` | [`liaison/srv/SubmitTask.srv`](idl.md#liaison-srv-submittask-srv) | `system/liaison/submit.v1.toml` |
| `robonix/system/liaison/voice` | Run a voice interaction session through Liaison. | service | `rpc_server_stream` | [`liaison/srv/StartVoiceSession.srv`](idl.md#liaison-srv-startvoicesession-srv) | `system/liaison/voice.v1.toml` |
| `robonix/system/pilot` | Submit a task to Pilot and stream planning events. | service | `rpc_server_stream` | [`pilot/srv/SubmitTask.srv`](idl.md#pilot-srv-submittask-srv) | `system/pilot.v1.toml` |
| `robonix/system/pilot/get_health` | Return the current health report of the Pilot module. | service | `rpc` | [`module_health/srv/GetModuleHealth.srv`](idl.md#module-health-srv-getmodulehealth-srv) | `system/pilot/get_health.toml` |
| `robonix/system/scene/get_object_context` | Return one scene object's node, relations, and nearby context. | service | `rpc` | [`semantic_map/srv/GetObjectContext.srv`](idl.md#semantic-map-srv-getobjectcontext-srv) | `system/scene/get_object_context.v1.toml` |
| `robonix/system/scene/get_robot_context` | Return the robot pose, containing room and areas, and nearby objects as one current Scene snapshot. | service | `rpc` | [`semantic_map/srv/GetRobotContext.srv`](idl.md#semantic-map-srv-getrobotcontext-srv) | `system/scene/get_robot_context.v1.toml` |
| `robonix/system/scene/get_scene_graph` | Return the current scene graph snapshot. | service | `rpc` | [`semantic_map/srv/GetSceneGraph.srv`](idl.md#semantic-map-srv-getscenegraph-srv) | `system/scene/get_scene_graph.v1.toml` |
| `robonix/system/scene/goal_near` | Compute a navigation-safe approach pose near a known physical scene object. Room annotations are not accepted; use robonix/system/scene/goal_room for rooms or named regions. | service | `rpc` | [`semantic_map/srv/GoalNear.srv`](idl.md#semantic-map-srv-goalnear-srv) | `system/scene/goal_near.v1.toml` |
| `robonix/system/scene/goal_room` | Resolve a room annotation to a navigation-safe pose inside its polygon. Use this for named rooms or regions before navigation/navigate. | service | `rpc` | [`semantic_map/srv/GoalRoom.srv`](idl.md#semantic-map-srv-goalroom-srv) | `system/scene/goal_room.v1.toml` |
| `robonix/system/scene/list_objects` | List all objects currently known to the scene registry. | service | `rpc` | [`semantic_map/srv/ListObjects.srv`](idl.md#semantic-map-srv-listobjects-srv) | `system/scene/list_objects.v1.toml` |
| `robonix/system/scene/list_relations` | List scene graph relations, optionally filtered by relation type. | service | `rpc` | [`semantic_map/srv/ListRelations.srv`](idl.md#semantic-map-srv-listrelations-srv) | `system/scene/list_relations.v1.toml` |
| `robonix/system/soma/description` | Return the robot body description, including URDF and high-level metadata. | service | `rpc` | [`soma/srv/GetDescription.srv`](idl.md#soma-srv-getdescription-srv) | `system/soma/description.v1.toml` |
| `robonix/system/soma/footprint` | Return the robot 2D collision footprint in the base frame. | service | `rpc` | [`soma/srv/GetFootprint.srv`](idl.md#soma-srv-getfootprint-srv) | `system/soma/footprint.v1.toml` |
| `robonix/system/soma/get_health` | - | service | `rpc` | [`soma/srv/GetHealth.srv`](idl.md#soma-srv-gethealth-srv) | `system/soma/get_health.v1.toml` |
| `robonix/system/soma/get_urdf` | - | service | `rpc` | [`soma/srv/GetUrdf.srv`](idl.md#soma-srv-geturdf-srv) | `system/soma/get_urdf.v1.toml` |
| `robonix/system/soma/get_yaml` | - | service | `rpc` | [`soma/srv/GetYaml.srv`](idl.md#soma-srv-getyaml-srv) | `system/soma/get_yaml.v1.toml` |
| `robonix/system/soma/health` | - | service | `rpc_server_stream` | [`soma/srv/StreamHealth.srv`](idl.md#soma-srv-streamhealth-srv) | `system/soma/health.v1.toml` |
| `robonix/system/vitals/get` | - | service | `rpc` | [`vitals/srv/GetVitals.srv`](idl.md#vitals-srv-getvitals-srv) | `system/vitals/get.v1.toml` |
| `robonix/system/vitals/modules/get` | Return the current aggregated module health snapshot from Vitals. | service | `rpc` | [`module_health/srv/GetModuleHealthSnapshot.srv`](idl.md#module-health-srv-getmodulehealthsnapshot-srv) | `system/vitals/modules/get.toml` |
| `robonix/system/vitals/stream` | - | service | `rpc_server_stream` | [`vitals/srv/StreamVitals.srv`](idl.md#vitals-srv-streamvitals-srv) | `system/vitals/stream.v1.toml` |

## other

| 能力约定 ID | 接口含义 | kind | mode | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|---|
| `robonix/lifecycle/driver` | Initialize, activate, deactivate, or shut down a managed provider. | service | `rpc` | [`lifecycle/srv/Driver.srv`](idl.md#lifecycle-srv-driver-srv) | `lifecycle/driver.v1.toml` |
