---
hide_table_of_contents: true
---

# ROS IDL 参考（自动生成）

> 由 `rbnx docs` 自动生成，请勿手改。

本页收录从 IDL 包含根（`rbnx docs --include`，默认 `capabilities/lib/`）收集的全部 ROS IDL（`.msg` / `.srv`）原文，按 ROS 包分组（共 304 个文件）。[能力约定参考](contracts.md) 的载荷列链到这里对应的锚点。

<details className="idl-package-index">
<summary>展开 ROS package 索引</summary>

[action_msgs](#action_msgs) · [actionlib_msgs](#actionlib_msgs) · [asr](#asr) · [audio](#audio) · [builtin_interfaces](#builtin_interfaces) · [camera](#camera) · [chassis](#chassis) · [composition_interfaces](#composition_interfaces) · [diagnostic_msgs](#diagnostic_msgs) · [executor](#executor) · [geometry_msgs](#geometry_msgs) · [health](#health) · [liaison](#liaison) · [lidar](#lidar) · [lifecycle](#lifecycle) · [lifecycle_msgs](#lifecycle_msgs) · [map](#map) · [memory](#memory) · [module_health](#module_health) · [nav_msgs](#nav_msgs) · [navigation](#navigation) · [perception](#perception) · [pilot](#pilot) · [rcl_interfaces](#rcl_interfaces) · [rosgraph_msgs](#rosgraph_msgs) · [semantic_map](#semantic_map) · [sensor_msgs](#sensor_msgs) · [shape_msgs](#shape_msgs) · [soma](#soma) · [speech](#speech) · [statistics_msgs](#statistics_msgs) · [std_msgs](#std_msgs) · [std_srvs](#std_srvs) · [stereo_msgs](#stereo_msgs) · [test_msgs](#test_msgs) · [trajectory_msgs](#trajectory_msgs) · [tts](#tts) · [unique_identifier_msgs](#unique_identifier_msgs) · [visualization_msgs](#visualization_msgs) · [vitals](#vitals) · [voiceprint](#voiceprint)

</details>

## action_msgs

### CancelGoal `srv` {/* #rcl-interfaces-action-msgs-srv-cancelgoal-srv */}

`rcl_interfaces/action_msgs/srv/CancelGoal.srv`

```rosidl
# Cancel one or more goals with the following policy:
#
# - If the goal ID is zero and timestamp is zero, cancel all goals.
# - If the goal ID is zero and timestamp is not zero, cancel all goals accepted
#   at or before the timestamp.
# - If the goal ID is not zero and timestamp is zero, cancel the goal with the
#   given ID regardless of the time it was accepted.
# - If the goal ID is not zero and timestamp is not zero, cancel the goal with
#   the given ID and all goals accepted at or before the timestamp.

# Goal info describing the goals to cancel, see above.
GoalInfo goal_info
---
##
## Return codes.
##

# Indicates the request was accepted without any errors.
#
# One or more goals have transitioned to the CANCELING state. The
# goals_canceling list is not empty.
int8 ERROR_NONE=0

# Indicates the request was rejected.
#
# No goals have transitioned to the CANCELING state. The goals_canceling list is
# empty.
int8 ERROR_REJECTED=1

# Indicates the requested goal ID does not exist.
#
# No goals have transitioned to the CANCELING state. The goals_canceling list is
# empty.
int8 ERROR_UNKNOWN_GOAL_ID=2

# Indicates the goal is not cancelable because it is already in a terminal state.
#
# No goals have transitioned to the CANCELING state. The goals_canceling list is
# empty.
int8 ERROR_GOAL_TERMINATED=3

# Return code, see above definitions.
int8 return_code

# Goals that accepted the cancel request.
GoalInfo[] goals_canceling
```

### GoalInfo `msg` {/* #rcl-interfaces-action-msgs-msg-goalinfo-msg */}

`rcl_interfaces/action_msgs/msg/GoalInfo.msg`

```rosidl
# Goal ID
unique_identifier_msgs/UUID goal_id

# Time when the goal was accepted
builtin_interfaces/Time stamp
```

### GoalStatus `msg` {/* #rcl-interfaces-action-msgs-msg-goalstatus-msg */}

`rcl_interfaces/action_msgs/msg/GoalStatus.msg`

```rosidl
# An action goal can be in one of these states after it is accepted by an action
# server.
#
# For more information, see http://design.ros2.org/articles/actions.html

# Indicates status has not been properly set.
int8 STATUS_UNKNOWN   = 0

# The goal has been accepted and is awaiting execution.
int8 STATUS_ACCEPTED  = 1

# The goal is currently being executed by the action server.
int8 STATUS_EXECUTING = 2

# The client has requested that the goal be canceled and the action server has
# accepted the cancel request.
int8 STATUS_CANCELING = 3

# The goal was achieved successfully by the action server.
int8 STATUS_SUCCEEDED = 4

# The goal was canceled after an external request from an action client.
int8 STATUS_CANCELED  = 5

# The goal was terminated by the action server without an external request.
int8 STATUS_ABORTED   = 6

# Goal info (contains ID and timestamp).
GoalInfo goal_info

# Action goal state-machine status.
int8 status
```

### GoalStatusArray `msg` {/* #rcl-interfaces-action-msgs-msg-goalstatusarray-msg */}

`rcl_interfaces/action_msgs/msg/GoalStatusArray.msg`

```rosidl
# An array of goal statuses.
GoalStatus[] status_list
```

## actionlib_msgs

### GoalID `msg` {/* #common-interfaces-actionlib-msgs-msg-goalid-msg */}

`common_interfaces/actionlib_msgs/msg/GoalID.msg`

```rosidl

# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
builtin_interfaces/Time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
```

### GoalStatus `msg` {/* #common-interfaces-actionlib-msgs-msg-goalstatus-msg */}

`common_interfaces/actionlib_msgs/msg/GoalStatus.msg`

```rosidl
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server.
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server.
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State).
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server
                            #   (Terminal State).
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State).
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State).
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution.
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing, but
                            #    the action server has not yet confirmed that the goal is canceled.
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State).
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not
                            #    be sent over the wire by an action server.

# Allow for the user to associate a string with GoalStatus for debugging.
string text
```

### GoalStatusArray `msg` {/* #common-interfaces-actionlib-msgs-msg-goalstatusarray-msg */}

`common_interfaces/actionlib_msgs/msg/GoalStatusArray.msg`

```rosidl
# Stores the statuses for goals that are currently being tracked
# by an action server
std_msgs/Header header
GoalStatus[] status_list
```

## asr

### AsrAudioChunk `msg` {/* #asr-msg-asraudiochunk-msg */}

`asr/msg/AsrAudioChunk.msg`

```rosidl
# Audio chunk sent by client during streaming ASR recognition.
audio/msg/AudioChunk chunk
```

### Recognize `srv` {/* #asr-srv-recognize-srv */}

`asr/srv/Recognize.srv`

```rosidl
# ASR one-shot recognition
# Contract: robonix/service/speech/asr
# Caller sends raw audio bytes; service returns a transcript string.
uint8[] audio_data
string encoding       # e.g. "pcm_s16le", "opus", "mp3"
uint32 sample_rate_hz # e.g. 16000
string language       # BCP-47, e.g. "zh-CN", "en-US", "" = auto-detect
---
string text           # recognised transcript (empty if silent / error)
float32 confidence    # 0.0 – 1.0; 0.0 if unavailable
string error          # non-empty on failure
```

### RecognizeStream `srv` {/* #asr-srv-recognizestream-srv */}

`asr/srv/RecognizeStream.srv`

```rosidl
# ASR streaming recognition (bidirectional stream).
# Client streams AsrAudioChunk; server streams RecognizeStreamEvent.
# Request/Response sections are the per-message stream element types.
asr/AsrAudioChunk chunk
---
asr/RecognizeStreamEvent event
```

### RecognizeStreamEvent `msg` {/* #asr-msg-recognizestreamevent-msg */}

`asr/msg/RecognizeStreamEvent.msg`

```rosidl
# Event streamed back by the ASR service during streaming recognition.

uint8 PARTIAL = 0
uint8 FINAL   = 1
uint8 ERROR   = 2
uint8 event_type

string text             # recognised text so far (partial or final)
float32 confidence      # 0.0 – 1.0
string language         # detected language (BCP-47)
bool is_final           # true when this is the final recognition result
string error            # non-empty on error
```

## audio

### AudioChunk `msg` {/* #audio-msg-audiochunk-msg */}

`audio/msg/AudioChunk.msg`

```rosidl
# A chunk of raw or encoded audio data.
# Used as the stream element for mic topic_out, speaker topic_in,
# ASR streaming, and TTS streaming.

uint64 timestamp_ns       # nanoseconds since epoch (replaces std_msgs/Header for portability)
uint8[] data              # raw or encoded audio bytes
uint32 sequence           # monotonically increasing within a session
float32 duration_s        # duration of this chunk in seconds (0 if unknown)
```

### AudioConfig `msg` {/* #audio-msg-audioconfig-msg */}

`audio/msg/AudioConfig.msg`

```rosidl
# Audio configuration descriptor for speech services.
# Shared across ASR, TTS, mic, and speaker primitives.

string encoding           # e.g. "pcm_s16le", "pcm_f32le", "opus", "mp3"
uint32 sample_rate_hz     # e.g. 16000, 22050, 44100
uint32 channels           # 1 = mono, 2 = stereo
uint32 bits_per_sample    # e.g. 16 for pcm_s16le
```

### AudioDevice `msg` {/* #audio-msg-audiodevice-msg */}

`audio/msg/AudioDevice.msg`

```rosidl
# One OS-level audio device that a multi-device audio cap's driver can
# route to internally. NOT a robonix cap — robonix caps stay 1:1 with
# packages. This message describes the driver's internal routing table:
# `com.robonix.primitive.audio.alsa` (one cap) drives many ALSA "hw:X,Y"
# devices, this is one row of that table.
#
# Each impl owns its own id space — sounddevice uses integer indices,
# ALSA uses "hw:0,0", PulseAudio uses sink names, etc. The id string
# round-trips back via SelectAudioDevice; only the impl needs to know
# what it means.

string id                 # impl-specific stable identifier
string name               # human-readable name (shown in pickers)
string kind               # "input" | "output" | "duplex"
bool is_default           # true if this is the OS-level default for its kind
uint32 channels           # max channels at the impl's preferred rate
string note               # free-form hint: "bluetooth" / "usb" / "airpods"
                          # — pickers can warn or auto-skip on these
```

### GetAudioBridgeInfo `srv` {/* #audio-srv-getaudiobridgeinfo-srv */}

`audio/srv/GetAudioBridgeInfo.srv`

```rosidl
# robonix/primitive/audio/bridge_info -- return the reverse-audio listener
# that a robonix-client should connect to. Only the client-bridge provider
# exposes this optional contract; fixed local audio drivers do not need it.
---
bool reverse
string endpoint          # ws://<robot-reachable-host>:<port>/client, or empty
bool connected           # whether a client currently has an active session
string detail
```

### ListAudioDevices `srv` {/* #audio-srv-listaudiodevices-srv */}

`audio/srv/ListAudioDevices.srv`

```rosidl
# robonix/primitive/audio/list_devices — enumerate every input + output
# device the audio impl can drive. Implementations that wrap a single
# fixed device (e.g. an embedded board with one mic and one speaker)
# may return UNIMPLEMENTED; consumers must treat that as "the primitive
# id is the device".
---
audio/msg/AudioDevice[] devices
string current_input_id     # id currently in use for input streams,
                            # "" if the OS default is in effect
string current_output_id    # id currently in use for output streams,
                            # "" if the OS default is in effect
```

### SelectAudioDevice `srv` {/* #audio-srv-selectaudiodevice-srv */}

`audio/srv/SelectAudioDevice.srv`

```rosidl
# robonix/primitive/audio/select_device — pin a specific device for
# subsequent mic / speaker streams from this cap. The impl is free to
# reopen its sounddevice / ALSA / WS handle synchronously or lazily
# (next stream call). Empty `id` resets to the OS default.
#
# Errors:
#   ok=false + error="unknown id"           — id not in ListAudioDevices
#   ok=false + error="device busy"          — currently streaming, can't switch
#   ok=false + error="kind mismatch"        — id is input but kind=output etc.

string kind     # "input" | "output"
string id       # AudioDevice.id from ListAudioDevices, or "" for default
---
bool ok
string error
```

## builtin_interfaces

### Duration `msg` {/* #builtin-interfaces-msg-duration-msg */}

`builtin_interfaces/msg/Duration.msg`

```rosidl
# This message defines a duration (seconds + nanoseconds).
int32 sec
uint32 nanosec
```

### Duration `msg` {/* #rcl-interfaces-builtin-interfaces-msg-duration-msg */}

`rcl_interfaces/builtin_interfaces/msg/Duration.msg`

```rosidl
# Duration defines a period between two time points.
# Messages of this datatype are of ROS Time following this design:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component.
# e.g.
# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
```

### Time `msg` {/* #builtin-interfaces-msg-time-msg */}

`builtin_interfaces/msg/Time.msg`

```rosidl
# This message communicates ROS Time defined in the ROS Time message spec.
int32 sec
uint32 nanosec
```

### Time `msg` {/* #rcl-interfaces-builtin-interfaces-msg-time-msg */}

`rcl_interfaces/builtin_interfaces/msg/Time.msg`

```rosidl
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component.
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
```

## camera

### GetCameraImage `srv` {/* #camera-srv-getcameraimage-srv */}

`camera/srv/GetCameraImage.srv`

```rosidl
# robonix/primitive/camera/snapshot or depth_snapshot — on-demand single frame (RGB or depth encoding).
---
sensor_msgs/Image image
```

### RGBD `msg` {/* #camera-msg-rgbd-msg */}

`camera/msg/RGBD.msg`

```rosidl
# Synchronized RGB-D image pair
# rgb and depth share the same timestamp for alignment
sensor_msgs/Image rgb
sensor_msgs/Image depth
```

## chassis

### ExecuteMoveCommand `srv` {/* #chassis-srv-executemovecommand-srv */}

`chassis/srv/ExecuteMoveCommand.srv`

```rosidl
MoveCommand command
---
std_msgs/String status
```

### MoveCommand `msg` {/* #chassis-msg-movecommand-msg */}

`chassis/msg/MoveCommand.msg`

```rosidl
# Payload for robonix/primitive/chassis/move. Three modes — driver picks
# whichever is non-zero, in priority order:
#
#   1. forward_m  != 0 → drive straight by that signed distance (m).
#                        Driver picks a sensible linear speed and computes
#                        how long to run cmd_vel. positive = forward.
#   2. rotate_deg != 0 → in-place yaw rotation by that signed angle (deg).
#                        Driver picks angular speed; positive = CCW.
#   3. velocity mode  → use linear_*/angular_* fields directly (cmd_vel-
#                        style twist). `duration_sec > 0` overrides the
#                        driver default (TIAGO_CHASSIS_CMD_DURATION_SEC);
#                        `duration_sec == 0` keeps the default.
#
# Modes are exclusive: forward_m takes priority over rotate_deg over the
# velocity fields. Distance / angle modes are the recommended shortcuts
# for an LLM driving the robot — "rotate 360°" or "move forward 1 m" map
# directly without the agent having to guess a velocity-time product.
float64 linear_x
float64 linear_y
float64 linear_z
float64 angular_x
float64 angular_y
float64 angular_z
float64 duration_sec
float64 forward_m
float64 rotate_deg
```

### Stop `srv` {/* #chassis-srv-stop-srv */}

`chassis/srv/Stop.srv`

```rosidl
# Legacy unregistered stop IDL. No contract TOML references this file.
---
bool success
string message
```

## composition_interfaces

### ListNodes `srv` {/* #rcl-interfaces-composition-interfaces-srv-listnodes-srv */}

`rcl_interfaces/composition_interfaces/srv/ListNodes.srv`

```rosidl
---
# List of full node names including namespace.
string[] full_node_names
# corresponding unique ids (must have same length as full_node_names).
uint64[] unique_ids
```

### LoadNode `srv` {/* #rcl-interfaces-composition-interfaces-srv-loadnode-srv */}

`rcl_interfaces/composition_interfaces/srv/LoadNode.srv`

```rosidl
# The ROS package in which the composable node can be found.
string package_name

# A plugin within the ROS package "package_name".
string plugin_name

# The assigned name of the composable node. Leave empty to use the node's
# default name.
string node_name

# The assigned namespace of the composable node. Leave empty to use the node's
# default namespace.
string node_namespace

# The assigned log level of the composable node. Enum values are found in
# message rcl_interfaces/Log.
uint8 log_level

# Remapping rules for this composable node.
#
# For more info about static_remapping rules and their syntax, see
# https://design.ros2.org/articles/static_remapping.html
# TODO(sloretz) rcl_interfaces message for remap rules?
string[] remap_rules

# The Parameters of this composable node to set.
rcl_interfaces/Parameter[] parameters

# key/value arguments that are specific to a type of container process.
rcl_interfaces/Parameter[] extra_arguments
---
# True if the node was successfully loaded.
bool success

# Human readable error message if success is false, else empty string.
string error_message

# Name of the loaded composable node (including namespace).
string full_node_name

# A unique identifier for the loaded node.
uint64 unique_id
```

### UnloadNode `srv` {/* #rcl-interfaces-composition-interfaces-srv-unloadnode-srv */}

`rcl_interfaces/composition_interfaces/srv/UnloadNode.srv`

```rosidl
# Container specific unique id of a loaded node.
uint64 unique_id
---
# True if the node existed and was unloaded.
bool success

# Human readable error message if success is false, else empty string.
string error_message
```

## diagnostic_msgs

### AddDiagnostics `srv` {/* #common-interfaces-diagnostic-msgs-srv-adddiagnostics-srv */}

`common_interfaces/diagnostic_msgs/srv/AddDiagnostics.srv`

```rosidl
# This service is used as part of the process for loading analyzers at runtime,
# and should be used by a loader script or program, not as a standalone service.
# Information about dynamic addition of analyzers can be found at
# http://wiki.ros.org/diagnostics/Tutorials/Adding%20Analyzers%20at%20Runtime

# The load_namespace parameter defines the namespace where parameters for the
# initialization of analyzers in the diagnostic aggregator have been loaded. The
# value should be a global name (i.e. /my/name/space), not a relative
# (my/name/space) or private (~my/name/space) name. Analyzers will not be added
# if a non-global name is used. The call will also fail if the namespace
# contains parameters that follow a namespace structure that does not conform to
# that expected by the analyzer definitions. See
# http://wiki.ros.org/diagnostics/Tutorials/Configuring%20Diagnostic%20Aggregators
# and http://wiki.ros.org/diagnostics/Tutorials/Using%20the%20GenericAnalyzer
# for examples of the structure of yaml files which are expected to have been
# loaded into the namespace.
string load_namespace
---

# True if diagnostic aggregator was updated with new diagnostics, False
# otherwise. A false return value means that either there is a bond in the
# aggregator which already used the requested namespace, or the initialization
# of analyzers failed.
bool success

# Message with additional information about the success or failure
string message
```

### DiagnosticArray `msg` {/* #common-interfaces-diagnostic-msgs-msg-diagnosticarray-msg */}

`common_interfaces/diagnostic_msgs/msg/DiagnosticArray.msg`

```rosidl
# This message is used to send diagnostic information about the state of the robot.
std_msgs/Header header # for timestamp
DiagnosticStatus[] status # an array of components being reported on
```

### DiagnosticStatus `msg` {/* #common-interfaces-diagnostic-msgs-msg-diagnosticstatus-msg */}

`common_interfaces/diagnostic_msgs/msg/DiagnosticStatus.msg`

```rosidl
# This message holds the status of an individual component of the robot.

# Possible levels of operations.
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

# Level of operation enumerated above.
byte level
# A description of the test/component reporting.
string name
# A description of the status.
string message
# A hardware unique string.
string hardware_id
# An array of values associated with the status.
KeyValue[] values

```

### KeyValue `msg` {/* #common-interfaces-diagnostic-msgs-msg-keyvalue-msg */}

`common_interfaces/diagnostic_msgs/msg/KeyValue.msg`

```rosidl
# What to label this value when viewing.
string key
# A value to track over time.
string value
```

### SelfTest `srv` {/* #common-interfaces-diagnostic-msgs-srv-selftest-srv */}

`common_interfaces/diagnostic_msgs/srv/SelfTest.srv`

```rosidl
---
string id
byte passed
DiagnosticStatus[] status
```

## executor

### CancelAll `srv` {/* #executor-srv-cancelall-srv */}

`executor/srv/CancelAll.srv`

```rosidl
---
bool success
```

### CapabilitySpec `msg` {/* #executor-msg-capabilityspec-msg */}

`executor/msg/CapabilitySpec.msg`

```rosidl
# LLM-facing capability description. Pilot builds one of these per
# (provider_id, contract_id) pair it wants to expose to the model.
string provider_id
string contract_id
string description
string input_schema_json
```

### ControlPlan `srv` {/* #executor-srv-controlplan-srv */}

`executor/srv/ControlPlan.srv`

```rosidl
string action
string plan_id
string op_id
string when
uint64 wait_ms
---
bool success
bool completed
string message
string error
```

### Execute `srv` {/* #executor-srv-execute-srv */}

`executor/srv/Execute.srv`

```rosidl
pilot/Plan plan
---
executor/RtdlEvent event
```

### ListActivePlans `srv` {/* #executor-srv-listactiveplans-srv */}

`executor/srv/ListActivePlans.srv`

```rosidl
---
bool success
string plans_json
string error
```

### RtdlEvent `msg` {/* #executor-msg-rtdlevent-msg */}

`executor/msg/RtdlEvent.msg`

```rosidl
uint32 event_kind
uint32 PLAN_STARTED=0
uint32 NODE_STATE=1
uint32 PLAN_COMPLETE=2
RtdlPlanStarted plan_started
pilot/RtdlNodeState node_state
RtdlPlanComplete plan_complete
```

### RtdlPlanComplete `msg` {/* #executor-msg-rtdlplancomplete-msg */}

`executor/msg/RtdlPlanComplete.msg`

```rosidl
# Correlates with Plan.plan_id.
string plan_id
bool any_failed
```

### RtdlPlanStarted `msg` {/* #executor-msg-rtdlplanstarted-msg */}

`executor/msg/RtdlPlanStarted.msg`

```rosidl
string plan_id
```

## geometry_msgs

### Accel `msg` {/* #common-interfaces-geometry-msgs-msg-accel-msg */}

`common_interfaces/geometry_msgs/msg/Accel.msg`

```rosidl
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
```

### AccelStamped `msg` {/* #common-interfaces-geometry-msgs-msg-accelstamped-msg */}

`common_interfaces/geometry_msgs/msg/AccelStamped.msg`

```rosidl
# An accel with reference coordinate frame and timestamp
std_msgs/Header header
Accel accel
```

### AccelWithCovariance `msg` {/* #common-interfaces-geometry-msgs-msg-accelwithcovariance-msg */}

`common_interfaces/geometry_msgs/msg/AccelWithCovariance.msg`

```rosidl
# This expresses acceleration in free space with uncertainty.

Accel accel

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
```

### AccelWithCovarianceStamped `msg` {/* #common-interfaces-geometry-msgs-msg-accelwithcovariancestamped-msg */}

`common_interfaces/geometry_msgs/msg/AccelWithCovarianceStamped.msg`

```rosidl
# This represents an estimated accel with reference coordinate frame and timestamp.
std_msgs/Header header
AccelWithCovariance accel
```

### Inertia `msg` {/* #common-interfaces-geometry-msgs-msg-inertia-msg */}

`common_interfaces/geometry_msgs/msg/Inertia.msg`

```rosidl
# Mass [kg]
float64 m

# Center of mass [m]
geometry_msgs/Vector3 com

# Inertia Tensor [kg-m^2]
#     | ixx ixy ixz |
# I = | ixy iyy iyz |
#     | ixz iyz izz |
float64 ixx
float64 ixy
float64 ixz
float64 iyy
float64 iyz
float64 izz
```

### InertiaStamped `msg` {/* #common-interfaces-geometry-msgs-msg-inertiastamped-msg */}

`common_interfaces/geometry_msgs/msg/InertiaStamped.msg`

```rosidl
# An Inertia with a time stamp and reference frame.

std_msgs/Header header
Inertia inertia
```

### Point `msg` {/* #common-interfaces-geometry-msgs-msg-point-msg */}

`common_interfaces/geometry_msgs/msg/Point.msg`

```rosidl
# This contains the position of a point in free space
float64 x
float64 y
float64 z
```

### Point32 `msg` {/* #common-interfaces-geometry-msgs-msg-point32-msg */}

`common_interfaces/geometry_msgs/msg/Point32.msg`

```rosidl
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z
```

### PointStamped `msg` {/* #common-interfaces-geometry-msgs-msg-pointstamped-msg */}

`common_interfaces/geometry_msgs/msg/PointStamped.msg`

```rosidl
# This represents a Point with reference coordinate frame and timestamp

std_msgs/Header header
Point point
```

### Polygon `msg` {/* #common-interfaces-geometry-msgs-msg-polygon-msg */}

`common_interfaces/geometry_msgs/msg/Polygon.msg`

```rosidl
# A specification of a polygon where the first and last points are assumed to be connected

Point32[] points
```

### PolygonInstance `msg` {/* #common-interfaces-geometry-msgs-msg-polygoninstance-msg */}

`common_interfaces/geometry_msgs/msg/PolygonInstance.msg`

```rosidl
# A specification of a polygon where the first and last points are assumed to be connected
# It includes a unique identification field for disambiguating multiple instances

geometry_msgs/Polygon polygon
int64 id
```

### PolygonInstanceStamped `msg` {/* #common-interfaces-geometry-msgs-msg-polygoninstancestamped-msg */}

`common_interfaces/geometry_msgs/msg/PolygonInstanceStamped.msg`

```rosidl
# This represents a Polygon with reference coordinate frame and timestamp
# It includes a unique identification field for disambiguating multiple instances

std_msgs/Header header
geometry_msgs/PolygonInstance polygon
```

### PolygonStamped `msg` {/* #common-interfaces-geometry-msgs-msg-polygonstamped-msg */}

`common_interfaces/geometry_msgs/msg/PolygonStamped.msg`

```rosidl
# This represents a Polygon with reference coordinate frame and timestamp

std_msgs/Header header
Polygon polygon
```

### Pose `msg` {/* #common-interfaces-geometry-msgs-msg-pose-msg */}

`common_interfaces/geometry_msgs/msg/Pose.msg`

```rosidl
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
```

### Pose2D `msg` {/* #common-interfaces-geometry-msgs-msg-pose2d-msg */}

`common_interfaces/geometry_msgs/msg/Pose2D.msg`

```rosidl
# Deprecated as of Foxy and will potentially be removed in any following release.
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
```

### PoseArray `msg` {/* #common-interfaces-geometry-msgs-msg-posearray-msg */}

`common_interfaces/geometry_msgs/msg/PoseArray.msg`

```rosidl
# An array of poses with a header for global reference.

std_msgs/Header header

Pose[] poses
```

### PoseStamped `msg` {/* #common-interfaces-geometry-msgs-msg-posestamped-msg */}

`common_interfaces/geometry_msgs/msg/PoseStamped.msg`

```rosidl
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
```

### PoseWithCovariance `msg` {/* #common-interfaces-geometry-msgs-msg-posewithcovariance-msg */}

`common_interfaces/geometry_msgs/msg/PoseWithCovariance.msg`

```rosidl
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
```

### PoseWithCovarianceStamped `msg` {/* #common-interfaces-geometry-msgs-msg-posewithcovariancestamped-msg */}

`common_interfaces/geometry_msgs/msg/PoseWithCovarianceStamped.msg`

```rosidl
# This expresses an estimated pose with a reference coordinate frame and timestamp

std_msgs/Header header
PoseWithCovariance pose
```

### Quaternion `msg` {/* #common-interfaces-geometry-msgs-msg-quaternion-msg */}

`common_interfaces/geometry_msgs/msg/Quaternion.msg`

```rosidl
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
```

### QuaternionStamped `msg` {/* #common-interfaces-geometry-msgs-msg-quaternionstamped-msg */}

`common_interfaces/geometry_msgs/msg/QuaternionStamped.msg`

```rosidl
# This represents an orientation with reference coordinate frame and timestamp.

std_msgs/Header header
Quaternion quaternion
```

### Transform `msg` {/* #common-interfaces-geometry-msgs-msg-transform-msg */}

`common_interfaces/geometry_msgs/msg/Transform.msg`

```rosidl
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
```

### TransformStamped `msg` {/* #common-interfaces-geometry-msgs-msg-transformstamped-msg */}

`common_interfaces/geometry_msgs/msg/TransformStamped.msg`

```rosidl
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id at the time of header.stamp
#
# This message is mostly used by the
# <a href="https://index.ros.org/p/tf2/">tf2</a> package.
# See its documentation for more information.
#
# The child_frame_id is necessary in addition to the frame_id
# in the Header to communicate the full reference for the transform
# in a self contained message.

# The frame id in the header is used as the reference frame of this transform.
std_msgs/Header header

# The frame id of the child frame to which this transform points.
string child_frame_id

# Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
Transform transform
```

### Twist `msg` {/* #common-interfaces-geometry-msgs-msg-twist-msg */}

`common_interfaces/geometry_msgs/msg/Twist.msg`

```rosidl
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
```

### TwistStamped `msg` {/* #common-interfaces-geometry-msgs-msg-twiststamped-msg */}

`common_interfaces/geometry_msgs/msg/TwistStamped.msg`

```rosidl
# A twist with reference coordinate frame and timestamp

std_msgs/Header header
Twist twist
```

### TwistWithCovariance `msg` {/* #common-interfaces-geometry-msgs-msg-twistwithcovariance-msg */}

`common_interfaces/geometry_msgs/msg/TwistWithCovariance.msg`

```rosidl
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
```

### TwistWithCovarianceStamped `msg` {/* #common-interfaces-geometry-msgs-msg-twistwithcovariancestamped-msg */}

`common_interfaces/geometry_msgs/msg/TwistWithCovarianceStamped.msg`

```rosidl
# This represents an estimated twist with reference coordinate frame and timestamp.

std_msgs/Header header
TwistWithCovariance twist
```

### Vector3 `msg` {/* #common-interfaces-geometry-msgs-msg-vector3-msg */}

`common_interfaces/geometry_msgs/msg/Vector3.msg`

```rosidl
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
```

### Vector3Stamped `msg` {/* #common-interfaces-geometry-msgs-msg-vector3stamped-msg */}

`common_interfaces/geometry_msgs/msg/Vector3Stamped.msg`

```rosidl
# This represents a Vector3 with reference coordinate frame and timestamp

# Note that this follows vector semantics with it always anchored at the origin,
# so the rotational elements of a transform are the only parts applied when transforming.

std_msgs/Header header
Vector3 vector
```

### VelocityStamped `msg` {/* #common-interfaces-geometry-msgs-msg-velocitystamped-msg */}

`common_interfaces/geometry_msgs/msg/VelocityStamped.msg`

```rosidl
# This expresses the timestamped velocity vector of a frame 'body_frame_id' in the reference frame 'reference_frame_id' expressed from arbitrary observation frame 'header.frame_id'.
# - If the 'body_frame_id' and 'header.frame_id' are identical, the velocity is observed and defined in the local coordinates system of the body
#   which is the usual use-case in mobile robotics and is also known as a body twist.

std_msgs/Header header
string body_frame_id
string reference_frame_id
Twist velocity
```

### Wrench `msg` {/* #common-interfaces-geometry-msgs-msg-wrench-msg */}

`common_interfaces/geometry_msgs/msg/Wrench.msg`

```rosidl
# This represents force in free space, separated into its linear and angular parts.

Vector3  force
Vector3  torque
```

### WrenchStamped `msg` {/* #common-interfaces-geometry-msgs-msg-wrenchstamped-msg */}

`common_interfaces/geometry_msgs/msg/WrenchStamped.msg`

```rosidl
# A wrench with reference coordinate frame and timestamp

std_msgs/Header header
Wrench wrench
```

## health

### GetHealthState `srv` {/* #health-srv-gethealthstate-srv */}

`health/srv/GetHealthState.srv`

```rosidl
---
HealthState state
```

### HealthState `msg` {/* #health-msg-healthstate-msg */}

`health/msg/HealthState.msg`

```rosidl
float32 voltage              # system voltage in V, -1 unknown
bool charging                # whether charging
int64 remaining_s            # estimated remaining seconds, -1 unknown
SensorReading[] readings     # all raw sensor readings
```

### SensorReading `msg` {/* #health-msg-sensorreading-msg */}

`health/msg/SensorReading.msg`

```rosidl
string name                  # sensor name, e.g. "cpu", "gpu", "nvme"
float32 temp_c               # temperature in °C, -1 if unavailable
float32 voltage              # voltage in V, -1 if unavailable
float32 current_a            # current in A, -1 if unavailable
float32 battery_percent      # battery percentage 0-100, -1 if no battery
```

### StreamHealthState `srv` {/* #health-srv-streamhealthstate-srv */}

`health/srv/StreamHealthState.srv`

```rosidl
---
HealthState state            # server_stream: push 1 frame per cycle
```

## liaison

### GetHandsfreeStatus `srv` {/* #liaison-srv-gethandsfreestatus-srv */}

`liaison/srv/GetHandsfreeStatus.srv`

```rosidl
# Read the current robot-local wake-word interaction state.
---
bool enabled
string state
string keyword
string mic_provider_id
string speaker_provider_id
uint64 last_wake_ms
string last_transcript
string last_error
```

### Interrupt `srv` {/* #liaison-srv-interrupt-srv */}

`liaison/srv/Interrupt.srv`

```rosidl
string session_id
string reason
---
bool ok
string session_id
```

### SetHandsfree `srv` {/* #liaison-srv-sethandsfree-srv */}

`liaison/srv/SetHandsfree.srv`

```rosidl
# Enable or disable robot-local wake-word interaction.
bool enabled
string mic_provider_id
string speaker_provider_id
---
bool ok
bool enabled
string state
string detail
```

### StartVoiceSession `srv` {/* #liaison-srv-startvoicesession-srv */}

`liaison/srv/StartVoiceSession.srv`

```rosidl
# Voice push-to-talk turn orchestrated entirely inside Liaison.
# Contract: robonix/system/liaison/voice
string  session_id              # empty → server allocates UUID
string  client_user_id          # hint (e.g. local OS user)
uint32  record_seconds          # 0 → server default (5s)
string  language                # BCP-47, "" = auto
bool    tts_enabled             # play synthesized response
string  mic_node_id             # "" = auto-discover via Atlas
string  asr_node_id             # ""
string  voiceprint_node_id      # ""
string  tts_node_id             # ""
string  speaker_node_id         # ""
string  context_json            # extra fields merged into Task.context_json
---
liaison/VoiceEvent event        # streamed
```

### SubmitTask `srv` {/* #liaison-srv-submittask-srv */}

`liaison/srv/SubmitTask.srv`

```rosidl
pilot/Task task
---
pilot/PilotEvent event
```

### VoiceEvent `msg` {/* #liaison-msg-voiceevent-msg */}

`liaison/msg/VoiceEvent.msg`

```rosidl
# Streamed by SrvLiaison.StartVoiceSession.
# event_kind values are stable; clients should ignore unknown kinds.
uint32 SESSION_STARTED   = 0
uint32 RECORDING_STARTED = 1
uint32 RECORDING_DONE    = 2
uint32 ASR_PARTIAL       = 3
uint32 ASR_FINAL         = 4
uint32 USER_IDENTIFIED   = 5
uint32 PILOT             = 6   # `pilot` field carries the wrapped PilotEvent
uint32 TTS_STARTED       = 7
uint32 TTS_DONE          = 8
uint32 SESSION_DONE      = 9
uint32 ERROR             = 10

uint32           event_kind
string           session_id
string           text
string           user_id
float32          confidence
pilot/PilotEvent pilot
string           error
string           status_message
uint64           timestamp_ms
```

### WatchHandsfreeEvents `srv` {/* #liaison-srv-watchhandsfreeevents-srv */}

`liaison/srv/WatchHandsfreeEvents.srv`

```rosidl
# Subscribe to voice events emitted by Liaison's current robot-local
# hands-free loop. This is observational; it does not own microphone input.
---
liaison/VoiceEvent event
```

## lidar

### GetLaserScan `srv` {/* #lidar-srv-getlaserscan-srv */}

`lidar/srv/GetLaserScan.srv`

```rosidl
# robonix/primitive/lidar/snapshot — on-demand single LaserScan.
---
sensor_msgs/LaserScan scan
```

## lifecycle

### Driver `srv` {/* #lifecycle-srv-driver-srv */}

`lifecycle/srv/Driver.srv`

```rosidl
uint8 CMD_INIT       = 0
uint8 CMD_ACTIVATE   = 1
uint8 CMD_DEACTIVATE = 2
uint8 CMD_SHUTDOWN   = 3

uint8  command
string config_json
---
bool   ok
string state
string error
```

## lifecycle_msgs

### ChangeState `srv` {/* #rcl-interfaces-lifecycle-msgs-srv-changestate-srv */}

`rcl_interfaces/lifecycle_msgs/srv/ChangeState.srv`

```rosidl
# The requested transition.
#
# This change state service will fail if the transition is not possible.
Transition transition
---

# Indicates whether the service was able to initiate the state transition
bool success
```

### GetAvailableStates `srv` {/* #rcl-interfaces-lifecycle-msgs-srv-getavailablestates-srv */}

`rcl_interfaces/lifecycle_msgs/srv/GetAvailableStates.srv`

```rosidl
---
# Array of possible states that can be transitioned to.
State[] available_states
```

### GetAvailableTransitions `srv` {/* #rcl-interfaces-lifecycle-msgs-srv-getavailabletransitions-srv */}

`rcl_interfaces/lifecycle_msgs/srv/GetAvailableTransitions.srv`

```rosidl
---
# An array of the possible start_state-goal_state transitions
TransitionDescription[] available_transitions
```

### GetState `srv` {/* #rcl-interfaces-lifecycle-msgs-srv-getstate-srv */}

`rcl_interfaces/lifecycle_msgs/srv/GetState.srv`

```rosidl
---
# The current state-machine state of the node.
State current_state
```

### State `msg` {/* #rcl-interfaces-lifecycle-msgs-msg-state-msg */}

`rcl_interfaces/lifecycle_msgs/msg/State.msg`

```rosidl
# Primary state definitions as depicted in:
# http://design.ros2.org/articles/node_lifecycle.html

# These are the primary states. State changes can only be requested when the
# node is in one of these states.

# Indicates state has not yet been set.
uint8 PRIMARY_STATE_UNKNOWN = 0

# This is the life cycle state the node is in immediately after being
# instantiated.
uint8 PRIMARY_STATE_UNCONFIGURED = 1

# This state represents a node that is not currently performing any processing.
uint8 PRIMARY_STATE_INACTIVE = 2

# This is the main state of the node's life cycle. While in this state, the node
# performs any processing, responds to service requests, reads and processes
# data, produces output, etc.
uint8 PRIMARY_STATE_ACTIVE = 3

# The finalized state is the state in which the node ends immediately before
# being destroyed.
uint8 PRIMARY_STATE_FINALIZED = 4

# Temporary intermediate states. When a transition is requested, the node
# changes its state into one of these states.

# In this transition state the node's onConfigure callback will be called to
# allow the node to load its configuration and conduct any required setup.
uint8 TRANSITION_STATE_CONFIGURING = 10

# In this transition state the node's callback onCleanup will be called to clear
# all state and return the node to a functionally equivalent state as when
# first created.
uint8 TRANSITION_STATE_CLEANINGUP = 11

# In this transition state the callback onShutdown will be executed to do any
# cleanup necessary before destruction.
uint8 TRANSITION_STATE_SHUTTINGDOWN = 12

# In this transition state the callback onActivate will be executed to do any
# final preparations to start executing.
uint8 TRANSITION_STATE_ACTIVATING = 13

# In this transition state the callback onDeactivate will be executed to do any
# cleanup to start executing, and reverse the onActivate changes.
uint8 TRANSITION_STATE_DEACTIVATING = 14

# This transition state is where any error may be cleaned up.
uint8 TRANSITION_STATE_ERRORPROCESSING = 15

# The state id value from the above definitions.
uint8 id

# A text label of the state.
string label
```

### Transition `msg` {/* #rcl-interfaces-lifecycle-msgs-msg-transition-msg */}

`rcl_interfaces/lifecycle_msgs/msg/Transition.msg`

```rosidl
# Default values for transitions as described in:
# http://design.ros2.org/articles/node_lifecycle.html

# Reserved [0-9], publicly available transitions.
# When a node is in one of these primary states, these transitions can be
# invoked.

# This transition will instantiate the node, but will not run any code beyond
# the constructor.
uint8 TRANSITION_CREATE = 0

# The node's onConfigure callback will be called to allow the node to load its
# configuration and conduct any required setup.
uint8 TRANSITION_CONFIGURE = 1

# The node's callback onCleanup will be called in this transition to allow the
# node to load its configuration and conduct any required setup.
uint8 TRANSITION_CLEANUP = 2

# The node's callback onActivate will be executed to do any final preparations
# to start executing.
uint8 TRANSITION_ACTIVATE = 3

# The node's callback onDeactivate will be executed to do any cleanup to start
# executing, and reverse the onActivate changes.
uint8 TRANSITION_DEACTIVATE = 4

# This signals shutdown during an unconfigured state, the node's callback
# onShutdown will be executed to do any cleanup necessary before destruction.
uint8 TRANSITION_UNCONFIGURED_SHUTDOWN  = 5

# This signals shutdown during an inactive state, the node's callback onShutdown
# will be executed to do any cleanup necessary before destruction.
uint8 TRANSITION_INACTIVE_SHUTDOWN = 6

# This signals shutdown during an active state, the node's callback onShutdown
# will be executed to do any cleanup necessary before destruction.
uint8 TRANSITION_ACTIVE_SHUTDOWN = 7

# This transition will simply cause the deallocation of the node.
uint8 TRANSITION_DESTROY = 8

# Reserved [10-69], private transitions
# These transitions are not publicly available and cannot be invoked by a user.
# The following transitions are implicitly invoked based on the callback
# feedback of the intermediate transition states.
uint8 TRANSITION_ON_CONFIGURE_SUCCESS = 10
uint8 TRANSITION_ON_CONFIGURE_FAILURE = 11
uint8 TRANSITION_ON_CONFIGURE_ERROR = 12

uint8 TRANSITION_ON_CLEANUP_SUCCESS = 20
uint8 TRANSITION_ON_CLEANUP_FAILURE = 21
uint8 TRANSITION_ON_CLEANUP_ERROR = 22

uint8 TRANSITION_ON_ACTIVATE_SUCCESS = 30
uint8 TRANSITION_ON_ACTIVATE_FAILURE = 31
uint8 TRANSITION_ON_ACTIVATE_ERROR = 32

uint8 TRANSITION_ON_DEACTIVATE_SUCCESS = 40
uint8 TRANSITION_ON_DEACTIVATE_FAILURE = 41
uint8 TRANSITION_ON_DEACTIVATE_ERROR = 42

uint8 TRANSITION_ON_SHUTDOWN_SUCCESS = 50
uint8 TRANSITION_ON_SHUTDOWN_FAILURE = 51
uint8 TRANSITION_ON_SHUTDOWN_ERROR = 52

uint8 TRANSITION_ON_ERROR_SUCCESS = 60
uint8 TRANSITION_ON_ERROR_FAILURE = 61
uint8 TRANSITION_ON_ERROR_ERROR = 62

# Reserved [90-99]. Transition callback success values.
# These return values ought to be set as a return value for each callback.
# Depending on which return value, the transition will be executed correctly or
# fallback/error callbacks will be triggered.

# The transition callback successfully performed its required functionality.
uint8 TRANSITION_CALLBACK_SUCCESS = 97

# The transition callback failed to perform its required functionality.
uint8 TRANSITION_CALLBACK_FAILURE = 98

# The transition callback encountered an error that requires special cleanup, if
# possible.
uint8 TRANSITION_CALLBACK_ERROR = 99

##
## Fields
##

# The transition id from above definitions.
uint8 id

# A text label of the transition.
string label
```

### TransitionDescription `msg` {/* #rcl-interfaces-lifecycle-msgs-msg-transitiondescription-msg */}

`rcl_interfaces/lifecycle_msgs/msg/TransitionDescription.msg`

```rosidl
# The transition id and label of this description.
Transition transition

# The current state from which this transition transitions.
State start_state

# The desired target state of this transition.
State goal_state
```

### TransitionEvent `msg` {/* #rcl-interfaces-lifecycle-msgs-msg-transitionevent-msg */}

`rcl_interfaces/lifecycle_msgs/msg/TransitionEvent.msg`

```rosidl
# The time point at which this event occurred.
uint64 timestamp

# The id and label of this transition event.
Transition transition

# The starting state from which this event transitioned.
State start_state

# The end state of this transition event.
State goal_state
```

## map

### DeleteMap `srv` {/* #map-srv-deletemap-srv */}

`map/srv/DeleteMap.srv`

```rosidl
# robonix/service/map/delete_map — permanently delete a saved map by id (RPC).
# Removes {MAPPING_MAPS_DIR}/<map_id>/ (rtabmap.db + preview artifacts) from
# disk. Does NOT affect the live running map unless <map_id> is the one
# currently loaded. Irreversible — back up first if unsure.
string map_id      # id of the saved map to delete
---
bool ok
string map_id      # sanitized id acted on
string detail
```

### GetMode `srv` {/* #map-srv-getmode-srv */}

`map/srv/GetMode.srv`

```rosidl
# robonix/service/map/get_mode — read the SLAM mode currently in effect (RPC).
# Complements switch_mode (which sets it): lets a UI / LLM reflect the REAL
# runtime mode instead of guessing from the startup config's map_mode.
---
bool ok
string mode        # "mapping" (build/extend) | "localization" (read-only)
string detail
```

### GetPose `srv` {/* #map-srv-getpose-srv */}

`map/srv/GetPose.srv`

```rosidl
# robonix/service/map/get_pose — read the robot's current pose in the MAP frame (RPC).
# Returns x/y (metres) + theta (yaw, radians) in the SLAM map frame, taken from
# the live pose the mapping service publishes. Read-only; complements
# pose_estimate (which SEEDS a pose for relocalization). Use it to record a
# named waypoint, report the robot's position, or branch on location.
---
bool ok
float64 x
float64 y
float64 theta       # yaw, radians
string frame_id     # map frame id (usually "map")
string detail
```

### ListMaps `srv` {/* #map-srv-listmaps-srv */}

`map/srv/ListMaps.srv`

```rosidl
# robonix/service/map/list_maps — list saved maps available for load/delete.
# maps_json is a UTF-8 JSON array so the contract can remain flat while map
# library metadata evolves. Current entries include map_id, has_spatial_artifact,
# spatial_ok, artifact_size, has_preview, updated, and meta. Provider artifacts
# are opaque; callers must load/delete them by map_id rather than by path.
---
bool ok
string detail
string maps_json
```

### LoadMap `srv` {/* #map-srv-loadmap-srv */}

`map/srv/LoadMap.srv`

```rosidl
# robonix/service/map/load_map — switch SLAM onto a previously-saved map (RPC).
# Loads <map_id>'s opaque spatial artifact and, in localization mode, freezes
# map growth so the robot relocalizes against the saved map and the map frame
# becomes stable across runs (scene keys its objects to that frame). With
# has_initial_pose set, the pose guess below is seeded before relocalizing so
# convergence is fast and unambiguous in repetitive spaces.
string map_id      # which saved map to load (must already exist on disk)
string mode        # "localization" (default) | "mapping" (resume building it)
bool has_initial_pose   # true → use x/y/theta below as the starting guess
float64 x          # initial pose guess, map frame, meters
float64 y          # initial pose guess, map frame, meters
float64 theta      # initial yaw, map frame, radians
---
bool ok
string detail      # human-readable status / error
```

### MapLifecycle `msg` {/* #map-msg-maplifecycle-msg */}

`map/msg/MapLifecycle.msg`

```rosidl
# Map identity + lifecycle broadcast from the mapping service.
#
# Published latched (transient_local) so late-joining consumers (e.g. the
# scene service) learn the current map identity at startup, and re-published
# on every lifecycle transition (init / load / reset / mode switch; save
# does not change the live identity — its re-key semantics are a later,
# separate design).
#
# The (map_id, generation) pair is the map-frame identity key: consumers
# holding map-frame coordinates must treat a change in EITHER field as
# "previous coordinates are no longer anchored" — reset_map keeps map_id
# but moves the map origin, which only generation makes visible.
string map_id      # named-map id currently bound; empty when ephemeral (no map_id configured)
string mode        # "mapping" or "localization"
uint64 generation  # monotonic; +1 when the map origin may have changed: mapping-mode
                   # session start/load and reset_map. NEVER bumped by a localization
                   # load (stable frame is the point) or by a mode switch.
```

### PoseEstimate `srv` {/* #map-srv-poseestimate-srv */}

`map/srv/PoseEstimate.srv`

```rosidl
# robonix/service/map/pose_estimate — seed rtabmap with a known pose (RPC).
# Publishes an initial pose estimate so rtabmap's localization re-converges:
# global relocalization, kidnapped-robot recovery, or "I know I'm roughly
# here, go figure out exactly where". The pose is in the map frame, same as
# load_map; rtabmap then refines it to the true map pose via scan/ICP matching.
float64 x          # pose guess, map frame, meters
float64 y          # pose guess, map frame, meters
float64 theta      # yaw guess, map frame, radians
float64 cov_xy     # optional 1-sigma position uncertainty, meters; 0 → default
float64 cov_theta  # optional 1-sigma yaw uncertainty, radians; 0 → default
---
bool ok
string detail      # human-readable status / error
```

### ResetMap `srv` {/* #map-srv-resetmap-srv */}

`map/srv/ResetMap.srv`

```rosidl
# robonix/service/map/reset_map — clear the LIVE SLAM map and start rebuilding
# from the robot's current pose (RPC). Calls rtabmap's reset; the map-frame
# origin moves to the current pose. Does NOT delete any saved map on disk
# (use delete_map for that) — only wipes the in-memory/working map.
---
bool ok
string detail
```

### SaveMap `srv` {/* #map-srv-savemap-srv */}

`map/srv/SaveMap.srv`

```rosidl
# robonix/service/map/save_map — snapshot the current SLAM map to disk (RPC).
# Persists the provider's current spatial map under a stable map_id. Backends
# may use RTAB-Map, occupancy-grid files, or another representation internally;
# callers must treat the artifact as opaque and address it only by map_id.
# Reuse the same map_id with scene's semantic map so spatial and semantic state
# stays aligned.
string map_id      # stable id; sanitized to a filesystem-safe directory name
string note        # optional human note recorded in meta.yaml (may be empty)
---
bool ok
string map_id          # sanitized id actually written
string detail          # human-readable status / error
```

### SwitchMode `srv` {/* #map-srv-switchmode-srv */}

`map/srv/SwitchMode.srv`

```rosidl
# robonix/service/map/switch_mode — flip the running SLAM between building and
# localizing on the CURRENT map, without loading a different one (RPC).
# The config's map_mode is only the startup pre-selection; this switches it at
# runtime: build a map, switch to localization to test/use it, switch back to
# mapping to extend it — all without re-deploying.
string mode        # "mapping" (build/extend) | "localization" (relocalize, read-only)
---
bool ok
string detail
```

## memory

### Compact `srv` {/* #memory-srv-compact-srv */}

`memory/srv/Compact.srv`

```rosidl
# robonix/service/memory/compact — summarize long-term memory (unary RPC).
# Request: no fields (empty trigger, maps to google.protobuf.Empty on the facade when appropriate).
---
std_msgs/String summary
```

### Save `srv` {/* #memory-srv-save-srv */}

`memory/srv/Save.srv`

```rosidl
# robonix/service/memory/save — persist a fact or preference (unary RPC).
std_msgs/String content
---
std_msgs/String confirmation
```

### Search `srv` {/* #memory-srv-search-srv */}

`memory/srv/Search.srv`

```rosidl
# robonix/service/memory/search — semantic search over long-term memory (unary RPC).
std_msgs/String query
---
std_msgs/String results
```

## module_health

### GetModuleHealth `srv` {/* #module-health-srv-getmodulehealth-srv */}

`module_health/srv/GetModuleHealth.srv`

```rosidl
---
ModuleHealthReport report
```

### GetModuleHealthSnapshot `srv` {/* #module-health-srv-getmodulehealthsnapshot-srv */}

`module_health/srv/GetModuleHealthSnapshot.srv`

```rosidl
---
ModuleHealthSnapshot snapshot
```

### ModuleHealth `msg` {/* #module-health-msg-modulehealth-msg */}

`module_health/msg/ModuleHealth.msg`

```rosidl
string module_key
string module_id
string provider_id

uint8 health
string state
string reason_code
string detail
string source

uint64 received_ts_ns
uint32 ttl_ms
```

### ModuleHealthEvent `msg` {/* #module-health-msg-modulehealthevent-msg */}

`module_health/msg/ModuleHealthEvent.msg`

```rosidl
uint64 ts_ns
uint64 seq

string module_key
uint8 previous_health
uint8 current_health
string reason_code
string detail
string source
```

### ModuleHealthReport `msg` {/* #module-health-msg-modulehealthreport-msg */}

`module_health/msg/ModuleHealthReport.msg`

```rosidl
uint32 schema_version
ModuleHealth module
```

### ModuleHealthSnapshot `msg` {/* #module-health-msg-modulehealthsnapshot-msg */}

`module_health/msg/ModuleHealthSnapshot.msg`

```rosidl
uint32 schema_version
uint64 ts_ns
uint64 seq
ModuleHealth[] modules
```

## nav_msgs

### GetMap `srv` {/* #common-interfaces-nav-msgs-srv-getmap-srv */}

`common_interfaces/nav_msgs/srv/GetMap.srv`

```rosidl
# Get the map as a nav_msgs/OccupancyGrid
---
# The current map hosted by this map service.
OccupancyGrid map
```

### GetPlan `srv` {/* #common-interfaces-nav-msgs-srv-getplan-srv */}

`common_interfaces/nav_msgs/srv/GetPlan.srv`

```rosidl
# Get a plan from the current position to the goal Pose

# The start pose for the plan
geometry_msgs/PoseStamped start

# The final pose of the goal position
geometry_msgs/PoseStamped goal

# If the goal is obstructed, how many meters the planner can
# relax the constraint in x and y before failing.
float32 tolerance
---
# Array of poses from start to goal if one was successfully found.
Path plan
```

### Goals `msg` {/* #common-interfaces-nav-msgs-msg-goals-msg */}

`common_interfaces/nav_msgs/msg/Goals.msg`

```rosidl
# An array of navigation goals


# This header will store the time at which the poses were computed (not to be confused with the stamps of the poses themselves)
# In the case that individual poses do not have their frame_id set or their timetamp set they will use the default value here.
std_msgs/Header header

# An array of goals to for navigation to achieve.
# The goals should be executed in the order of the array.
# The header and stamp are intended to be used for computing the position of the goals.
# They may vary to support cases of goals that are moving with respect to the robot.
geometry_msgs/PoseStamped[] goals
```

### GridCells `msg` {/* #common-interfaces-nav-msgs-msg-gridcells-msg */}

`common_interfaces/nav_msgs/msg/GridCells.msg`

```rosidl
# An array of cells in a 2D grid

std_msgs/Header header

# Width of each cell
float32 cell_width

# Height of each cell
float32 cell_height

# Each cell is represented by the Point at the center of the cell
geometry_msgs/Point[] cells
```

### LoadMap `srv` {/* #common-interfaces-nav-msgs-srv-loadmap-srv */}

`common_interfaces/nav_msgs/srv/LoadMap.srv`

```rosidl
# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_url
---
# Result code defintions
uint8 RESULT_SUCCESS=0
uint8 RESULT_MAP_DOES_NOT_EXIST=1
uint8 RESULT_INVALID_MAP_DATA=2
uint8 RESULT_INVALID_MAP_METADATA=3
uint8 RESULT_UNDEFINED_FAILURE=255

# Returned map is only valid if result equals RESULT_SUCCESS
nav_msgs/OccupancyGrid map
uint8 result
```

### MapMetaData `msg` {/* #common-interfaces-nav-msgs-msg-mapmetadata-msg */}

`common_interfaces/nav_msgs/msg/MapMetaData.msg`

```rosidl
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
```

### OccupancyGrid `msg` {/* #common-interfaces-nav-msgs-msg-occupancygrid-msg */}

`common_interfaces/nav_msgs/msg/OccupancyGrid.msg`

```rosidl
# This represents a 2-D grid map
std_msgs/Header header

# MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).
# Cell (1, 0) will be listed second, representing the next cell in the x direction.
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
# The values inside are application dependent, but frequently,
# 0 represents unoccupied, 1 represents definitely occupied, and
# -1 represents unknown.
int8[] data
```

### Odometry `msg` {/* #common-interfaces-nav-msgs-msg-odometry-msg */}

`common_interfaces/nav_msgs/msg/Odometry.msg`

```rosidl
# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

# Includes the frame id of the pose parent.
std_msgs/Header header

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
```

### Path `msg` {/* #common-interfaces-nav-msgs-msg-path-msg */}

`common_interfaces/nav_msgs/msg/Path.msg`

```rosidl
# An array of poses that represents a Path for a robot to follow.

# Indicates the frame_id of the path.
std_msgs/Header header

# Array of poses to follow.
geometry_msgs/PoseStamped[] poses
```

### SetMap `srv` {/* #common-interfaces-nav-msgs-srv-setmap-srv */}

`common_interfaces/nav_msgs/srv/SetMap.srv`

```rosidl
# Set a new map together with an initial pose

# Requested 2D map to be set.
nav_msgs/OccupancyGrid map

# Estimated initial pose when setting new map.
geometry_msgs/PoseWithCovarianceStamped initial_pose
---

# True if the map was successfully set, false otherwise.
bool success
```

## navigation

### CancelNavigation `srv` {/* #navigation-srv-cancelnavigation-srv */}

`navigation/srv/CancelNavigation.srv`

```rosidl
# robonix/service/navigation/navigate/cancel — request cancellation of a
# navigation goal (RPC). Empty `run_id` cancels the current active
# goal.
string run_id
---
bool accepted
string detail
```

### GetNavigationStatus `srv` {/* #navigation-srv-getnavigationstatus-srv */}

`navigation/srv/GetNavigationStatus.srv`

```rosidl
# robonix/service/navigation/navigate/status — query status of a navigation goal (RPC)
string run_id
---
bool known
string state
string detail
```

### Navigate `srv` {/* #navigation-srv-navigate-srv */}

`navigation/srv/Navigate.srv`

```rosidl
# robonix/service/navigation/navigate — goal-based navigation (RPC).
# Returns an optional provider-allocated `run_id` so callers can address the
# goal in the async sub-contracts `navigate/status` and `navigate/cancel`.
# Empty run_id means those sub-contracts should target the most recent
# navigation call.
geometry_msgs/PoseStamped goal
---
bool accepted
string run_id
string detail
```

### NavigationStatus `msg` {/* #navigation-msg-navigationstatus-msg */}

`navigation/msg/NavigationStatus.msg`

```rosidl
# Navigation goal status
# status values: success, failed, navigating, timeout, cancelled
string goal_id
string status
```

## perception

### PerceptionDetect `srv` {/* #perception-srv-perceptiondetect-srv */}

`perception/srv/PerceptionDetect.srv`

```rosidl
# Legacy unregistered detection IDL. No contract TOML references this file.
sensor_msgs/Image image
---
std_msgs/String results
```

## pilot

### AbortSession `srv` {/* #pilot-srv-abortsession-srv */}

`pilot/srv/AbortSession.srv`

```rosidl
string session_id
---
bool ok
```

### BatchResult `msg` {/* #pilot-msg-batchresult-msg */}

`pilot/msg/BatchResult.msg`

```rosidl
# Correlates with Plan.plan_id for this execution round.
string plan_id
string session_id

uint32 round
# Every node that reached a terminal state in this round (leaf and non-leaf),
# as a full RtdlNodeState record — not just the leaf capability-call results.
RtdlNodeState[] results
bool any_failed
```

### CapabilityCall `msg` {/* #pilot-msg-capabilitycall-msg */}

`pilot/msg/CapabilityCall.msg`

```rosidl
# One node in a Plan: invoke `contract_id` on `provider_id` with `args_json`.
# executor looks `provider_id` up in atlas at dispatch time, gets the endpoint,
# and routes via the cap's declared transport (currently MCP only for
# LLM-callable contracts).
string call_id
string provider_id
string contract_id
string args_json
```

### CapabilityCallResult `msg` {/* #pilot-msg-capabilitycallresult-msg */}

`pilot/msg/CapabilityCallResult.msg`

```rosidl
string call_id
string provider_id
string contract_id
bool success
string output
string error
```

### ListSessions `srv` {/* #pilot-srv-listsessions-srv */}

`pilot/srv/ListSessions.srv`

```rosidl
# Empty request.
---
SessionInfo[] sessions
```

### PilotEvent `msg` {/* #pilot-msg-pilotevent-msg */}

`pilot/msg/PilotEvent.msg`

```rosidl
# event_kind: 0=text_chunk 1=plan 2=batch_result 3=status 4=final_text
#             5=node_state 6=task_state
uint32 event_kind
string session_id
string text_chunk
Plan plan
BatchResult batch_result
SessionStatusEvent status
string final_text
RtdlNodeState node_state
TaskStateEvent task_state
```

### Plan `msg` {/* #pilot-msg-plan-msg */}

`pilot/msg/Plan.msg`

```rosidl
# Plan: structural payload pilot hands to executor for execution.
# Encodes an RTDL execution tree as arena nodes. Empty plans are represented
# as one sequence root node with no children.
string plan_id
string session_id
uint32 round
RtdlNode[] nodes
uint32 root_index
```

### RtdlNode `msg` {/* #pilot-msg-rtdlnode-msg */}

`pilot/msg/RtdlNode.msg`

```rosidl
# RTDL execution tree node encoded as an arena entry in Plan.nodes.
# node_kind: 0=sequence, 1=parallel, 2=do.
uint32 node_kind

# Child node indices for sequence and parallel nodes. Empty for do nodes.
uint32[] children

# Capability call payload for do nodes. Sequence and parallel leave this empty.
CapabilityCall call

# Stable identifier for this RTDL node, generated by Pilot from the RTDL plan.
string op_id

# Human-readable intent for this RTDL node.
string description
```

### RtdlNodeState `msg` {/* #pilot-msg-rtdlnodestate-msg */}

`pilot/msg/RtdlNodeState.msg`

```rosidl
string plan_id
uint32 node_index
uint32 node_kind
uint32 state
uint32 PENDING=0
uint32 RUNNING=1
uint32 SUCCEEDED=2
uint32 FAILED=3
uint32 CANCELED=4
uint32 TIMEOUT=5
uint32 PAUSED=6
# RTDL non-leaf node result, like sequence or parallel.
string operator_detail
# RTDL leaf node result.
CapabilityCallResult leaf_result
# Stable identifier copied from Plan.nodes[node_index].
string op_id
# Human-readable intent copied from Plan.nodes[node_index].
string description
```

### SessionInfo `msg` {/* #pilot-msg-sessioninfo-msg */}

`pilot/msg/SessionInfo.msg`

```rosidl
string session_id
uint32 state
uint64 created_at_ms
uint32 turn_count
```

### SessionStatusEvent `msg` {/* #pilot-msg-sessionstatusevent-msg */}

`pilot/msg/SessionStatusEvent.msg`

```rosidl
# SessionState: ACTIVE=0 COMPLETED=1 FAILED=2 WAITING_INPUT=3
string session_id
uint32 state
string message
```

### SubmitTask `srv` {/* #pilot-srv-submittask-srv */}

`pilot/srv/SubmitTask.srv`

```rosidl
pilot/Task task
---
pilot/PilotEvent event
```

### Task `msg` {/* #pilot-msg-task-msg */}

`pilot/msg/Task.msg`

```rosidl
# TaskSource: TEXT=0 AUDIO=1 API=2
#
# TODO(post-liaison-port): drop `source` + `audio_data` — modality
# normalisation belongs in liaison (mic → ASR → text → pilot). pilot
# should never see PCM bytes. Keeping them here only because liaison's
# `dev` branch still writes raw audio into Task; remove once liaison
# moves off this path.
#
# TODO(post-liaison-port): add `string user_id` — who issued the task
# (free-form: user@host, uuid, …). `session_id` only identifies the
# conversation thread, not the user. Multi-user-on-one-robot has no
# discriminator today.
#
# TODO(post-liaison-port): replace `context_json` with typed control
# bools. Current consumers parse JSON for two flags only:
#   {"session_end": true}  → trigger compact_memory, no VLM turn
#   {"abort_turn":  true}  → cancel any in-flight turn for session_id
# Lift those into `bool session_end` + `bool abort_turn` fields; keep
# context_json as the genuine-extras escape hatch (image refs, location,
# …). Liaison currently writes to context_json directly, so deferred
# until liaison ports.
string task_id
string session_id
uint32 source
string text
uint8[] audio_data
string context_json
uint64 timestamp_ms
```

### TaskStateEvent `msg` {/* #pilot-msg-taskstateevent-msg */}

`pilot/msg/TaskStateEvent.msg`

```rosidl
# The pilot's current overall task, forwarded to the chat for live display.
# status: "in_progress" | "done" (empty until the LLM first sets a task).
string goal
string success_criterion
string status
```

## rcl_interfaces

### DescribeParameters `srv` {/* #rcl-interfaces-rcl-interfaces-srv-describeparameters-srv */}

`rcl_interfaces/rcl_interfaces/srv/DescribeParameters.srv`

```rosidl
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
```

### FloatingPointRange `msg` {/* #rcl-interfaces-rcl-interfaces-msg-floatingpointrange-msg */}

`rcl_interfaces/rcl_interfaces/msg/FloatingPointRange.msg`

```rosidl
# Represents bounds and a step value for a floating point typed parameter.

# Start value for valid values, inclusive.
float64 from_value

# End value for valid values, inclusive.
float64 to_value

# Size of valid steps between the from and to bound.
#
# Step is considered to be a magnitude, therefore negative values are treated
# the same as positive values, and a step value of zero implies a continuous
# range of values.
#
# Ideally, the step would be less than or equal to the distance between the
# bounds, as well as an even multiple of the distance between the bounds, but
# neither are required.
#
# If the absolute value of the step is larger than or equal to the distance
# between the two bounds, then the bounds will be the only valid values. e.g. if
# the range is defined as {from_value: 1.0, to_value: 2.0, step: 5.0} then the
# valid values will be 1.0 and 2.0.
#
# If the step is less than the distance between the bounds, but the distance is
# not a multiple of the step, then the "to" bound will always be a valid value,
# e.g. if the range is defined as {from_value: 2.0, to_value: 5.0, step: 2.0}
# then the valid values will be 2.0, 4.0, and 5.0.
float64 step
```

### GetParameterTypes `srv` {/* #rcl-interfaces-rcl-interfaces-srv-getparametertypes-srv */}

`rcl_interfaces/rcl_interfaces/srv/GetParameterTypes.srv`

```rosidl
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
```

### GetParameters `srv` {/* #rcl-interfaces-rcl-interfaces-srv-getparameters-srv */}

`rcl_interfaces/rcl_interfaces/srv/GetParameters.srv`

```rosidl
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
```

### IntegerRange `msg` {/* #rcl-interfaces-rcl-interfaces-msg-integerrange-msg */}

`rcl_interfaces/rcl_interfaces/msg/IntegerRange.msg`

```rosidl
# Represents bounds and a step value for an integer typed parameter.

# Start value for valid values, inclusive.
int64 from_value

# End value for valid values, inclusive.
int64 to_value

# Size of valid steps between the from and to bound.
#
# A step value of zero implies a continuous range of values. Ideally, the step
# would be less than or equal to the distance between the bounds, as well as an
# even multiple of the distance between the bounds, but neither are required.
#
# If the absolute value of the step is larger than or equal to the distance
# between the two bounds, then the bounds will be the only valid values. e.g. if
# the range is defined as {from_value: 1, to_value: 2, step: 5} then the valid
# values will be 1 and 2.
#
# If the step is less than the distance between the bounds, but the distance is
# not a multiple of the step, then the "to" bound will always be a valid value,
# e.g. if the range is defined as {from_value: 2, to_value: 5, step: 2} then
# the valid values will be 2, 4, and 5.
uint64 step
```

### ListParameters `srv` {/* #rcl-interfaces-rcl-interfaces-srv-listparameters-srv */}

`rcl_interfaces/rcl_interfaces/srv/ListParameters.srv`

```rosidl
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
```

### ListParametersResult `msg` {/* #rcl-interfaces-rcl-interfaces-msg-listparametersresult-msg */}

`rcl_interfaces/rcl_interfaces/msg/ListParametersResult.msg`

```rosidl
# The resulting parameters under the given prefixes.
string[] names

# The resulting prefixes under the given prefixes.
# TODO(wjwwood): link to prefix definition and rules.
string[] prefixes
```

### Log `msg` {/* #rcl-interfaces-rcl-interfaces-msg-log-msg */}

`rcl_interfaces/rcl_interfaces/msg/Log.msg`

```rosidl
##
## Severity level constants
##
## These logging levels follow the Python Standard
## https://docs.python.org/3/library/logging.html#logging-levels
## And are implemented in rcutils as well
## https://github.com/ros2/rcutils/blob/35f29850064e0c33a4063cbc947ebbfeada11dba/include/rcutils/logging.h#L164-L172
## This leaves space for other standard logging levels to be inserted in the middle in the future,
## as well as custom user defined levels.
## Since there are several other logging enumeration standard for different implementations,
## other logging implementations may need to provide level mappings to match their internal implementations.
##

# Debug is for pedantic information, which is useful when debugging issues.
byte DEBUG=10

# Info is the standard informational level and is used to report expected
# information.
byte INFO=20

# Warning is for information that may potentially cause issues or possibly unexpected
# behavior.
byte WARN=30

# Error is for information that this node cannot resolve.
byte ERROR=40

# Information about a impending node shutdown.
byte FATAL=50

##
## Fields
##

# Timestamp when this message was generated by the node.
builtin_interfaces/Time stamp

# Corresponding log level, see above definitions.
uint8 level

# The name representing the logger this message came from.
string name

# The full log message.
string msg

# The file the message came from.
string file

# The function the message came from.
string function

# The line in the file the message came from.
uint32 line
```

### Parameter `msg` {/* #rcl-interfaces-rcl-interfaces-msg-parameter-msg */}

`rcl_interfaces/rcl_interfaces/msg/Parameter.msg`

```rosidl
# This is the message to communicate a parameter. It is an open struct with an enum in
# the descriptor to select which value is active.

# The full name of the parameter.
string name

# The parameter's value which can be one of several types, see
# `ParameterValue.msg` and `ParameterType.msg`.
ParameterValue value
```

### ParameterDescriptor `msg` {/* #rcl-interfaces-rcl-interfaces-msg-parameterdescriptor-msg */}

`rcl_interfaces/rcl_interfaces/msg/ParameterDescriptor.msg`

```rosidl
# This is the message to communicate a parameter's descriptor.

# The name of the parameter.
string name

# Enum values are defined in the `ParameterType.msg` message.
uint8 type

# Description of the parameter, visible from introspection tools.
string description

# Parameter constraints

# Plain English description of additional constraints which cannot be expressed
# with the available constraints, e.g. "only prime numbers".
#
# By convention, this should only be used to clarify constraints which cannot
# be completely expressed with the parameter constraints below.
string additional_constraints

# If 'true' then the value cannot change after it has been initialized.
bool read_only false

# If true, the parameter is allowed to change type.
bool dynamic_typing false

# If any of the following sequences are not empty, then the constraint inside of
# them apply to this parameter.
#
# FloatingPointRange and IntegerRange are mutually exclusive.

# FloatingPointRange consists of a from_value, a to_value, and a step.
FloatingPointRange[<=1] floating_point_range

# IntegerRange consists of a from_value, a to_value, and a step.
IntegerRange[<=1] integer_range
```

### ParameterEvent `msg` {/* #rcl-interfaces-rcl-interfaces-msg-parameterevent-msg */}

`rcl_interfaces/rcl_interfaces/msg/ParameterEvent.msg`

```rosidl
# This message contains a parameter event.
# Because the parameter event was an atomic update, a specific parameter name
# can only be in one of the three sets.

# The time stamp when this parameter event occurred.
builtin_interfaces/Time stamp

# Fully qualified ROS path to node.
string node

# New parameters that have been set for this node.
Parameter[] new_parameters

# Parameters that have been changed during this event.
Parameter[] changed_parameters

# Parameters that have been deleted during this event.
Parameter[] deleted_parameters
```

### ParameterEventDescriptors `msg` {/* #rcl-interfaces-rcl-interfaces-msg-parametereventdescriptors-msg */}

`rcl_interfaces/rcl_interfaces/msg/ParameterEventDescriptors.msg`

```rosidl
# This message contains descriptors of a parameter event.
# It was an atomic update.
# A specific parameter name can only be in one of the three sets.

ParameterDescriptor[] new_parameters
ParameterDescriptor[] changed_parameters
ParameterDescriptor[] deleted_parameters
```

### ParameterType `msg` {/* #rcl-interfaces-rcl-interfaces-msg-parametertype-msg */}

`rcl_interfaces/rcl_interfaces/msg/ParameterType.msg`

```rosidl
# These types correspond to the value that is set in the ParameterValue message.

# Default value, which implies this is not a valid parameter.
uint8 PARAMETER_NOT_SET=0

uint8 PARAMETER_BOOL=1
uint8 PARAMETER_INTEGER=2
uint8 PARAMETER_DOUBLE=3
uint8 PARAMETER_STRING=4
uint8 PARAMETER_BYTE_ARRAY=5
uint8 PARAMETER_BOOL_ARRAY=6
uint8 PARAMETER_INTEGER_ARRAY=7
uint8 PARAMETER_DOUBLE_ARRAY=8
uint8 PARAMETER_STRING_ARRAY=9
```

### ParameterValue `msg` {/* #rcl-interfaces-rcl-interfaces-msg-parametervalue-msg */}

`rcl_interfaces/rcl_interfaces/msg/ParameterValue.msg`

```rosidl
# Used to determine which of the next *_value fields are set.
# ParameterType.PARAMETER_NOT_SET indicates that the parameter was not set
# (if gotten) or is uninitialized.
# Values are enumerated in `ParameterType.msg`.

# The type of this parameter, which corresponds to the appropriate field below.
uint8 type

# "Variant" style storage of the parameter value. Only the value corresponding
# the type field will have valid information.

# Boolean value, can be either true or false.
bool bool_value

# Integer value ranging from -9,223,372,036,854,775,808 to
# 9,223,372,036,854,775,807.
int64 integer_value

# A double precision floating point value following IEEE 754.
float64 double_value

# A textual value with no practical length limit.
string string_value

# An array of bytes, used for non-textual information.
byte[] byte_array_value

# An array of boolean values.
bool[] bool_array_value

# An array of 64-bit integer values.
int64[] integer_array_value

# An array of 64-bit floating point values.
float64[] double_array_value

# An array of string values.
string[] string_array_value
```

### SetParameters `srv` {/* #rcl-interfaces-rcl-interfaces-srv-setparameters-srv */}

`rcl_interfaces/rcl_interfaces/srv/SetParameters.srv`

```rosidl
# A list of parameters to set.
Parameter[] parameters

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
```

### SetParametersAtomically `srv` {/* #rcl-interfaces-rcl-interfaces-srv-setparametersatomically-srv */}

`rcl_interfaces/rcl_interfaces/srv/SetParametersAtomically.srv`

```rosidl
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
```

### SetParametersResult `msg` {/* #rcl-interfaces-rcl-interfaces-msg-setparametersresult-msg */}

`rcl_interfaces/rcl_interfaces/msg/SetParametersResult.msg`

```rosidl
# A true value of the same index indicates that the parameter was set
# successfully. A false value indicates the change was rejected.
bool successful

# Reason why the setting was either successful or a failure. This should only be
# used for logging and user interfaces.
string reason
```

## rosgraph_msgs

### Clock `msg` {/* #rcl-interfaces-rosgraph-msgs-msg-clock-msg */}

`rcl_interfaces/rosgraph_msgs/msg/Clock.msg`

```rosidl
# This message communicates the current time.
#
# For more information, see https://design.ros2.org/articles/clock_and_time.html.
builtin_interfaces/Time clock
```

## semantic_map

### GetObjectContext `srv` {/* #semantic-map-srv-getobjectcontext-srv */}

`semantic_map/srv/GetObjectContext.srv`

```rosidl
# robonix/system/scene/get_object_context — return one object's node
# info, its direct relations, and nearby objects sorted by distance.
string object_id
---
SceneGraphNode object
SceneGraphEdge[] relations
Object[] nearby_objects
```

### GetRobotContext `srv` {/* #semantic-map-srv-getrobotcontext-srv */}

`semantic_map/srv/GetRobotContext.srv`

```rosidl
# Return the current spatial context used by Pilot before planning.
---
bool pose_known
string map_id
float64 x
float64 y
float64 z
float64 yaw
string room_id
string room_name
string[] containing_area_ids
string[] containing_area_names
Object[] nearby_objects
float64 observed_at_unix
float64 snapshot_at_unix
bool stale
string reason
```

### GetSceneGraph `srv` {/* #semantic-map-srv-getscenegraph-srv */}

`semantic_map/srv/GetSceneGraph.srv`

```rosidl
# robonix/system/scene/get_scene_graph — return the current scene graph
# snapshot with all stable nodes and their inferred relations.
---
SceneGraphNode[] nodes
SceneGraphEdge[] edges
SceneAnnotation[] annotations
float64 updated_at
```

### GoalNear `srv` {/* #semantic-map-srv-goalnear-srv */}

`semantic_map/srv/GoalNear.srv`

```rosidl
# robonix/system/scene/goal_near — find a navigation-safe approach pose
# near a registered physical scene object. Room annotations are rejected;
# use robonix/system/scene/goal_room for rooms. Map-frame (x, y, yaw); pass to
# robonix/service/navigation/navigate after.
#
# `reachable=false` when the occupancy grid has no free cell within
# the (service-side-default) search radius of the object — caller
# should pick a different target. `reason` is a short human-readable
# string for both branches.

string object_id
---
bool reachable
float64 x
float64 y
float64 yaw
string reason
```

### GoalRoom `srv` {/* #semantic-map-srv-goalroom-srv */}

`semantic_map/srv/GoalRoom.srv`

```rosidl
# robonix/system/scene/goal_room - find a navigation-safe pose inside
# a user-defined room annotation. The returned map-frame pose is always
# inside the room polygon; pass it to navigation/navigate.

string room_id
---
bool reachable
float64 x
float64 y
float64 yaw
string reason
```

### ListObjects `srv` {/* #semantic-map-srv-listobjects-srv */}

`semantic_map/srv/ListObjects.srv`

```rosidl
# robonix/system/scene/list_objects — return every object the scene
# registry currently believes exists. No filters, no scoping; the LLM
# calls this every round to ground its world model and filters
# client-side by label / distance / etc. (cheaper than baking those
# knobs into the schema).
---
Object[] objects
float64 stamp_unix
```

### ListRelations `srv` {/* #semantic-map-srv-listrelations-srv */}

`semantic_map/srv/ListRelations.srv`

```rosidl
# robonix/system/scene/list_relations — list scene graph edges,
# optionally filtered by relation type. Pass empty string for all.
string relation
---
SceneGraphEdge[] edges
```

### Object `msg` {/* #semantic-map-msg-object-msg */}

`semantic_map/msg/Object.msg`

```rosidl
# One tracked thing in the world. Seven primitives — no nested types,
# no enums. Position is map-frame xyz + yaw (radians, CCW around +z).
# The robot itself appears here with its current map-frame pose;
# yaw is meaningful for it. For passive objects yaw is the best-effort
# orientation the registry has (0.0 if unknown). The registry only
# emits objects it currently believes exist (no missing flag, no
# observation_count: those are internal telemetry, not API).

string id
string label
float64 x
float64 y
float64 z
float64 yaw
float64 last_seen_unix
```

### SceneAnnotation `msg` {/* #semantic-map-msg-sceneannotation-msg */}

`semantic_map/msg/SceneAnnotation.msg`

```rosidl
# User-authored semantic annotation anchored to the SLAM map.
# Rooms and POIs are maintained by scene, persisted per map_id, and
# returned with the scene graph so clients/agents can reason about
# human-provided regions as first-class scene context.

string annotation_id
string kind
string name
float64[] points_xy
float64 theta
bool stale
string stale_reason
float64 updated_at_unix
```

### SceneGraphEdge `msg` {/* #semantic-map-msg-scenegraphedge-msg */}

`semantic_map/msg/SceneGraphEdge.msg`

```rosidl
# Directed relation between two scene graph nodes.
# relation is one of: near, on_top_of, under, inside, contains,
# attached_to, part_of, same_object.

string source_id
string target_id
string relation
float64 confidence
string reason
```

### SceneGraphNode `msg` {/* #semantic-map-msg-scenegraphnode-msg */}

`semantic_map/msg/SceneGraphNode.msg`

```rosidl
# One node in the scene graph. Corresponds to a stable object in the
# scene registry, enriched with a caption and observation metadata.

string object_id
string label
string caption
float64 x
float64 y
float64 z
float64 confidence
int32 observation_count
float64 last_seen_unix
```

## sensor_msgs

### BatteryState `msg` {/* #common-interfaces-sensor-msgs-msg-batterystate-msg */}

`common_interfaces/sensor_msgs/msg/BatteryState.msg`

```rosidl

# Constants are chosen to match the enums in the linux kernel
# defined in include/linux/power_supply.h as of version 3.7
# The one difference is for style reasons the constants are
# all uppercase not mixed case.

# Power supply status constants
uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL = 4

# Power supply health constants
uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
uint8 POWER_SUPPLY_HEALTH_GOOD = 1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
uint8 POWER_SUPPLY_HEALTH_DEAD = 3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
uint8 POWER_SUPPLY_HEALTH_COLD = 6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

# Power supply technology (chemistry) constants
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6

std_msgs/Header  header
float32 voltage          # Voltage in Volts (Mandatory)
float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
float32 current          # Negative when discharging (A)  (If unmeasured NaN)
float32 charge           # Current charge in Ah  (If unmeasured NaN)
float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
uint8   power_supply_status     # The charging status as reported. Values defined above
uint8   power_supply_health     # The battery health metric. Values defined above
uint8   power_supply_technology # The battery chemistry. Values defined above
bool    present          # True if the battery is present

float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
                         # If individual voltages unknown but number of cells known set each to NaN
float32[] cell_temperature # An array of individual cell temperatures for each cell in the pack
                           # If individual temperatures unknown but number of cells known set each to NaN
string location          # The location into which the battery is inserted. (slot number or plug)
string serial_number     # The best approximation of the battery serial number
```

### CameraInfo `msg` {/* #common-interfaces-sensor-msgs-msg-camerainfo-msg */}

`common_interfaces/sensor_msgs/msg/CameraInfo.msg`

```rosidl
# This message defines meta information for a camera. It should be in a
# camera namespace on topic "camera_info" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc)
# for producing the four processed image topics from image_raw and
# camera_info. The meaning of the camera parameters are described in
# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to
# common operations using this meta information. If you want to, e.g.,
# project a 3d point into image coordinates, we strongly recommend
# using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left
# zeroed out. In particular, clients may assume that K[0] == 0.0
# indicates an uncalibrated camera.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of camera
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may "recalibrate" frequently.              #
#                                                                     #
# The internal parameters can be used to warp a raw (distorted) image #
# to:                                                                 #
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)                           #
# The projection matrix P projects 3D points into the rectified image.#
#######################################################################

# The image dimensions with which the camera was calibrated.
# Normally this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.hpp. For most cameras, "plumb_bob" - a
# simple model of radial and tangential distortion - is sufficent.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] d

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  k # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  r # 3x3 row-major matrix

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.
float64[12] p # 3x4 row-major matrix


#######################################################################
#                      Operational Parameters                         #
#######################################################################
# These define the image region actually captured by the camera       #
# driver. Although they affect the geometry of the output image, they #
# may be changed freely without recalibrating the camera.             #
#######################################################################

# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), given in
#  full resolution (unbinned) image coordinates. A particular ROI
#  always denotes the same window of pixels on the camera sensor,
#  regardless of binning settings.
# The default setting of roi (all values 0) is considered the same as
#  full resolution (roi.width = width, roi.height = height).
RegionOfInterest roi
```

### ChannelFloat32 `msg` {/* #common-interfaces-sensor-msgs-msg-channelfloat32-msg */}

`common_interfaces/sensor_msgs/msg/ChannelFloat32.msg`

```rosidl
# This message is used by the PointCloud message to hold optional data
# associated with each point in the cloud. The length of the values
# array should be the same as the length of the points array in the
# PointCloud, and each value should be associated with the corresponding
# point.
#
# Channel names in existing practice include:
#   "u", "v" - row and column (respectively) in the left stereo image.
#              This is opposite to usual conventions but remains for
#              historical reasons. The newer PointCloud2 message has no
#              such problem.
#   "rgb" - For point clouds produced by color stereo cameras. uint8
#           (R,G,B) values packed into the least significant 24 bits,
#           in order.
#   "intensity" - laser or pixel intensity.
#   "distance"

# The channel name should give semantics of the channel (e.g.
# "intensity" instead of "value").
string name

# The values array should be 1-1 with the elements of the associated
# PointCloud.
float32[] values
```

### CompressedImage `msg` {/* #common-interfaces-sensor-msgs-msg-compressedimage-msg */}

`common_interfaces/sensor_msgs/msg/CompressedImage.msg`

```rosidl
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
```

### FluidPressure `msg` {/* #common-interfaces-sensor-msgs-msg-fluidpressure-msg */}

`common_interfaces/sensor_msgs/msg/FluidPressure.msg`

```rosidl
# Single pressure reading.  This message is appropriate for measuring the
# pressure inside of a fluid (air, water, etc).  This also includes
# atmospheric or barometric pressure.
#
# This message is not appropriate for force/pressure contact sensors.

std_msgs/Header header # timestamp of the measurement
                             # frame_id is the location of the pressure sensor

float64 fluid_pressure       # Absolute pressure reading in Pascals.

float64 variance             # 0 is interpreted as variance unknown
```

### Illuminance `msg` {/* #common-interfaces-sensor-msgs-msg-illuminance-msg */}

`common_interfaces/sensor_msgs/msg/Illuminance.msg`

```rosidl
# Single photometric illuminance measurement.  Light should be assumed to be
# measured along the sensor's x-axis (the area of detection is the y-z plane).
# The illuminance should have a 0 or positive value and be received with
# the sensor's +X axis pointing toward the light source.
#
# Photometric illuminance is the measure of the human eye's sensitivity of the
# intensity of light encountering or passing through a surface.
#
# All other Photometric and Radiometric measurements should not use this message.
# This message cannot represent:
#  - Luminous intensity (candela/light source output)
#  - Luminance (nits/light output per area)
#  - Irradiance (watt/area), etc.

std_msgs/Header header # timestamp is the time the illuminance was measured
                             # frame_id is the location and direction of the reading

float64 illuminance          # Measurement of the Photometric Illuminance in Lux.

float64 variance             # 0 is interpreted as variance unknown
```

### Image `msg` {/* #common-interfaces-sensor-msgs-msg-image-msg */}

`common_interfaces/sensor_msgs/msg/Image.msg`

```rosidl
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image
                             # If the frame_id here and the frame_id of the CameraInfo
                             # message associated with the image conflict
                             # the behavior is undefined

uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.ros.org and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
```

### Imu `msg` {/* #common-interfaces-sensor-msgs-msg-imu-msg */}

`common_interfaces/sensor_msgs/msg/Imu.msg`

```rosidl
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.

std_msgs/Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z
```

### JointState `msg` {/* #common-interfaces-sensor-msgs-msg-jointstate-msg */}

`common_interfaces/sensor_msgs/msg/JointState.msg`

```rosidl
# This is a message that holds data to describe the state of a set of torque controlled joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

std_msgs/Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
```

### Joy `msg` {/* #common-interfaces-sensor-msgs-msg-joy-msg */}

`common_interfaces/sensor_msgs/msg/Joy.msg`

```rosidl
# Reports the state of a joystick's axes and buttons.

# The timestamp is the time at which data is received from the joystick.
std_msgs/Header header

# The axes measurements from a joystick.
float32[] axes

# The buttons measurements from a joystick.
int32[] buttons
```

### JoyFeedback `msg` {/* #common-interfaces-sensor-msgs-msg-joyfeedback-msg */}

`common_interfaces/sensor_msgs/msg/JoyFeedback.msg`

```rosidl
# Declare of the type of feedback
uint8 TYPE_LED    = 0
uint8 TYPE_RUMBLE = 1
uint8 TYPE_BUZZER = 2

uint8 type

# This will hold an id number for each type of each feedback.
# Example, the first led would be id=0, the second would be id=1
uint8 id

# Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is
# actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.
float32 intensity
```

### JoyFeedbackArray `msg` {/* #common-interfaces-sensor-msgs-msg-joyfeedbackarray-msg */}

`common_interfaces/sensor_msgs/msg/JoyFeedbackArray.msg`

```rosidl
# This message publishes values for multiple feedback at once.
JoyFeedback[] array
```

### LaserEcho `msg` {/* #common-interfaces-sensor-msgs-msg-laserecho-msg */}

`common_interfaces/sensor_msgs/msg/LaserEcho.msg`

```rosidl
# This message is a submessage of MultiEchoLaserScan and is not intended
# to be used separately.

float32[] echoes  # Multiple values of ranges or intensities.
                  # Each array represents data from the same angle increment.
```

### LaserScan `msg` {/* #common-interfaces-sensor-msgs-msg-laserscan-msg */}

`common_interfaces/sensor_msgs/msg/LaserScan.msg`

```rosidl
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
```

### MagneticField `msg` {/* #common-interfaces-sensor-msgs-msg-magneticfield-msg */}

`common_interfaces/sensor_msgs/msg/MagneticField.msg`

```rosidl
# Measurement of the Magnetic Field vector at a specific location.
#
# If the covariance of the measurement is known, it should be filled in.
# If all you know is the variance of each measurement, e.g. from the datasheet,
# just put those along the diagonal.
# A covariance matrix of all zeros will be interpreted as "covariance unknown",
# and to use the data a covariance will have to be assumed or gotten from some
# other source.

std_msgs/Header header               # timestamp is the time the
                                           # field was measured
                                           # frame_id is the location and orientation
                                           # of the field measurement

geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
                                           # field vector in Tesla
                                           # If your sensor does not output 3 axes,
                                           # put NaNs in the components not reported.

float64[9] magnetic_field_covariance       # Row major about x, y, z axes
                                           # 0 is interpreted as variance unknown
```

### MultiDOFJointState `msg` {/* #common-interfaces-sensor-msgs-msg-multidofjointstate-msg */}

`common_interfaces/sensor_msgs/msg/MultiDOFJointState.msg`

```rosidl
# Representation of state for joints with multiple degrees of freedom,
# following the structure of JointState which can only represent a single degree of freedom.
#
# It is assumed that a joint in a system corresponds to a transform that gets applied
# along the kinematic chain. For example, a planar joint (as in URDF) is 3DOF (x, y, yaw)
# and those 3DOF can be expressed as a transformation matrix, and that transformation
# matrix can be converted back to (x, y, yaw)
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# wrench associated with them, you can leave the wrench array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

std_msgs/Header header

string[] joint_names
geometry_msgs/Transform[] transforms
geometry_msgs/Twist[] twist
geometry_msgs/Wrench[] wrench
```

### MultiEchoLaserScan `msg` {/* #common-interfaces-sensor-msgs-msg-multiecholaserscan-msg */}

`common_interfaces/sensor_msgs/msg/MultiEchoLaserScan.msg`

```rosidl
# Single scan from a multi-echo planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

LaserEcho[] ranges           # range data [m]
                             # (Note: NaNs, values < range_min or > range_max should be discarded)
                             # +Inf measurements are out of range
                             # -Inf measurements are too close to determine exact distance.
LaserEcho[] intensities      # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
```

### NavSatFix `msg` {/* #common-interfaces-sensor-msgs-msg-navsatfix-msg */}

`common_interfaces/sensor_msgs/msg/NavSatFix.msg`

```rosidl
# Navigation Satellite fix for any Global Navigation Satellite System
#
# Specified using the WGS 84 reference ellipsoid

# header.stamp specifies the ROS time for this measurement (the
#        corresponding satellite time may be reported using the
#        sensor_msgs/TimeReference message).
#
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
std_msgs/Header header

# Satellite fix status information.
NavSatStatus status

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Position covariance [m^2] defined relative to a tangential plane
# through the reported position. The components are East, North, and
# Up (ENU), in row-major order.
#
# Beware: this coordinate system exhibits singularities at the poles.
float64[9] position_covariance

# If the covariance of the fix is known, fill it in completely. If the
# GPS receiver provides the variance of each measurement, put them
# along the diagonal. If only Dilution of Precision is available,
# estimate an approximate covariance from that.

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 position_covariance_type
```

### NavSatStatus `msg` {/* #common-interfaces-sensor-msgs-msg-navsatstatus-msg */}

`common_interfaces/sensor_msgs/msg/NavSatStatus.msg`

```rosidl
# Navigation Satellite fix status for any Global Navigation Satellite System.
#
# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.  A
# fix is valid when status >= STATUS_FIX.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.

uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service
```

### PointCloud `msg` {/* #common-interfaces-sensor-msgs-msg-pointcloud-msg */}

`common_interfaces/sensor_msgs/msg/PointCloud.msg`

```rosidl
## THIS MESSAGE IS DEPRECATED AS OF FOXY
## Please use sensor_msgs/PointCloud2

# This message holds a collection of 3d points, plus optional additional
# information about each point.

# Time of sensor data acquisition, coordinate frame ID.
std_msgs/Header header

# Array of 3d points. Each Point32 should be interpreted as a 3d point
# in the frame given in the header.
geometry_msgs/Point32[] points

# Each channel should have the same number of elements as points array,
# and the data in each channel should correspond 1:1 with each point.
# Channel names in common practice are listed in ChannelFloat32.msg.
ChannelFloat32[] channels
```

### PointCloud2 `msg` {/* #common-interfaces-sensor-msgs-msg-pointcloud2-msg */}

`common_interfaces/sensor_msgs/msg/PointCloud2.msg`

```rosidl
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.
#
# The point cloud data may be organized 2d (image-like) or 1d (unordered).
# Point clouds organized as 2d images may be produced by camera depth sensors
# such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
std_msgs/Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
```

### PointField `msg` {/* #common-interfaces-sensor-msgs-msg-pointfield-msg */}

`common_interfaces/sensor_msgs/msg/PointField.msg`

```rosidl
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

# Common PointField names are x, y, z, intensity, rgb, rgba
string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
```

### Range `msg` {/* #common-interfaces-sensor-msgs-msg-range-msg */}

`common_interfaces/sensor_msgs/msg/Range.msg`

```rosidl
# Single range reading from an active ranger that emits energy and reports
# one range reading that is valid along an arc at the distance measured.
# This message is  not appropriate for laser scanners. See the LaserScan
# message if you are working with a laser scanner.
#
# This message also can represent a fixed-distance (binary) ranger.  This
# sensor will have min_range===max_range===distance of detection.
# These sensors follow REP 117 and will output -Inf if the object is detected
# and +Inf if the object is outside of the detection range.

std_msgs/Header header # timestamp in the header is the time the ranger
                             # returned the distance reading

# Radiation type enums
# If you want a value added to this list, send an email to the ros-users list
uint8 ULTRASOUND=0
uint8 INFRARED=1

uint8 radiation_type    # the type of radiation used by the sensor
                        # (sound, IR, etc) [enum]

float32 field_of_view   # the size of the arc that the distance reading is
                        # valid for [rad]
                        # the object causing the range reading may have
                        # been anywhere within -field_of_view/2 and
                        # field_of_view/2 at the measured range.
                        # 0 angle corresponds to the x-axis of the sensor.

float32 min_range       # minimum range value [m]
float32 max_range       # maximum range value [m]
                        # Fixed distance rangers require min_range==max_range

float32 range           # range data [m]
                        # (Note: values < range_min or > range_max should be discarded)
                        # Fixed distance rangers only output -Inf or +Inf.
                        # -Inf represents a detection within fixed distance.
                        # (Detection too close to the sensor to quantify)
                        # +Inf represents no detection within the fixed distance.
                        # (Object out of range)
```

### RegionOfInterest `msg` {/* #common-interfaces-sensor-msgs-msg-regionofinterest-msg */}

`common_interfaces/sensor_msgs/msg/RegionOfInterest.msg`

```rosidl
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
```

### RelativeHumidity `msg` {/* #common-interfaces-sensor-msgs-msg-relativehumidity-msg */}

`common_interfaces/sensor_msgs/msg/RelativeHumidity.msg`

```rosidl
# Single reading from a relative humidity sensor.
# Defines the ratio of partial pressure of water vapor to the saturated vapor
# pressure at a temperature.

std_msgs/Header header # timestamp of the measurement
                             # frame_id is the location of the humidity sensor

float64 relative_humidity    # Expression of the relative humidity
                             # from 0.0 to 1.0.
                             # 0.0 is no partial pressure of water vapor
                             # 1.0 represents partial pressure of saturation

float64 variance             # 0 is interpreted as variance unknown
```

### SetCameraInfo `srv` {/* #common-interfaces-sensor-msgs-srv-setcamerainfo-srv */}

`common_interfaces/sensor_msgs/srv/SetCameraInfo.srv`

```rosidl
# This service requests that a camera stores the given CameraInfo as that
# camera's calibration information.
#
# The width and height in the camera_info field should match what the
# camera is currently outputting on its camera_info topic, and the camera
# will assume that the region of the imager that is being referred to is
# the region that the camera is currently capturing.

sensor_msgs/CameraInfo camera_info # The camera_info to store
---
bool success                             # True if the call succeeded
string status_message                    # Used to give details about success
```

### Temperature `msg` {/* #common-interfaces-sensor-msgs-msg-temperature-msg */}

`common_interfaces/sensor_msgs/msg/Temperature.msg`

```rosidl
# Single temperature reading.

std_msgs/Header header # timestamp is the time the temperature was measured
                             # frame_id is the location of the temperature reading

float64 temperature          # Measurement of the Temperature in Degrees Celsius.

float64 variance             # 0 is interpreted as variance unknown.
```

### TimeReference `msg` {/* #common-interfaces-sensor-msgs-msg-timereference-msg */}

`common_interfaces/sensor_msgs/msg/TimeReference.msg`

```rosidl
# Measurement from an external time source not actively synchronized with the system clock.

std_msgs/Header header      # stamp is system time for which measurement was valid
                                  # frame_id is not used

builtin_interfaces/Time time_ref  # corresponding time from this external source
string source                     # (optional) name of time source
```

## shape_msgs

### Mesh `msg` {/* #common-interfaces-shape-msgs-msg-mesh-msg */}

`common_interfaces/shape_msgs/msg/Mesh.msg`

```rosidl
# Definition of a mesh.

# List of triangles; the index values refer to positions in vertices[].
MeshTriangle[] triangles

# The actual vertices that make up the mesh.
geometry_msgs/Point[] vertices
```

### MeshTriangle `msg` {/* #common-interfaces-shape-msgs-msg-meshtriangle-msg */}

`common_interfaces/shape_msgs/msg/MeshTriangle.msg`

```rosidl
# Definition of a triangle's vertices.

uint32[3] vertex_indices
```

### Plane `msg` {/* #common-interfaces-shape-msgs-msg-plane-msg */}

`common_interfaces/shape_msgs/msg/Plane.msg`

```rosidl
# Representation of a plane, using the plane equation ax + by + cz + d = 0.
#
# a := coef[0]
# b := coef[1]
# c := coef[2]
# d := coef[3]
float64[4] coef
```

### SolidPrimitive `msg` {/* #common-interfaces-shape-msgs-msg-solidprimitive-msg */}

`common_interfaces/shape_msgs/msg/SolidPrimitive.msg`

```rosidl
# Defines box, sphere, cylinder, cone and prism.
# All shapes are defined to have their bounding boxes centered around 0,0,0.

uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4
uint8 PRISM=5

# The type of the shape
uint8 type

# The dimensions of the shape
float64[<=3] dimensions  # At no point will dimensions have a length > 3.

# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array.

# For type BOX, the X, Y, and Z dimensions are the length of the corresponding sides of the box.
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2

# For the SPHERE type, only one component is used, and it gives the radius of the sphere.
uint8 SPHERE_RADIUS=0

# For the CYLINDER and CONE types, the center line is oriented along the Z axis.
# Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component of dimensions gives the
# height of the cylinder (cone).
# The CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the radius of
# the base of the cylinder (cone).
# Cone and cylinder primitives are defined to be circular. The tip of the cone
# is pointing up, along +Z axis.

uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1

uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1

# For the type PRISM, the center line is oriented along Z axis.
# The PRISM_HEIGHT component of dimensions gives the
# height of the prism.
# The polygon defines the Z axis centered base of the prism.
# The prism is constructed by extruding the base in +Z and -Z
# directions by half of the PRISM_HEIGHT
# Only x and y fields of the points are used in the polygon.
# Points of the polygon are ordered counter-clockwise.

uint8 PRISM_HEIGHT=0
geometry_msgs/Polygon polygon
```

## soma

### ActuatorState `msg` {/* #soma-msg-actuatorstate-msg */}

`soma/msg/ActuatorState.msg`

```rosidl
string component_id
string joint_name

Scalar position
Scalar velocity
Scalar effort
Scalar current
Scalar voltage
Scalar motor_temp
Scalar driver_temp

bool torque_enabled
bool brake_engaged
bool communication_ok

uint32 vendor_mode
uint32 vendor_error_code
uint32 status_flags
```

### ComponentStatus `msg` {/* #soma-msg-componentstatus-msg */}

`soma/msg/ComponentStatus.msg`

```rosidl
uint8 HEALTH_OK=0
uint8 HEALTH_WARN=1
uint8 HEALTH_ERROR=2
uint8 HEALTH_STALE=3
uint8 HEALTH_UNKNOWN=4

uint8 KIND_UNKNOWN=0
uint8 KIND_BODY=1
uint8 KIND_ARM=2
uint8 KIND_LEG=3
uint8 KIND_JOINT=4
uint8 KIND_WHEEL=5
uint8 KIND_GRIPPER=6
uint8 KIND_BATTERY=7
uint8 KIND_COMPUTER=8
uint8 KIND_SENSOR=9
uint8 KIND_CONTROLLER=10
uint8 KIND_END_EFFECTOR=11

uint8 OP_UNKNOWN=0
uint8 OP_OFFLINE=1
uint8 OP_POWERED_OFF=2
uint8 OP_IDLE=3
uint8 OP_ACTIVE=4
uint8 OP_DEGRADED=5
uint8 OP_PROTECTIVE_STOP=6
uint8 OP_ESTOP=7
uint8 OP_FAULT=8

string id
string parent_id
uint8 kind
string name
string frame_id
string model
string serial
uint8 health
uint8 operational_state
bool present
bool online
string detail
```

### FaultState `msg` {/* #soma-msg-faultstate-msg */}

`soma/msg/FaultState.msg`

```rosidl
uint8 INFO=0
uint8 WARN=1
uint8 ERROR=2
uint8 CRITICAL=3

string component_id
string fault_id
uint8 severity
bool active
bool clearable
int64 onset_ts_ns

uint32 vendor_code
string vendor_code_text
string message
string[] attributes
string vendor_raw_json
```

### GetDescription `srv` {/* #soma-srv-getdescription-srv */}

`soma/srv/GetDescription.srv`

```rosidl
# Get the robot's URDF + high-level metadata. Consumers parse the
# URDF with their preferred lib (urdf_parser_py / KDL / Pinocchio).
# soma loads the URDF once at startup and serves the same string;
# don't hammer this — it doesn't change at runtime.
---
string urdf_xml          # full URDF contents (utf-8)
string model_name        # e.g. "ranger_mini_v2", "tiago"
float64 mass_kg          # sum of all link masses; -1 if unknown
string base_frame        # convention frame for kinematic root, e.g. "base_link"
```

### GetFootprint `srv` {/* #soma-srv-getfootprint-srv */}

`soma/srv/GetFootprint.srv`

```rosidl
# Get the robot's 2D footprint polygon in the base frame. Used by
# nav2_wrapper for costmap inflation, by simple_nav for the inscribed
# radius, by collision-monitor services for anti-bump checks. Closed
# polygon — first point is NOT repeated as last; consumers close it
# themselves.
#
# Polygon is computed once at soma startup from the URDF (axis-aligned
# convex hull of every link mesh's projection onto the base plane).
# Override path: deploy manifest's `config.footprint_xy_pts` lets you
# hand-supply a polygon when the URDF mesh hull is too generous (e.g.
# Tiago's torso overhang is way wider than its base).
---
geometry_msgs/Point[] points    # CCW-ordered vertices in `base_frame`, z=0
string base_frame               # e.g. "base_link"
float64 inscribed_radius_m      # largest disc fitting inside the polygon
float64 circumscribed_radius_m  # smallest disc enclosing the polygon
```

### GetHealth `srv` {/* #soma-srv-gethealth-srv */}

`soma/srv/GetHealth.srv`

```rosidl
---
SomaHealthSnapshot snapshot
```

### GetSensorExtrinsics `srv` {/* #soma-srv-getsensorextrinsics-srv */}

`soma/srv/GetSensorExtrinsics.srv`

```rosidl
# Get every sensor's static mount transform (read from URDF). Replaces
# the per-primitive `primitive/<area>/extrinsics` contract pattern with
# a single source of truth — the body description package owns the
# kinematic chain, primitives just publish their data streams.
#
# Filter via `sensor_kind` to narrow the response (empty = all):
#   ""        → every sensor in the URDF
#   "camera"  → just RGBD / mono cameras
#   "lidar"   → just 2D / 3D lidars
#   "imu"     → just imus
string sensor_kind  # filter; empty = all
---
soma/SensorExtrinsic[] sensors
```

### GetUrdf `srv` {/* #soma-srv-geturdf-srv */}

`soma/srv/GetUrdf.srv`

```rosidl
# Return the URDF text referenced by one loaded robot's Soma YAML.
string robot_id  # empty = default robot
---
string robot_id
string urdf_xml
```

### GetYaml `srv` {/* #soma-srv-getyaml-srv */}

`soma/srv/GetYaml.srv`

```rosidl
# Return the raw Soma YAML text for one loaded robot.
string robot_id  # empty = default robot
---
string robot_id
string yaml_text
```

### Metric `msg` {/* #soma-msg-metric-msg */}

`soma/msg/Metric.msg`

```rosidl
string component_id
string name
Scalar value
string source_key
```

### PowerSourceState `msg` {/* #soma-msg-powersourcestate-msg */}

`soma/msg/PowerSourceState.msg`

```rosidl
string component_id

Scalar soc_percent
Scalar soh_percent
Scalar voltage
Scalar current
Scalar temperature
Scalar remaining_s

uint32 cycle_count
Scalar[] cell_voltages
uint32 vendor_status_code
```

### SafetyEndpointState `msg` {/* #soma-msg-safetyendpointstate-msg */}

`soma/msg/SafetyEndpointState.msg`

```rosidl
uint8 TYPE_UNKNOWN=0
uint8 TYPE_HARDWARE=1
uint8 TYPE_SOFTWARE=2
uint8 TYPE_REMOTE=3
uint8 TYPE_PAYLOAD=4

uint8 RELEASED=0
uint8 ACTIVE=1

string name
uint8 type
uint8 state
string detail
```

### SafetyState `msg` {/* #soma-msg-safetystate-msg */}

`soma/msg/SafetyState.msg`

```rosidl
uint8 UNKNOWN=0
uint8 NORMAL=1
uint8 REDUCED=2
uint8 PROTECTIVE_STOP=3
uint8 ESTOP=4
uint8 FAULT=5

bool motion_allowed
bool motor_power_allowed
uint8 aggregate_state
string detail
```

### Scalar `msg` {/* #soma-msg-scalar-msg */}

`soma/msg/Scalar.msg`

```rosidl
uint8 VALID=0
uint8 STALE=1
uint8 UNAVAILABLE=2
uint8 INVALID=3

float64 value
string unit
uint8 quality
```

### SensorExtrinsic `msg` {/* #soma-msg-sensorextrinsic-msg */}

`soma/msg/SensorExtrinsic.msg`

```rosidl
# One static-mounted sensor's extrinsics: where it sits relative to a
# parent frame (typically `base_link`). Read from the URDF at soma
# startup; consumers (scene's camera-to-world projection, multi-cam
# fusion) call `system/soma/sensor_extrinsics` instead of looking up
# tf2 themselves so the dependency graph is auditable via atlas.

string sensor_name        # e.g. "rgb_camera", "depth_camera", "lidar3d"
string parent_frame       # e.g. "base_link"
string child_frame        # e.g. "camera_color_optical_frame"
geometry_msgs/Transform transform   # parent → child
```

### SomaHealthSnapshot `msg` {/* #soma-msg-somahealthsnapshot-msg */}

`soma/msg/SomaHealthSnapshot.msg`

```rosidl
uint32 schema_version
string body_id
uint64 seq
int64 source_ts_ns
int64 soma_ts_ns
uint32 ttl_ms

ComponentStatus[] components
ActuatorState[] actuators
PowerSourceState[] power_sources
SafetyState safety
SafetyEndpointState[] safety_endpoints
FaultState[] faults
Metric[] metrics
```

### StreamHealth `srv` {/* #soma-srv-streamhealth-srv */}

`soma/srv/StreamHealth.srv`

```rosidl
---
SomaHealthSnapshot snapshot
```

## speech

### DetectWakeWord `srv` {/* #speech-srv-detectwakeword-srv */}

`speech/srv/DetectWakeWord.srv`

```rosidl
# Stream microphone PCM to the speech service until a configured wake phrase is detected.
audio/msg/AudioChunk chunk
---
bool detected
string keyword
float32 confidence
string error
```

### DialogEvent `msg` {/* #speech-msg-dialogevent-msg */}

`speech/msg/DialogEvent.msg`

```rosidl
# Event emitted by the speech dialog service during a voice interaction session.
# Streamed back to the caller (e.g. Liaison) over the dialog RPC stream.

uint64 timestamp_ns       # nanoseconds since epoch
string session_id

uint8 VAD_EVENT       = 0
uint8 PARTIAL_TRANSCRIPT = 1
uint8 FINAL_TRANSCRIPT   = 2
uint8 BOT_RESPONSE_TEXT  = 3
uint8 BOT_AUDIO_CHUNK    = 4
uint8 STATE_CHANGE       = 5
uint8 ERROR              = 6
uint8 event_type

# VAD event data
speech/msg/VadEvent vad_event

# Transcript data (for PARTIAL_TRANSCRIPT and FINAL_TRANSCRIPT)
string text
float32 confidence
string language           # detected or specified language

# Bot response text (for BOT_RESPONSE_TEXT)
string response_text

# Audio data (for BOT_AUDIO_CHUNK)
audio/msg/AudioChunk audio_chunk

# State change (for STATE_CHANGE)
speech/msg/DialogState dialog_state

# Error info (for ERROR)
string error
```

### DialogRequest `msg` {/* #speech-msg-dialogrequest-msg */}

`speech/msg/DialogRequest.msg`

```rosidl
# Dialog request sent by client to start a voice dialog session.
string language           # BCP-47, e.g. "zh-CN", "" = auto-detect
string voice_id           # preferred voice, "" = default
bool   enable_vad         # enable voice activity detection
float32 vad_silence_threshold_s  # seconds of silence before ending speech (default 0.8)
float32 vad_energy_threshold     # energy threshold for VAD (0.0 = auto)
audio/msg/AudioConfig audio_config  # desired audio format
```

### DialogState `msg` {/* #speech-msg-dialogstate-msg */}

`speech/msg/DialogState.msg`

```rosidl
# State of a voice dialog session.
# Used to track the lifecycle of a voice interaction.

string session_id
uint8 IDLE       = 0
uint8 LISTENING  = 1    # mic is active, waiting for speech
uint8 PROCESSING = 2    # ASR/NLP processing in progress
uint8 SPEAKING   = 3    # TTS is playing audio to speaker
uint8 ERROR      = 4
uint8 state
string language           # current BCP-47 language code
string error              # non-empty if state == ERROR
```

### GetDialogStatus `srv` {/* #speech-srv-getdialogstatus-srv */}

`speech/srv/GetDialogStatus.srv`

```rosidl
# Query current dialog session status.
string session_id
---
speech/msg/DialogState state
string error
```

### ListSpeakers `srv` {/* #speech-srv-listspeakers-srv */}

`speech/srv/ListSpeakers.srv`

```rosidl
# robonix/service/speech/list_speakers — enumerate every audio/speaker
# primitive registered in atlas, so a caller can choose a `target` for
# speak. `namespace_prefix` optionally filters by provider namespace.
# `speakers_json` is a JSON array of {provider_id, namespace, description}.
string namespace_prefix
---
string speakers_json
```

### SendAudio `srv` {/* #speech-srv-sendaudio-srv */}

`speech/srv/SendAudio.srv`

```rosidl
# Inject audio into an active dialog session (for non-streaming input).
string session_id
uint8[] audio_data
string encoding
uint32 sample_rate_hz
---
bool accepted
string error
```

### SendText `srv` {/* #speech-srv-sendtext-srv */}

`speech/srv/SendText.srv`

```rosidl
# Send a text-based message into the dialog (bypass ASR).
string session_id
string text
string language
---
bool accepted
string error
```

### SetLanguage `srv` {/* #speech-srv-setlanguage-srv */}

`speech/srv/SetLanguage.srv`

```rosidl
# Change language mid-session.
string session_id
string language           # BCP-47 code
---
bool success
string error
```

### Speak `srv` {/* #speech-srv-speak-srv */}

`speech/srv/Speak.srv`

```rosidl
# robonix/service/speech/speak — synthesize `text` to speech and play it
# out loud on the audio/speaker primitive identified by `target`
# (its provider_id, from list_speakers). Empty target = first available
# speaker. Lets an agent make the robot announce things aloud.
string target
string text
---
bool ok
string detail
```

### StartDialog `srv` {/* #speech-srv-startdialog-srv */}

`speech/srv/StartDialog.srv`

```rosidl
# Start a voice dialog session.
# Returns initial dialog state; events are streamed via the Dialog contract.
string language
bool   enable_vad
audio/msg/AudioConfig audio_config
---
speech/msg/DialogEvent event
```

### StopDialog `srv` {/* #speech-srv-stopdialog-srv */}

`speech/srv/StopDialog.srv`

```rosidl
# Stop an active voice dialog session.
string session_id
---
bool success
string error
```

### VadEvent `msg` {/* #speech-msg-vadevent-msg */}

`speech/msg/VadEvent.msg`

```rosidl
# Voice Activity Detection event.
# Emitted by the dialog manager or ASR service to signal speech state changes.

uint64 timestamp_ns       # nanoseconds since epoch
uint8 SPEECH_BEGIN = 0
uint8 SPEECH_END   = 1
uint8 SILENCE      = 2
uint8 event_type
float32 confidence       # 0.0 – 1.0 confidence of the VAD decision
float32 energy_level     # audio energy level (optional, 0.0 if unavailable)
```

## statistics_msgs

### MetricsMessage `msg` {/* #rcl-interfaces-statistics-msgs-msg-metricsmessage-msg */}

`rcl_interfaces/statistics_msgs/msg/MetricsMessage.msg`

```rosidl
#############################################
# A generic metrics message providing statistics for measurements from different sources. For example,
# measure a system's CPU % for a given window yields the following data points over a window of time:
#
#   - average cpu %
#   - std deviation
#   - min
#   - max
#   - sample count
#
# These are all represented as different 'StatisticDataPoint's.
#############################################

# Name metric measurement source, e.g., node, topic, or process name
string measurement_source_name

# Name of the metric being measured, e.g. cpu_percentage, free_memory_mb, message_age, etc.
string metrics_source

# Unit of measure of the metric, e.g. percent, mb, seconds, etc.
string unit

# Measurement window start time
builtin_interfaces/Time window_start

# Measurement window end time
builtin_interfaces/Time window_stop

# A list of statistics data point, defined in StatisticDataPoint.msg
StatisticDataPoint[] statistics
```

### StatisticDataPoint `msg` {/* #rcl-interfaces-statistics-msgs-msg-statisticdatapoint-msg */}

`rcl_interfaces/statistics_msgs/msg/StatisticDataPoint.msg`

```rosidl
#############################################
# This holds the structure of a single data point of a StatisticDataType.
#
# This message is used in MetricsStatisticsMessage, defined in MetricsStatisticsMessage.msg.
#
# Examples of the value of data point are
# - average size of messages received
# - standard deviation of the period of messages published
# - maximum age of messages published
#
# A value of nan represents no data is available.
# One example is that standard deviation is only available when there are two or more data points but there is only one,
# and in this case the value would be nan.
# +inf and -inf are not allowed.
#
#############################################

# The statistic type of this data point, defined in StatisticDataType.msg
# Default value should be StatisticDataType.STATISTICS_DATA_TYPE_UNINITIALIZED (0).
uint8 data_type

# The value of the data point
float64 data
```

### StatisticDataType `msg` {/* #rcl-interfaces-statistics-msgs-msg-statisticdatatype-msg */}

`rcl_interfaces/statistics_msgs/msg/StatisticDataType.msg`

```rosidl
#############################################
# This file contains the commonly used constants for the statistics data type.
#
# The value 0 is reserved for unitialized statistic message data type.
# Range of values: [0, 255].
# Unallowed values: any value that is not specified in this file.
#
#############################################

# Constant for uninitialized
uint8 STATISTICS_DATA_TYPE_UNINITIALIZED = 0

# Allowed values
uint8 STATISTICS_DATA_TYPE_AVERAGE = 1
uint8 STATISTICS_DATA_TYPE_MINIMUM = 2
uint8 STATISTICS_DATA_TYPE_MAXIMUM = 3
uint8 STATISTICS_DATA_TYPE_STDDEV = 4
uint8 STATISTICS_DATA_TYPE_SAMPLE_COUNT = 5
```

## std_msgs

### Bool `msg` {/* #common-interfaces-std-msgs-msg-bool-msg */}

`common_interfaces/std_msgs/msg/Bool.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

bool data
```

### Byte `msg` {/* #common-interfaces-std-msgs-msg-byte-msg */}

`common_interfaces/std_msgs/msg/Byte.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

byte data
```

### ByteMultiArray `msg` {/* #common-interfaces-std-msgs-msg-bytemultiarray-msg */}

`common_interfaces/std_msgs/msg/ByteMultiArray.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
byte[]            data          # array of data
```

### Char `msg` {/* #common-interfaces-std-msgs-msg-char-msg */}

`common_interfaces/std_msgs/msg/Char.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

char data
```

### ColorRGBA `msg` {/* #common-interfaces-std-msgs-msg-colorrgba-msg */}

`common_interfaces/std_msgs/msg/ColorRGBA.msg`

```rosidl
float32 r
float32 g
float32 b
float32 a
```

### Empty `msg` {/* #common-interfaces-std-msgs-msg-empty-msg */}

`common_interfaces/std_msgs/msg/Empty.msg`

```rosidl

```

### Float32 `msg` {/* #common-interfaces-std-msgs-msg-float32-msg */}

`common_interfaces/std_msgs/msg/Float32.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

float32 data
```

### Float32MultiArray `msg` {/* #common-interfaces-std-msgs-msg-float32multiarray-msg */}

`common_interfaces/std_msgs/msg/Float32MultiArray.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data
```

### Float64 `msg` {/* #common-interfaces-std-msgs-msg-float64-msg */}

`common_interfaces/std_msgs/msg/Float64.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

float64 data
```

### Float64MultiArray `msg` {/* #common-interfaces-std-msgs-msg-float64multiarray-msg */}

`common_interfaces/std_msgs/msg/Float64MultiArray.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data
```

### Header `msg` {/* #common-interfaces-std-msgs-msg-header-msg */}

`common_interfaces/std_msgs/msg/Header.msg`

```rosidl
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
```

### Int16 `msg` {/* #common-interfaces-std-msgs-msg-int16-msg */}

`common_interfaces/std_msgs/msg/Int16.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int16 data
```

### Int16MultiArray `msg` {/* #common-interfaces-std-msgs-msg-int16multiarray-msg */}

`common_interfaces/std_msgs/msg/Int16MultiArray.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int16[]           data          # array of data
```

### Int32 `msg` {/* #common-interfaces-std-msgs-msg-int32-msg */}

`common_interfaces/std_msgs/msg/Int32.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int32 data
```

### Int32MultiArray `msg` {/* #common-interfaces-std-msgs-msg-int32multiarray-msg */}

`common_interfaces/std_msgs/msg/Int32MultiArray.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int32[]           data          # array of data
```

### Int64 `msg` {/* #common-interfaces-std-msgs-msg-int64-msg */}

`common_interfaces/std_msgs/msg/Int64.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int64 data
```

### Int64MultiArray `msg` {/* #common-interfaces-std-msgs-msg-int64multiarray-msg */}

`common_interfaces/std_msgs/msg/Int64MultiArray.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int64[]           data          # array of data
```

### Int8 `msg` {/* #common-interfaces-std-msgs-msg-int8-msg */}

`common_interfaces/std_msgs/msg/Int8.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int8 data
```

### Int8MultiArray `msg` {/* #common-interfaces-std-msgs-msg-int8multiarray-msg */}

`common_interfaces/std_msgs/msg/Int8MultiArray.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int8[]            data          # array of data
```

### MultiArrayDimension `msg` {/* #common-interfaces-std-msgs-msg-multiarraydimension-msg */}

`common_interfaces/std_msgs/msg/MultiArrayDimension.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
```

### MultiArrayLayout `msg` {/* #common-interfaces-std-msgs-msg-multiarraylayout-msg */}

`common_interfaces/std_msgs/msg/MultiArrayLayout.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
```

### String `msg` {/* #common-interfaces-std-msgs-msg-string-msg */}

`common_interfaces/std_msgs/msg/String.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string data
```

### UInt16 `msg` {/* #common-interfaces-std-msgs-msg-uint16-msg */}

`common_interfaces/std_msgs/msg/UInt16.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint16 data
```

### UInt16MultiArray `msg` {/* #common-interfaces-std-msgs-msg-uint16multiarray-msg */}

`common_interfaces/std_msgs/msg/UInt16MultiArray.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint16[]            data        # array of data
```

### UInt32 `msg` {/* #common-interfaces-std-msgs-msg-uint32-msg */}

`common_interfaces/std_msgs/msg/UInt32.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint32 data
```

### UInt32MultiArray `msg` {/* #common-interfaces-std-msgs-msg-uint32multiarray-msg */}

`common_interfaces/std_msgs/msg/UInt32MultiArray.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint32[]          data          # array of data
```

### UInt64 `msg` {/* #common-interfaces-std-msgs-msg-uint64-msg */}

`common_interfaces/std_msgs/msg/UInt64.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint64 data
```

### UInt64MultiArray `msg` {/* #common-interfaces-std-msgs-msg-uint64multiarray-msg */}

`common_interfaces/std_msgs/msg/UInt64MultiArray.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint64[]          data          # array of data
```

### UInt8 `msg` {/* #common-interfaces-std-msgs-msg-uint8-msg */}

`common_interfaces/std_msgs/msg/UInt8.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint8 data
```

### UInt8MultiArray `msg` {/* #common-interfaces-std-msgs-msg-uint8multiarray-msg */}

`common_interfaces/std_msgs/msg/UInt8MultiArray.msg`

```rosidl
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint8[]           data          # array of data
```

## std_srvs

### Empty `srv` {/* #common-interfaces-std-srvs-srv-empty-srv */}

`common_interfaces/std_srvs/srv/Empty.srv`

```rosidl
---
```

### SetBool `srv` {/* #common-interfaces-std-srvs-srv-setbool-srv */}

`common_interfaces/std_srvs/srv/SetBool.srv`

```rosidl
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

### Trigger `srv` {/* #common-interfaces-std-srvs-srv-trigger-srv */}

`common_interfaces/std_srvs/srv/Trigger.srv`

```rosidl
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

## stereo_msgs

### DisparityImage `msg` {/* #common-interfaces-stereo-msgs-msg-disparityimage-msg */}

`common_interfaces/stereo_msgs/msg/DisparityImage.msg`

```rosidl
# Separate header for compatibility with current TimeSynchronizer.
# Likely to be removed in a later release, use image.header instead.
std_msgs/Header header

# Floating point disparity image. The disparities are pre-adjusted for any
# x-offset between the principal points of the two cameras (in the case
# that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)
sensor_msgs/Image image

# Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.
float32 f # Focal length, pixels
float32 t # Baseline, world units

# Subwindow of (potentially) valid disparity values.
sensor_msgs/RegionOfInterest valid_window

# The range of disparities searched.
# In the disparity image, any disparity less than min_disparity is invalid.
# The disparity search range defines the horopter, or 3D volume that the
# stereo algorithm can "see". Points with Z outside of:
#     Z_min = fT / max_disparity
#     Z_max = fT / min_disparity
# could not be found.
float32 min_disparity
float32 max_disparity

# Smallest allowed disparity increment. The smallest achievable depth range
# resolution is delta_Z = (Z^2/fT)*delta_d.
float32 delta_d
```

## test_msgs

### Builtins `msg` {/* #rcl-interfaces-test-msgs-msg-builtins-msg */}

`rcl_interfaces/test_msgs/msg/Builtins.msg`

```rosidl
builtin_interfaces/Duration duration_value
builtin_interfaces/Time time_value
```

## trajectory_msgs

### JointTrajectory `msg` {/* #common-interfaces-trajectory-msgs-msg-jointtrajectory-msg */}

`common_interfaces/trajectory_msgs/msg/JointTrajectory.msg`

```rosidl
# The header is used to specify the coordinate frame and the reference time for
# the trajectory durations
std_msgs/Header header

# The names of the active joints in each trajectory point. These names are
# ordered and must correspond to the values in each trajectory point.
string[] joint_names

# Array of trajectory points, which describe the positions, velocities,
# accelerations and/or efforts of the joints at each time point.
JointTrajectoryPoint[] points
```

### JointTrajectoryPoint `msg` {/* #common-interfaces-trajectory-msgs-msg-jointtrajectorypoint-msg */}

`common_interfaces/trajectory_msgs/msg/JointTrajectoryPoint.msg`

```rosidl
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg.

# Single DOF joint positions for each joint relative to their "0" position.
# The units depend on the specific joint type: radians for revolute or
# continuous joints, and meters for prismatic joints.
float64[] positions

# The rate of change in position of each joint. Units are joint type dependent.
# Radians/second for revolute or continuous joints, and meters/second for
# prismatic joints.
float64[] velocities

# Rate of change in velocity of each joint. Units are joint type dependent.
# Radians/second^2 for revolute or continuous joints, and meters/second^2 for
# prismatic joints.
float64[] accelerations

# The torque or the force to be applied at each joint. For revolute/continuous
# joints effort denotes a torque in newton-meters. For prismatic joints, effort
# denotes a force in newtons.
float64[] effort

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start
```

### MultiDOFJointTrajectory `msg` {/* #common-interfaces-trajectory-msgs-msg-multidofjointtrajectory-msg */}

`common_interfaces/trajectory_msgs/msg/MultiDOFJointTrajectory.msg`

```rosidl
# The header is used to specify the coordinate frame and the reference time for the trajectory durations
std_msgs/Header header

# A representation of a multi-dof joint trajectory (each point is a transformation)
# Each point along the trajectory will include an array of positions/velocities/accelerations
# that has the same length as the array of joint names, and has the same order of joints as
# the joint names array.

string[] joint_names
MultiDOFJointTrajectoryPoint[] points
```

### MultiDOFJointTrajectoryPoint `msg` {/* #common-interfaces-trajectory-msgs-msg-multidofjointtrajectorypoint-msg */}

`common_interfaces/trajectory_msgs/msg/MultiDOFJointTrajectoryPoint.msg`

```rosidl
# Each multi-dof joint can specify a transform (up to 6 DOF).
geometry_msgs/Transform[] transforms

# There can be a velocity specified for the origin of the joint.
geometry_msgs/Twist[] velocities

# There can be an acceleration specified for the origin of the joint.
geometry_msgs/Twist[] accelerations

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start
```

## tts

### Synthesize `srv` {/* #tts-srv-synthesize-srv */}

`tts/srv/Synthesize.srv`

```rosidl
# TTS one-shot synthesis
# Contract: robonix/service/speech/tts
string text           # text to synthesize
string language       # BCP-47, e.g. "zh-CN", "" = default
string voice          # voice ID, "" = default
float32 speed         # 1.0 = normal; < 1.0 = slower; > 1.0 = faster
---
uint8[] audio_data    # synthesized audio (format specified in encoding)
string encoding       # e.g. "pcm_s16le", "mp3"
uint32 sample_rate_hz
string error          # non-empty on failure
```

### SynthesizeAudioChunk `msg` {/* #tts-msg-synthesizeaudiochunk-msg */}

`tts/msg/SynthesizeAudioChunk.msg`

```rosidl
# Audio chunk streamed back by TTS during streaming synthesis.
audio/msg/AudioChunk chunk
string encoding           # e.g. "pcm_s16le", "mp3"
uint32 sample_rate_hz
bool is_final             # true when this is the last chunk
```

### SynthesizeStream `srv` {/* #tts-srv-synthesizestream-srv */}

`tts/srv/SynthesizeStream.srv`

```rosidl
# TTS streaming synthesis (server stream).
# Client sends text request; server streams back SynthesizeAudioChunk.
# Stream element type is referenced in the contract TOML.
string text
string language
string voice
float32 speed
audio/AudioConfig audio_config
---
tts/SynthesizeAudioChunk chunk
```

## unique_identifier_msgs

### UUID `msg` {/* #unique-identifier-msgs-msg-uuid-msg */}

`unique_identifier_msgs/msg/UUID.msg`

```rosidl
# A universally unique identifier (UUID).
#
#  http://en.wikipedia.org/wiki/Universally_unique_identifier
#  http://tools.ietf.org/html/rfc4122.html

uint8[16] uuid
```

## visualization_msgs

### GetInteractiveMarkers `srv` {/* #common-interfaces-visualization-msgs-srv-getinteractivemarkers-srv */}

`common_interfaces/visualization_msgs/srv/GetInteractiveMarkers.srv`

```rosidl
---
# Sequence number.
# Set to the sequence number of the latest update message
# at the time the server received the request.
# Clients use this to detect if any updates were missed.
uint64 sequence_number

# All interactive markers provided by the server.
InteractiveMarker[] markers
```

### ImageMarker `msg` {/* #common-interfaces-visualization-msgs-msg-imagemarker-msg */}

`common_interfaces/visualization_msgs/msg/ImageMarker.msg`

```rosidl
int32 CIRCLE=0
int32 LINE_STRIP=1
int32 LINE_LIST=2
int32 POLYGON=3
int32 POINTS=4

int32 ADD=0
int32 REMOVE=1

std_msgs/Header header
# Namespace which is used with the id to form a unique id.
string ns
# Unique id within the namespace.
int32 id
# One of the above types, e.g. CIRCLE, LINE_STRIP, etc.
int32 type
# Either ADD or REMOVE.
int32 action
# Two-dimensional coordinate position, in pixel-coordinates.
geometry_msgs/Point position
# The scale of the object, e.g. the diameter for a CIRCLE.
float32 scale
# The outline color of the marker.
std_msgs/ColorRGBA outline_color
# Whether or not to fill in the shape with color.
uint8 filled
# Fill color; in the range: [0.0-1.0]
std_msgs/ColorRGBA fill_color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime

# Coordinates in 2D in pixel coords. Used for LINE_STRIP, LINE_LIST, POINTS, etc.
geometry_msgs/Point[] points
# The color for each line, point, etc. in the points field.
std_msgs/ColorRGBA[] outline_colors
```

### InteractiveMarker `msg` {/* #common-interfaces-visualization-msgs-msg-interactivemarker-msg */}

`common_interfaces/visualization_msgs/msg/InteractiveMarker.msg`

```rosidl
# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name

# Short description (< 40 characters).
string description

# Scale to be used for default controls (default=1).
float32 scale

# All menu and submenu entries associated with this marker.
MenuEntry[] menu_entries

# List of controls displayed for this marker.
InteractiveMarkerControl[] controls
```

### InteractiveMarkerControl `msg` {/* #common-interfaces-visualization-msgs-msg-interactivemarkercontrol-msg */}

`common_interfaces/visualization_msgs/msg/InteractiveMarkerControl.msg`

```rosidl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
```

### InteractiveMarkerFeedback `msg` {/* #common-interfaces-visualization-msgs-msg-interactivemarkerfeedback-msg */}

`common_interfaces/visualization_msgs/msg/InteractiveMarkerFeedback.msg`

```rosidl
# Time/frame info.
std_msgs/Header header

# Identifying string. Must be unique in the topic namespace.
string client_id

# Feedback message sent back from the GUI, e.g.
# when the status of an interactive marker was modified by the user.

# Specifies which interactive marker and control this message refers to
string marker_name
string control_name

# Type of the event
# KEEP_ALIVE: sent while dragging to keep up control of the marker
# MENU_SELECT: a menu entry has been selected
# BUTTON_CLICK: a button control has been clicked
# POSE_UPDATE: the pose has been changed using one of the controls
uint8 KEEP_ALIVE = 0
uint8 POSE_UPDATE = 1
uint8 MENU_SELECT = 2
uint8 BUTTON_CLICK = 3

uint8 MOUSE_DOWN = 4
uint8 MOUSE_UP = 5

uint8 event_type

# Current pose of the marker
# Note: Has to be valid for all feedback types.
geometry_msgs/Pose pose

# Contains the ID of the selected menu entry
# Only valid for MENU_SELECT events.
uint32 menu_entry_id

# If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point
# may contain the 3 dimensional position of the event on the
# control.  If it does, mouse_point_valid will be true.  mouse_point
# will be relative to the frame listed in the header.
geometry_msgs/Point mouse_point
bool mouse_point_valid
```

### InteractiveMarkerInit `msg` {/* #common-interfaces-visualization-msgs-msg-interactivemarkerinit-msg */}

`common_interfaces/visualization_msgs/msg/InteractiveMarkerInit.msg`

```rosidl
# Identifying string. Must be unique in the topic namespace
# that this server works on.
string server_id

# Sequence number.
# The client will use this to detect if it has missed a subsequent
# update.  Every update message will have the same sequence number as
# an init message.  Clients will likely want to unsubscribe from the
# init topic after a successful initialization to avoid receiving
# duplicate data.
uint64 seq_num

# All markers.
InteractiveMarker[] markers
```

### InteractiveMarkerPose `msg` {/* #common-interfaces-visualization-msgs-msg-interactivemarkerpose-msg */}

`common_interfaces/visualization_msgs/msg/InteractiveMarkerPose.msg`

```rosidl

# Time/frame info.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name
```

### InteractiveMarkerUpdate `msg` {/* #common-interfaces-visualization-msgs-msg-interactivemarkerupdate-msg */}

`common_interfaces/visualization_msgs/msg/InteractiveMarkerUpdate.msg`

```rosidl

# Identifying string. Must be unique in the topic namespace
# that this server works on.
string server_id

# Sequence number.
# The client will use this to detect if it has missed an update.
uint64 seq_num

# Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.
# UPDATE: Incremental update to previous state.
#         The sequence number must be 1 higher than for
#         the previous update.
# KEEP_ALIVE: Indicates the that the server is still living.
#             The sequence number does not increase.
#             No payload data should be filled out (markers, poses, or erases).
uint8 KEEP_ALIVE = 0
uint8 UPDATE = 1

uint8 type

# Note: No guarantees on the order of processing.
#       Contents must be kept consistent by sender.

# Markers to be added or updated
InteractiveMarker[] markers

# Poses of markers that should be moved
InteractiveMarkerPose[] poses

# Names of markers to be erased
string[] erases
```

### Marker `msg` {/* #common-interfaces-visualization-msgs-msg-marker-msg */}

`common_interfaces/visualization_msgs/msg/Marker.msg`

```rosidl
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
```

### MarkerArray `msg` {/* #common-interfaces-visualization-msgs-msg-markerarray-msg */}

`common_interfaces/visualization_msgs/msg/MarkerArray.msg`

```rosidl
Marker[] markers
```

### MenuEntry `msg` {/* #common-interfaces-visualization-msgs-msg-menuentry-msg */}

`common_interfaces/visualization_msgs/msg/MenuEntry.msg`

```rosidl
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
```

### MeshFile `msg` {/* #common-interfaces-visualization-msgs-msg-meshfile-msg */}

`common_interfaces/visualization_msgs/msg/MeshFile.msg`

```rosidl
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
```

### UVCoordinate `msg` {/* #common-interfaces-visualization-msgs-msg-uvcoordinate-msg */}

`common_interfaces/visualization_msgs/msg/UVCoordinate.msg`

```rosidl
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
```

## vitals

### BodyComponent `msg` {/* #vitals-msg-bodycomponent-msg */}

`vitals/msg/BodyComponent.msg`

```rosidl
string name                  # "joint_1" / "motor_drive" / "display"
string kind                  # "joint" | "motor" | "actuator" | "sensor" | "other"
float32 temperature          # °C, -1 if unavailable
uint32 error_code            # 0 = OK, model-defined bitmask otherwise
bool enabled                 # torque / driver enable status
string id                    # stable component path, e.g. "body/arm_right/joint_1"
string parent_id             # parent component path, empty for root
string model                 # component model, e.g. "piper_motor" / "jetson_agx_orin"
```

### BodyHealth `msg` {/* #vitals-msg-bodyhealth-msg */}

`vitals/msg/BodyHealth.msg`

```rosidl
uint32 NORMAL=0
uint32 FAULT=1
uint32 ESTOP=2

string body_type             # root child id/name, e.g. "computer_jetson" | "arm_right"
string model                 # "jetson_agx_orin" | "piper" | "mock_bms" | ...
uint32 state                 # NORMAL / FAULT / ESTOP
string message               # human-readable status summary (can be empty)
BodyComponent[] components   # component-tree entries below this root child
```

### ComponentHealth `msg` {/* #vitals-msg-componenthealth-msg */}

`vitals/msg/ComponentHealth.msg`

```rosidl
uint8 OK=0
uint8 WARN=1
uint8 ERROR=2
string name                  # component name, e.g. "cpu", "gpu", "battery"
uint8 health                 # OK / WARN / ERROR
string detail                # human-readable detail, e.g. "CPU temp 85C exceeds 80C threshold"
float32 value                # current value (temp:°C, voltage:V, current:A), -1 if absent
float32 threshold            # threshold that was crossed, -1 if absent
```

### GetVitals `srv` {/* #vitals-srv-getvitals-srv */}

`vitals/srv/GetVitals.srv`

```rosidl
---
VitalsSnapshot snapshot
```

### PowerState `msg` {/* #vitals-msg-powerstate-msg */}

`vitals/msg/PowerState.msg`

```rosidl
float32 battery_percent      # battery percentage 0-100, -1 if no battery
float32 voltage              # battery/system voltage (V), -1 if unknown
bool charging                # charging status
int64 remaining_s            # estimated remaining seconds, -1 if unknown
```

### StreamVitals `srv` {/* #vitals-srv-streamvitals-srv */}

`vitals/srv/StreamVitals.srv`

```rosidl
---
VitalsSnapshot snapshot      # server_stream: one frame per push
```

### VitalsSnapshot `msg` {/* #vitals-msg-vitalssnapshot-msg */}

`vitals/msg/VitalsSnapshot.msg`

```rosidl
uint64 ts_ns                 # chronos canonical timestamp (CLOCK_MONOTONIC ns)
PowerState power
ComponentHealth[] components
BodyHealth[] bodies          # zero or more body health entries
```

## voiceprint

### DeleteEnrolled `srv` {/* #voiceprint-srv-deleteenrolled-srv */}

`voiceprint/srv/DeleteEnrolled.srv`

```rosidl
# Voiceprint deletion (admin RPC).
# Caller sends a user_id; service forgets the embedding + persists
# the change to enrolled.json. Subsequent Identify(...) calls will no
# longer match this speaker. Used by `rbnx chat`'s Settings → Users
# modal when the operator presses `d` on the cursor row.
# Contract: robonix/service/voiceprint/delete
string  user_id          # which user to forget; no-op + success=true if absent
---
bool    success          # true if the row was removed OR was already absent
string  error            # non-empty on failure (I/O, etc.)
```

### Enroll `srv` {/* #voiceprint-srv-enroll-srv */}

`voiceprint/srv/Enroll.srv`

```rosidl
# Voiceprint enrollment (admin RPC).
# Caller sends raw audio + the user_id/user_name to bind. Service
# extracts the speaker embedding and persists it. Subsequent calls
# to Identify(...) with the same speaker should now match this user_id.
# Contract: robonix/service/voiceprint/enroll
string  user_id          # stable id (caller-chosen, must be unique)
string  user_name        # human-friendly label (free-form)
uint8[] audio_data       # raw PCM bytes, length >= 1 second is recommended
string  encoding         # "pcm_s16le" | "wav"; empty -> pcm_s16le
uint32  sample_rate_hz   # sample rate of audio_data; 0 -> 16000
---
bool    success
string  error            # non-empty on failure
```

### Identify `srv` {/* #voiceprint-srv-identify-srv */}

`voiceprint/srv/Identify.srv`

```rosidl
# Voiceprint identification (one-shot)
# Contract: robonix/service/voiceprint/identify
uint8[] audio_data
string  encoding         # e.g. "pcm_s16le"
uint32  sample_rate_hz   # e.g. 16000
---
string  user_id          # enrolled id, or "unknown"
string  user_name        # human-friendly label, "" if unknown
float32 confidence       # 0.0 – 1.0
bool    is_known         # true when matched against an enrolled voiceprint
string  error            # non-empty on failure
```

### ListEnrolled `srv` {/* #voiceprint-srv-listenrolled-srv */}

`voiceprint/srv/ListEnrolled.srv`

```rosidl
# List every currently enrolled voiceprint user (admin RPC).
# Returns a JSON-encoded array so the catalog shape can evolve without
# changing the IDL — same pattern as robonix/service/speech/list_speakers.
# Contract: robonix/service/voiceprint/list
---
string  users_json       # JSON array: [{"user_id":"...","user_name":"..."}, ...]
uint32  count            # convenience: number of users in users_json
string  error            # non-empty on failure
```
