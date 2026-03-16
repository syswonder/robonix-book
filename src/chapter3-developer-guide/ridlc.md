# ridlc 开发手册

本文档描述 ridlc 的代码生成规范与通信实现，供实现者与开发者参考。RIDL 语义定义见 [RFC001](../rfc/RFC001-RIDL.md)。

RIDL 语法速览（关键词、文件规则、命名等）见 RFC001 §3。

**类型引用简写**：`import pkg/msg/Name` 后，字段类型可使用短名 `Name`，也可使用全名 `pkg/msg/Name`；若多个 import 的末段相同则需用全名避免歧义。详见 RFC001 §5.1。

**LLM/Agent 友好注释**：`@desc("...")` 用于接口或字段，描述用途、含义与格式。会写入 DESCRIPTOR.annotations，供 AI 理解接口、正确构造请求。详见 RFC001 §5.2。

**业务逻辑补全**：生成代码只提供 channel 注册、ROS 绑定和工厂函数；用户需在指定位置补全回调或业务逻辑。§5 详细说明每种原语下**在哪里补全**、**补全什么**、**如何调用**，是编写 server/client/stream 代码的核心参考。

---

## 0. 文件与接口约定

- **一文件一接口**：每个 `.ridl` 文件只定义一个接口。
- **接口名 = 文件名**：接口名必须与文件名（不含扩展名）一致，如 `depth.ridl` 定义 `stream depth`，`move.ridl` 定义 `command move`。类似 Java：一个文件一个 public 类，类名与文件名一致。
- **接口语法**：`command/stream/query 接口名 [注解...] { 字段... }`；字段为 `方向 字段名 类型 [注解...];`。注解可多个，如 `@desc`、`@frame`。完整语法见 [RFC001 §3.1](../rfc/RFC001-RIDL.md)。

---

## 1. 通信实现映射

当前 ridlc 采用 ROS 2 作为后端。各原语与 ROS 2 / gRPC 的映射如下：

| 原语 | ROS 2 | gRPC（可选实现） |
|------|-------|------------------|
| stream | topic | server/client streaming |
| command | action | 双向流或 server streaming |
| query | service | unary RPC |

---

## 2. 代码生成规范

### 2.1 命名规律

| 原语 | 生成文件 | Python 工厂 | Rust 模块 |
|------|----------|-------------|-----------|
| query | `{name}_query.py` | `create_{name}_client`, `create_{name}_server` | `{name}_query` |
| stream | `{name}_stream.py` | `create_{name}_publisher`, `create_{name}_subscriber` | `{name}_stream` |
| command | `{name}_command.py` | `create_{name}_client`, `create_{name}_server` | `{name}_command` |

### 2.2 命名空间映射

- `robonix/a/b/c` → Python 包 `robonix.a.b.c`
- `robonix/a/b/c` → Rust 模块 `robonix_a_b_c`
- ROS 2 类型名：namespace 去掉 `robonix/` 后按层级 PascalCase 拼接 + 接口名，如 `robonix/system/debug` + `ping` → `SystemDebugPing`

---

## 3. 运行时与 channel

生成代码通过 gRPC meta API 完成 Register*/Resolve* 调用。channel 由 robonix-server 分配，生成代码据此创建 ROS service/topic/action 并绑定。

---

## 4. 注解

RIDL 支持的注解（如 `@desc`、`@frame`、`@requires_interface`、`@interruptible`）见 RFC001 §5.2。ridlc 解析后传入 codegen，具体语义由生成代码与运行时实现。

---

## 5. 用户逻辑补全（Python）

生成代码提供工厂函数和基类，用户需补全「回调」或「业务逻辑」。下表说明每种通信模式下，**在哪里补全**、**补全什么**。

### 5.1 Query（请求-响应）

| 角色 | 补全位置 | 补全方式 | 说明 |
|------|----------|----------|------|
| **Server** | `server.start(handler)` | 传入 `handler` 函数 | 每次收到请求时调用 |
| **Client** | 无回调 | 直接调用 `client.call(request)` | 同步阻塞，返回 `Response \| None` |

**Server 补全示例**：

```python
server = create_semantic_query_server(runtime_client, node_id=node_id)

def handler(request, response):
    # request: 服务 Request 类型；response: 服务 Response 类型；需返回 response
    response.objects = [...]  # 填充 response
    return response

server.start(handler)   # 传入 handler
rclpy.spin(server)
```

**Client 补全示例**：

```python
client = create_semantic_query_client(runtime_client, requester_id=..., target=...)

request = client._srv_type.Request()
request.filter.data = "room"

response = client.call(request)
if response is None:
    raise RuntimeError("call failed or timed out")
for obj in response.objects:
    print(obj.id, obj.label)
```

---

### 5.2 Stream（发布/订阅）

| 角色 | 补全位置 | 补全方式 | 说明 |
|------|----------|----------|------|
| **Publisher** | 定时器或业务逻辑 | 调用 `publisher.publish(msg)` | 构造消息并发布 |
| **Subscriber** | `subscriber.start(callback)` | 传入 `callback(msg)` 函数 | 每次收到消息时调用 |

**Publisher 补全示例**：

```python
publisher = create_pose_cov_publisher(runtime_client, node_id=node_id)

def publish_once():
    msg = publisher._msg_type()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 1.25
    publisher.publish(msg)   # 调用 publish

timer = publisher.create_timer(0.5, publish_once)
rclpy.spin(publisher)
```

**Subscriber 补全示例**：

```python
subscriber = create_pose_cov_subscriber(runtime_client, requester_id=..., target=...)

def on_msg(msg):
    # msg: 消息类型（如 PoseWithCovarianceStamped）
    print(msg.pose.pose.position)

subscriber.start(on_msg)   # 传入 callback
rclpy.spin(subscriber)
```

---

### 5.3 Command（动作/技能）

Command 有 input/output/result 三个字段，对应 ROS2 Action 的 Goal/Feedback/Result。

| 角色 | 补全位置 | 补全方式 | 说明 |
|------|----------|----------|------|
| **Server** | `server.execute` | 赋值 `server.execute = your_execute` 或子类重写 | 处理 Goal，可选发 Feedback，返回 Result |
| **Client** | 无回调（简单用法） | 调用 `client.send(request)` → `goal_handle` → `get_result_async()` | 需 `rclpy.spin_until_future_complete` |
| **Client** | `send_goal_async(..., feedback_callback=...)` | 传入 `feedback_callback` 接收 output | 接收执行过程中的 feedback |

**Server 补全示例**：

```python
server = create_greet_server(runtime_client, node_id=node_id)

def execute(request, goal_handle):
    # request: Action.Goal；goal_handle: ServerGoalHandle | None
    # output (feedback): 可选，通过 goal_handle.publish_feedback(fb) 发送
    if goal_handle:
        fb = server._action_type.Feedback()
        fb.feedback.progress = "processing..."
        goal_handle.publish_feedback(fb)
    # result: 必须返回
    result = server._action_type.Result()
    result.response.message = f"Hello, {request.request.name}!"
    return result

server.execute = execute   # 赋值 execute
server.start()
rclpy.spin(server)
```

**Client 补全示例（简单用法，不接收 feedback）**：

```python
client = create_greet_client(runtime_client, requester_id=..., target=...)

request = client._action_type.Goal()
request.request = GreetRequest()
request.request.name = "world"

goal_handle = client.send(request)
if goal_handle is None or not goal_handle.accepted:
    raise RuntimeError("goal not accepted")

result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(client, result_future, timeout_sec=10.0)
wrapped = result_future.result()
if wrapped is None:
    raise RuntimeError("timeout waiting for result")
print(wrapped.result.response.message, wrapped.result.response.success)
```

**Client 补全示例（接收 feedback）**：

```python
client = create_greet_client(runtime_client, requester_id=..., target=...)

request = client._action_type.Goal()
request.request = GreetRequest()
request.request.name = "world"

def on_feedback(fb_msg):
    # fb_msg 为 FeedbackMessage，访问自定义 output 字段：fb_msg.feedback.feedback.<字段名>
    print("feedback:", fb_msg.feedback.feedback.progress)

goal_future = client._client.send_goal_async(request, feedback_callback=on_feedback)
rclpy.spin_until_future_complete(client, goal_future, timeout_sec=10.0)
goal_handle = goal_future.result()
if goal_handle is None or not goal_handle.accepted:
    raise RuntimeError("goal not accepted")

result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(client, result_future, timeout_sec=10.0)
wrapped = result_future.result()
# 处理 wrapped.result
```

**Client 接收 feedback 注意**：`feedback_callback` 收到的是 `FeedbackMessage`（含 `goal_id` 和 `feedback`），不是裸 `Feedback`。访问自定义 output 字段需：`fb_msg.feedback.feedback.<字段名>`（第一个 `feedback` 为 FeedbackMessage 的字段，第二个为 action Feedback 中 output 字段名，如 `progress`）。

---

### 5.4 快速对照表

| 原语 | Server/Provider 补全 | Client/Consumer 补全 |
|------|---------------------|----------------------|
| **Query** | `server.start(handler)`，`handler(request, response) -> response` | 构造 `Request` → `client.call(request)` → 处理 `Response \| None` |
| **Stream** | `publisher.publish(msg)`（在定时器或业务逻辑中调用） | `subscriber.start(callback)`，`callback(msg)` 处理消息 |
| **Command** | `server.execute = fn`，`fn(request, goal_handle) -> result`；可选 `goal_handle.publish_feedback(fb)` | 构造 `Goal` → `client.send(request)` 或 `send_goal_async(..., feedback_callback=on_fb)` → `goal_handle.get_result_async()` → `rclpy.spin_until_future_complete` → `wrapped.result`；feedback 回调内用 `fb_msg.feedback.feedback.<字段名>` 访问 output |
