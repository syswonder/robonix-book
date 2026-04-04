# 端到端验收

完成代码和 manifest 编写后，按以下步骤验证"Agent 能调用你的接口"。每步给出具体命令和预期输出。

## 1. 校验 Manifest

```bash
cd rust && make install
rbnx validate path/to/your_package
```

预期输出包含 `manifest OK` 或类似确认。若 `manifestVersion`、`package.id`、`nodes` 结构有问题，会输出具体错误信息。

## 2. 构建

```bash
rbnx build -p path/to/your_package
```

执行 `build.script` 中定义的脚本。Python 包通常设置 PYTHONPATH，Docker compose 包则构建镜像。确认退出码为 0。

## 3. 启动控制平面

```bash
robonix-atlas
```

日志中出现 `starting robonix runtime meta API (gRPC)` 表示服务就绪。

## 4. 启动节点

```bash
rbnx start \
  -p path/to/your_package \
  -n your.node.id \
  --endpoint 127.0.0.1:50051
```

观察节点日志，确认：

- `RegisterNode` 成功（服务端日志应有 `meta-runtime: registered node 'your.node.id'`）
- `DeclareInterface` 成功（服务端日志应有 `meta-runtime: declared interface 'xxx' on node 'your.node.id' → endpoint 'localhost:xxxxx'`）
- 数据面 server 已启动（如 `MCP HTTP on port xxxx` 或 `gRPC listening on port xxxx`）

## 5. 确认注册

用 `rbnx` CLI 检查节点是否出现在控制平面中：

```bash
rbnx nodes
```

应看到你的 node_id、namespace 和接口列表。若提供了 MCP 工具：

```bash
rbnx tools
```

应看到你的工具名称出现在列表中。

## 6. 启动 Agent 并测试

```bash
robonix-pilot
```

Agent 启动时会打印它发现的 VLM 和 MCP 工具。在终端输入一条会触发你的工具的指令，观察 Agent 是否成功调用。

对 gRPC 接口（非 MCP 工具），可用 `grpcurl` 或编写小脚本，通过 `NegotiateChannel` 获取端点后直接调用：

```bash
# 查看运行时快照，确认接口和通道
rbnx inspect
```

## 常见失败与排查

### DeclareInterface 返回 INVALID_ARGUMENT

对 gRPC/ROS 2 传输，**`contract_id`**（建议在 `DeclareInterface` 中显式填写，与 `rust/contracts` 中 `[contract] id` 一致）必须在系统接口目录中。若需定义新能力：先在 `rust/crates/robonix-interfaces/lib/` 添加 IDL，在 **`rust/contracts/`** 添加契约 TOML，运行带 **`--contracts`** 的 `robonix-codegen`，并将该契约 ID 加入 `robonix-atlas` 的 **`ROBO_SYSTEM_INTERFACE_CATALOG`**。

MCP 传输不受此限制。

### Agent 发现不到 MCP 工具

Agent 通过 `QueryNodes(transport="mcp")` 发现 MCP provider，再从 `metadata_json.endpoint` 连接 MCP server。确认以下各项：

- `DeclareInterface` 的 `supported_transports` 包含 `"mcp"`
- `metadata_json` 是合法 JSON 且包含工具定义
- MCP HTTP server 确实在 `listen_port` 上监听
- 网络可达（若 provider 在 Docker 中，endpoint 的 host 部分须为宿主机可访问的地址）

### gRPC 端口冲突

若多个节点均由控制平面自动分配端口（`listen_port=0`），端口从 50100 开始递增，通常不会冲突。若指定了固定端口且与其他服务重叠，会导致绑定失败。建议 provider 自行绑定到空闲端口，再通过 `listen_port` 告知控制平面。

### 容器内 provider 的 endpoint 不可达

Docker 容器内的 provider 注册时，`ROBONIX_DATA_PLANE_HOST` 默认为 `localhost`，但容器的 localhost 与宿主机隔离。需将 `ROBONIX_DATA_PLANE_HOST` 设置为容器对外暴露的地址（如 Docker 网络的网关 IP 或 `host.docker.internal`）。

### 节点注册后消失

若 provider 进程崩溃或被终止，节点会保留在控制平面注册表中，但心跳停止。当前实现不会自动清理过期节点。重启 provider 时，`RegisterNode` 会以 upsert 语义更新已有记录。
