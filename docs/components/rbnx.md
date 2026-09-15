---
title: rbnx 命令行工具
---

# rbnx 命令行工具

`rbnx` 是 Robonix 的部署命令行工具，包括构建软件包、启动 Robonix、查看 Robonix 运行时状态、与 Robonix 对话交互等功能。

相关文档：
1. 部署与启动的两层清单和阶段划分见[部署与启动](../architecture/deployment-and-startup.md)
2. `robonix_manifest.yaml` 字段的完整规格见[软件包与部署清单规范](../integration-guide/packaging-spec.md)
3. 构建与代码生成见[构建与代码生成](../integration-guide/build-and-codegen.md)

## 安装与登记 Robonix 源码位置

```bash
cd robonix && make install     # rbnx、atlas、pilot、executor、liaison、codegen → ~/.cargo/bin
rbnx setup                     # 把当前 clone 目录登记为 Robonix 源码位置
```

`rbnx setup` 把当前克隆的绝对路径写进 `~/.robonix/config.yaml`。此后在磁盘任何位置调用 `rbnx path` 都能解析到源码树内的路径。

多数子命令需要它：代码生成要找 `capabilities/lib` 下的 IDL 树，构建要找 `system/atlas/proto`，Python 软件包要找 `pylib/robonix-api`。

## 命令速查

### 构建与代码生成

| 命令 | 作用 |
|---|---|
| `rbnx validate` | 只校验软件包清单，不构建 |
| `rbnx codegen [--mcp] [--ros2]` | 为软件包生成 proto、gRPC 桩和 MCP 类型。`--ros2` 额外生成 `ros2_idl/` |
| `rbnx build [PKG]` | 构建一个部署或单个软件包，含 ROS 工作空间 |
| `rbnx clean [--cache]` | 删除构建产物。`--cache` 会清掉远端软件包缓存 |
| `rbnx update` | 把 `url:` 声明的远端软件包更新到上游最新提交 |

### 运行

| 命令 | 作用 |
|---|---|
| `rbnx boot [-f MANIFEST]` | 按部署清单拉起整栈，阻塞直到 Ctrl-C |
| `rbnx shutdown` | 停掉 `rbnx boot` 拉起的所有组件 |
| `rbnx start -p PKG` | 单独启动一个软件包，boot 内部也用它 |

### 查看运行时状态

| 命令 | 作用 |
|---|---|
| `rbnx caps [-v]` | 列出当前注册到 Atlas 的能力，一个提供方一行 |
| `rbnx contracts` | 列出 Atlas 已加载的能力约定注册表 |
| `rbnx channels` | 列出已打开的消费方到提供方通道 |
| `rbnx tools` | 列出智能体可见的全部 MCP 工具 |
| `rbnx describe [--provider P]` | 显示已注册提供方的 `CAPABILITY.md` |
| `rbnx inspect` | 把完整运行时状态导出为 JSON |
| `rbnx logs [DIR]` | 读取 Scribe 日志文件并按标签或级别渲染 |

### 与智能体交互

| 命令 | 作用 |
|---|---|
| `rbnx chat` | 交互式 TUI，Esc 取消当前推理，Ctrl-C 退出 |
| `rbnx ask "..."` | 单轮任务，结果打到标准输出后退出 |

### 其他

| 命令 | 作用 |
|---|---|
| `rbnx init` | 初始化一个新的机器人部署目录 |
| `rbnx package-new` | 在对应角色目录下创建一个新软件包 |
| `rbnx path KEY` | 解析源码树内的路径。KEY 取 `root`、`rust`、`capabilities`、`interfaces-lib`、`runtime-proto`、`robonix-api` |
| `rbnx docs` | 重新生成手册的能力约定与 ROS IDL 参考 |

每个子命令的详细用法见 `rbnx <cmd> --help`。

## 常用流程

### 启动 Robonix

```bash
cd DEPLOY_DIR       # 机器人部署仓库的根目录
rbnx build          # 构建清单里声明的全部软件包
rbnx boot           # 拉起整栈，阻塞
```

另开一个终端确认：

```bash
rbnx caps           # 每个提供方一行，看状态是不是 ACTIVE
rbnx tools          # 智能体能调用的工具
rbnx ask "go to the kitchen"
```

### 只启动一个 Robonix Package

```bash
rbnx build -p ./packages/my_primitive
rbnx start -p ./packages/my_primitive
```

单独启动时软件包用自己源码里的默认 ID，不是部署清单里的实例名。

### 关闭 Robonix

```bash
rbnx shutdown
```

它会按逆序停掉 `rbnx boot` 拉起的组件。`rbnx boot` 的终端里按 Ctrl-C 效果相同。

## 内置系统组件

`rbnx boot` 按依赖顺序启动清单里声明的内置组件：Atlas、Executor、Soma、Pilot、Vitals、Liaison。清单里没有的组件不会启动，只有一个例外：部署包含原语或技能时会自动补上 Soma。

`system.<name>` 的处理分两种情况。`atlas`、`executor`、`pilot`、`liaison`、`soma`、`vitals` 是随 Robonix 发布的二进制，整个 `config` 块序列化成 JSON 由 `--config-json` 传入，其中若干字段另外翻译成独立的命令行参数。其余 `system:` 条目（例如 `scene`、`memory`、`speech`）是 `<robonix 源码>/system/<name>/` 下的软件包，与普通软件包一样通过 `Driver(CMD_INIT)` 收配置，不经过命令行。例如：

```yaml
system:
  vitals:
    listen: 127.0.0.1:50093
    log: info
```

## 配置如何到达软件包

软件包的实例配置写在部署清单里嵌套的 `config:` 映射中。`rbnx boot` 把它序列化后通过 `Driver(CMD_INIT)` 发给能力提供方。

每个原语、服务、技能条目的 `name` 就是它在 Atlas 上的提供方 ID，在一次部署内必须唯一，这一点由 Atlas 在注册时强制。`rbnx boot` 把这个身份作为 `RBNX_INSTANCE_NAME` 传给软件包。启动只等待这个确切 ID 的一次新注册，因此并发注册的无关提供方拿不到这个实例的生命周期配置。如果该 ID 在启动前已存在于 Atlas，启动会失败，不接管已有提供方。

## 环境变量

| 变量 | 默认值 | 作用 |
|---|---|---|
| `ROBONIX_DRIVER_INIT_TIMEOUT_S` | `90` | `Driver(CMD_ACTIVATE)` 的等待上限 |

`ROBONIX_DRIVER_INIT_TIMEOUT_S` 是排障工具，不是常规配置。一个组件在 90 秒里激活不完，通常是被某个外部资源阻塞（例如上一次部署没有完全退出、端口仍被占用），调大它只会把问题推后。先用日志确认时间花在哪一步。

## 定位常见问题

**某个软件包启动失败，说 ID 已经在 Atlas 上存在。** 上一次部署没有完全退出。先 `rbnx shutdown`，再用 `rbnx caps` 确认那个 ID 不在了。

**`rbnx boot` 报某个组件 `[FAIL]` 但进程还在。** 生命周期激活超时了。到 `rbnx-boot/logs/<组件>.log` 里看最后几行，判断是模型加载慢还是被资源阻塞。

**远端软件包没有更新。** `url:` 声明的软件包有缓存。用 `rbnx update` 拉上游最新提交。注意 `rbnx clean --cache` 会清掉缓存目录，其中的本地改动会一并消失。

**找不到源码树。** 在一个不是 Robonix 克隆的目录里调用了需要源码的子命令。跑一次 `rbnx setup` 登记，或者用 `rbnx path root` 确认当前登记的是哪一个。
