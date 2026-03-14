# ridlc 开发手册

本文档描述 ridlc 的代码生成规范与通信实现，供实现者与开发者参考。RIDL 语义定义见 [RFC001](../rfc/RFC001-RIDL.md)。

RIDL 语法速览（关键词、文件规则、命名等）见 RFC001 §3。

---

## 0. 文件与接口约定

- **一文件一接口**：每个 `.ridl` 文件只定义一个接口。
- **接口名 = 文件名**：接口名必须与文件名（不含扩展名）一致，如 `depth.ridl` 定义 `stream depth`，`move.ridl` 定义 `command move`。类似 Java：一个文件一个 public 类，类名与文件名一致。

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

RIDL 支持的注解（如 `@frame`、`@requires_interface`、`@interruptible`）见 RFC001 §5.2。ridlc 解析后传入 codegen，具体语义由生成代码与运行时实现。
