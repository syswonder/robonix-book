# Package 构建与代码生成

TODO：本页内容待更新。

本页讲**怎么用 `rbnx` 命令把一个 package 从源码构建成可运行状态**——包括定位 Robonix
主仓源码、生成 proto/MCP stubs、以及 package 自己的 build.sh 模板。

## 第一步：登记 Robonix 源码根（`rbnx setup`）

把当前 clone 登记为 robonix 源码根目录，写入 `~/.robonix/config.yaml`：

```bash
cd /path/to/robonix   # 仓库根目录
rbnx setup
```

`rbnx setup` 从当前目录向上寻找 `rust/Cargo.toml`、`rust/contracts`、
`rust/crates/robonix-interfaces/lib` 三个 marker；找到就登记，找不到就报错。
也支持显式传参 `rbnx setup /abs/path/to/robonix`。

**`make install` 会自动帮你跑一次 `rbnx setup`**，clone 完只要
`cd robonix/rust && make install`，终端会打印：

```
[make install] registering robonix_source_path → /home/xxx/dev/robonix
✓ robonix source path registered:
  /home/xxx/dev/robonix
```

### 旧配置文件迁移

如果 `~/.robonix/config.yaml` 没有 `robonix_source_path` 字段（老版本留下的），
任何 `rbnx build/start/validate/install/codegen` 会立即停止并提示：

```
[rbnx] config is missing robonix_source_path (legacy config from before the `rbnx setup` migration).
Fix:  cd /path/to/robonix
      rbnx setup
```

按提示跑一次即可。

## 第二步：生成 package 的 stubs（`rbnx codegen`）

**一条命令代替整个 build.sh 模板**：

```bash
rbnx codegen -p /path/to/my_package                 # 基础 proto + Python stubs
rbnx codegen -p /path/to/my_package --mcp           # 再生成 robonix_mcp_types/（MCP 包需要）
rbnx codegen -p /path/to/my_package --mcp --clean   # 先清理再生成
rbnx codegen -p /path/to/my_package --mcp --out-dir bridge   # 生成物放到 bridge/ 子目录
```

做的事（所有都在 `rbnx` 内部完成，不需要你手写）：

1. 调 `robonix-codegen --lang proto`，重刷 `<source>/rust/crates/robonix-interfaces/robonix_proto/`
2. 若加 `--mcp`，调 `robonix-codegen --lang mcp` 生成 `<pkg>/robonix_mcp_types/`（或 `<pkg>/<out-dir>/robonix_mcp_types/`）
3. 调 `grpc_tools.protoc` 把所有 proto 翻成 Python stubs，落到 `<pkg>/proto_gen/`
4. 写 `<pkg>/rbnx-build/ws/install/setup.bash`，导出 `PYTHONPATH`，`rbnx start` 会自动 source

## 第三步：package 的 build.sh 模板

绝大多数 package 只需要这几行：

```bash
#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
# Add --mcp if your package uses MCP tools (robonix_api.mcp_contract).
FLAGS+=(--mcp)

rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[build] done."
```

如果 package 还需要额外构建步骤（cargo build rust 库、docker compose build、
`pip install -e`），加在 `rbnx codegen` 之后即可。

### 参考：现有 package 的 build.sh

改造后的 build.sh 基本都在 10 行左右：

| Package | 额外步骤 |
|---------|----------|
| `examples/packages/vlm_service` | 无 |
| `examples/packages/memsearch_service` | 无（只需 `--mcp`） |
| `examples/packages/tiago_sim_stack` | `docker compose build`（`--out-dir tiago_bridge`） |
| `examples/packages/maniskill_vla_demo` | 多一步编 demo-local 的 `maniskill_env.proto` |
| `examples/packages/zero_copy_demo` | `cargo build -p robonix-buffer` + `pip install -e` |

## 进阶：package 自带 contract / IDL

按照约定，package 自己声明的 contract 放在：

```
my_package/
├── capabilities/              # 本地 contract TOML（mirror 主仓 rust/capabilities/）
│   └── my_custom.v1.toml
│   ├── msg/
│   └── srv/
└── scripts/build.sh
```

> ⚠️ **TODO**：`rbnx codegen` 目前**不会**自动 union 这两个目录到 system paths。
> 如果你的 package 有 `capabilities/` 或 `interfaces/lib/` 里的自定义内容，暂时需要
> 在 build.sh 里自己做合并（maniskill_vla_demo 保留了这段 staging 逻辑作为参考模板）。
> 未来 `rbnx codegen` 会原生支持。

Atlas 不会在 `ROBO_SYSTEM_INTERFACE_CATALOG` 里强制限制 package-local contract id
—— 它们出场时 atlas 只会打一条 "unknown contract (not in catalog) — allowing anyway"
warning，不影响功能。

## 常用命令速查

```bash
# 一次性初始化
cd /path/to/robonix && rbnx setup       # 自动 by `make install`

# 每次改了 contract/IDL 后重新生成所有 stubs
rbnx codegen -p /path/to/my_package --mcp --clean

# 查询当前配置
rbnx config --show

# 取某个路径（用于自己的脚本）
$(rbnx path contracts)
$(rbnx path interfaces-lib)
$(rbnx path robonix-api)
```
