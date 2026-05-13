# Package 构建与代码生成

讲怎么用 `rbnx` 把一个 package 从源码构建到可运行——登记 Robonix 源码根、生成 stubs、`build.sh` 模板、package 自带 contract / IDL。

## 第一步：登记 Robonix 源码根（`rbnx setup`）

把当前 clone 登记为 robonix 源码根目录，写入 `~/.robonix/config.yaml`：

```bash
cd /path/to/robonix     # 仓库根目录
rbnx setup
```

`rbnx setup` 在当前目录及其上级查 `rust/Cargo.toml`、`capabilities`、`rust/crates/robonix-interfaces/lib` 三个 marker 都齐了才登记，缺一个直接报错。也支持显式传参 `rbnx setup /abs/path/to/robonix`。

**`make install` 会自动跑一次 `rbnx setup`**——clone 完只要 `cd robonix/rust && make install` 即可。

### 旧配置文件迁移

如果 `~/.robonix/config.yaml` 没有 `robonix_source_path` 字段（老版本留下的），后续 `rbnx build / start / validate / install / codegen` 会立刻停下并提示：

```
[rbnx] config is missing robonix_source_path (legacy config from before the `rbnx setup` migration).
Fix:  cd /path/to/robonix && rbnx setup
```

跑一次即可。

## 第二步：生成 package 的 stubs（`rbnx codegen`）

```bash
rbnx codegen -p /path/to/my_package                 # 生成 proto + Python stubs
rbnx codegen -p /path/to/my_package --mcp           # 同时生成 robonix_mcp_types/（MCP 工具包需要）
rbnx codegen -p /path/to/my_package --mcp --clean   # 先清理旧产物再生成
rbnx codegen -p /path/to/my_package --mcp --out-dir bridge   # 产物落到 <pkg>/bridge/ 子目录
```

做的事（全在 `rbnx` 内部）：

1. 把 robonix 源码仓库的 `capabilities/` 和 package 自己的 `capabilities/` 合并，扫出当前 package 能引用的 contract 全集（同 `contract_id` 包内覆盖全局）
2. 调 `robonix-codegen` 生成 proto 描述
3. 若加 `--mcp`，再生成 MCP dataclass 到 `<pkg>/robonix_mcp_types/`
4. 调 `grpc_tools.protoc` 把 proto 翻成 Python stubs，落到 `<pkg>/proto_gen/`
5. 不需要写 `PYTHONPATH`——`robonix_api` 第一次 import 时会自动把上面两个目录加进 `sys.path`

## 第三步：package 的 build.sh 模板

绝大多数 package 一句话：

```bash
#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
# 用了 @<provider>.mcp(...) 的包加上 --mcp
FLAGS+=(--mcp)

rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[build] done."
```

需要额外构建（cargo 编 Rust 库、`pip install -e`、docker compose build 等）的，加在 `rbnx codegen` 之后即可。

### 参考：仓库内现有 build.sh

仓库自带的 webots 例子 + system 服务都遵循上面这套模板：

| Package | 路径 | 额外步骤 |
|---|---|---|
| `tiago_chassis` / `tiago_camera` / `tiago_lidar` / `audio_driver` | `examples/webots/primitives/` | 无 |
| `simple_nav` | `examples/webots/services/simple_nav` | 无 |
| `scene` | `system/scene` | docker compose build（perception 容器内跑 YOLO-World + MobileSAM）|
| `memory` | `system/memory` | 无 |
| `speech` | `system/speech` | 无（首次跑会下 FunASR / TTS 权重）|

## 进阶：package 自带 contract / IDL

如果包要暴露一个**官方仓库没有**的 contract（典型：skill 包），把契约写在包自己的 `capabilities/` 下：

```
my_package/
├── capabilities/                # 包内 contract
│   ├── my_custom.v1.toml
│   ├── msg/
│   └── srv/
├── package_manifest.yaml
└── scripts/build.sh
```

`package_manifest.yaml` 里 `capabilities:` 引用时**带 `path:`** 指向包内 TOML：

```yaml
capabilities:
  - name: robonix/skill/my_stack/my_custom
    path: capabilities/my_custom.v1.toml
```

`rbnx codegen` 扫描两种 `capabilities/` 根（robonix 源码 + 包内）合并，包内 contract 跟官方 contract 一样进 stub。无需在 `build.sh` 里手动 staging。

IDL 路径解析规则：

- **官方 TOML**（在 robonix 源码 `capabilities/` 里）：`[io.srv] srv = "lidar/srv/Foo"` → 去 `rust/crates/robonix-interfaces/lib/lidar/srv/Foo.srv` 找
- **包内 TOML**（在你 package 的 `capabilities/` 里）：`[io.srv] srv = "srv/Foo"` → 去**包的 `capabilities/srv/Foo.srv`** 找

atlas 对包内 contract id **不强制**校验前缀；只要 namespace 跟能力提供者的 namespace 一致（`DeclareCapability` 会校验），随便用 `myorg/...` 前缀都行——atlas 只在 contract 注册表里找不到时 log 一条 debug，不影响功能。

## 常用命令速查

```bash
# 一次性初始化
cd /path/to/robonix && rbnx setup       # make install 已自动跑

# 改了 contract/IDL 后重生所有 stubs
rbnx codegen -p /path/to/my_package --mcp --clean

# 查询当前配置
rbnx config --show

# 取某个路径（用于自己的脚本）
$(rbnx path root)
$(rbnx path capabilities)
$(rbnx path interfaces-lib)
$(rbnx path robonix-api)
```

合法 path key：`root` / `rust` / `capabilities` / `interfaces-lib` / `runtime-proto` / `robonix-api`。
