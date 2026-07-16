# 代码接口文档

各 Rust crate 与 Python 包的逐项 API 文档，分别由 rustdoc（Rust）和 Sphinx（Python）从源码注释生成。

本页链接的线上 API 文档只对应站点当前声明的 `ROBONIX_SOURCE_REVISION`，不是独立维护的历史版本集。

手册顶部搜索用于查找概念、流程和标准接口页面。Rust API 与 Python API 是独立生成的文档树，进入对应页面后分别使用 rustdoc 或 Sphinx 自带的搜索框查询符号。

## Rust 接口文档

| Crate | 角色 | 文档 |
|---|---|---|
| `robonix-atlas` | 能力目录 / 注册中心 | [API](https://robonix.syswonder.org/api/rust/robonix_atlas/index.html) |
| `robonix-executor` | 方案编排与能力分发 | [API](https://robonix.syswonder.org/api/rust/robonix_executor/index.html) |
| `robonix-pilot` | 规划 / 决策 / 记忆 | [API](https://robonix.syswonder.org/api/rust/robonix_pilot/index.html) |
| `robonix-liaison` | 人机交互入口 | [API](https://robonix.syswonder.org/api/rust/robonix_liaison/index.html) |
| `robonix-codegen` | 能力约定 / IDL 代码生成 | [API](https://robonix.syswonder.org/api/rust/robonix_codegen/index.html) |
| `rbnx`（robonix-cli） | 开发 / 部署 CLI | [API](https://robonix.syswonder.org/api/rust/rbnx/index.html) |

本地生成需要一份已包含子模块、当前提交（HEAD）与本书 `ROBONIX_SOURCE_REVISION` 一致的 Robonix 源码检出。从**手册仓库根目录**运行：

```bash
export ROBONIX_SOURCE=/absolute/path/to/robonix
test -f "$ROBONIX_SOURCE/Cargo.toml"
test "$(git -C "$ROBONIX_SOURCE" rev-parse HEAD)" = "$(tr -d '[:space:]' < ROBONIX_SOURCE_REVISION)"

rm -rf build/api/rust
cargo doc --manifest-path "$ROBONIX_SOURCE/Cargo.toml" --locked --no-deps --workspace
mkdir -p build/api/rust
cp -R "$ROBONIX_SOURCE/target/doc/." build/api/rust/
```

## Python 接口文档

由 Sphinx（autodoc + napoleon + furo 主题）生成。

| 包 | 角色 | 文档 |
|---|---|---|
| `robonix-api` | Python 开发工具包（原语、服务、技能、Atlas、生命周期、通道） | [API](https://robonix.syswonder.org/api/python/_autosummary/robonix_api.html) |
| `scene`（scene_service） | 场景 / 语义地图服务的 Python 实现 | [API](https://robonix.syswonder.org/api/python/_autosummary/scene_service.html) |

> scene 是服务，它的**对外** API 是能力约定（见 [能力约定参考](contracts.md) 的 `robonix/system/scene/*`）；这里收录的是它的 Python 实现文档。

继续使用上面已验证的 `ROBONIX_SOURCE`，从**手册仓库根目录**运行：

```bash
make api-install ROBONIX_SOURCE="$ROBONIX_SOURCE"
make api-python \
  ROBONIX_SOURCE="$ROBONIX_SOURCE" \
  API_PYTHON=.venv-api/bin/python
```

`api-python` 在临时目录为 Scene 生成真实的 MCP 与 protobuf 类型，再交给 Sphinx 导入。临时目录在命令退出时删除，固定的 Robonix 源码检出保持只读且干净。

Rust 从各 crate 的入口页查看，例如 `build/api/rust/robonix_atlas/index.html`；Python 入口页是 `build/api/python/index.html`。

若要一次生成并检查手册和两套 API 文档，先安装一次 API 文档依赖，再运行完整检查：

```bash
make api-install ROBONIX_SOURCE="$ROBONIX_SOURCE"
make full-check \
  ROBONIX_SOURCE="$ROBONIX_SOURCE" \
  API_PYTHON=.venv-api/bin/python
```

完整本地预览使用：

```bash
make full-serve \
  ROBONIX_SOURCE="$ROBONIX_SOURCE" \
  API_PYTHON=.venv-api/bin/python
```

Rust 文档默认复用 `$ROBONIX_SOURCE/target`。系统盘空间不足时，可以把 Cargo 生成目录放到容量充足的磁盘；该选项只改变可再生成的构建缓存位置：

```bash
make full-check \
  ROBONIX_SOURCE="$ROBONIX_SOURCE" \
  API_PYTHON=.venv-api/bin/python \
  API_CARGO_TARGET_DIR=/absolute/path/to/cargo-target
```
