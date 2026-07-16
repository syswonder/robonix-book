# 代码接口文档

各 Rust crate 与 Python 包的逐项 API 文档，分别由 rustdoc（Rust）和 Sphinx（Python）从源码注释生成。

本页链接的线上 API 文档只对应站点当前声明的 `ROBONIX_SOURCE_REVISION`，不是独立维护的历史版本集。

## Rust 接口文档

| Crate | 角色 | 文档 |
|---|---|---|
| `robonix-atlas` | 能力目录 / 注册中心 | [API](https://robonix.syswonder.org/api/rust/robonix_atlas/index.html) |
| `robonix-executor` | 方案编排与能力分发 | [API](https://robonix.syswonder.org/api/rust/robonix_executor/index.html) |
| `robonix-pilot` | 规划 / 决策 / 记忆 | [API](https://robonix.syswonder.org/api/rust/robonix_pilot/index.html) |
| `robonix-liaison` | 人机交互入口 | [API](https://robonix.syswonder.org/api/rust/robonix_liaison/index.html) |
| `robonix-codegen` | 能力约定 / IDL 代码生成 | [API](https://robonix.syswonder.org/api/rust/robonix_codegen/index.html) |
| `rbnx`（robonix-cli） | 开发 / 部署 CLI | [API](https://robonix.syswonder.org/api/rust/rbnx/index.html) |

本地生成需要一份已包含 submodule、HEAD 与本书 `ROBONIX_SOURCE_REVISION` 一致的 Robonix 源码 checkout。从 **Book 仓库根目录**运行：

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
| `robonix-api` | Python SDK（primitive / service / skill、atlas、lifecycle、Channel） | [API](https://robonix.syswonder.org/api/python/_autosummary/robonix_api.html) |
| `scene`（scene_service） | 场景 / 语义地图服务的 Python 实现 | [API](https://robonix.syswonder.org/api/python/_autosummary/scene_service.html) |

> scene 是服务，它的**对外** API 是能力约定（见 [能力约定参考](contracts.md) 的 `robonix/system/scene/*`）；这里收录的是它的 Python 实现文档。

继续使用上面已验证的 `ROBONIX_SOURCE`，从 **Book 仓库根目录**运行：

```bash
python3 -m venv .venv-api
. .venv-api/bin/activate
python -m pip install sphinx furo numpy grpcio protobuf pyyaml
python -m pip install "$ROBONIX_SOURCE/pylib/robonix-api"

rm -rf build/api/python
mkdir -p build/api/python
ROBONIX_SPHINX_MOCKS="torch,rclpy,cv2,open3d,scipy,sklearn,semantic_map_mcp" \
ROBONIX_API_SRC="$ROBONIX_SOURCE/pylib/robonix-api" \
ROBONIX_SCENE_SRC="$ROBONIX_SOURCE/system/scene" \
  python -m sphinx -b html apidoc/python build/api/python
```

Rust 从各 crate 的入口页查看，例如 `build/api/rust/robonix_atlas/index.html`；Python 入口页是 `build/api/python/index.html`。
