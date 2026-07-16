# 代码接口文档

各 Rust crate 与 Python 包的逐项 API 文档，分别由 rustdoc（Rust）和 Sphinx（Python）从源码注释生成。

## Rust 接口文档

| Crate | 角色 | 文档 |
|---|---|---|
| `robonix-atlas` | 能力目录 / 注册中心 | [API](https://robonix.syswonder.org/api/rust/robonix_atlas/index.html) |
| `robonix-executor` | 方案编排与能力分发 | [API](https://robonix.syswonder.org/api/rust/robonix_executor/index.html) |
| `robonix-pilot` | 规划 / 决策 / 记忆 | [API](https://robonix.syswonder.org/api/rust/robonix_pilot/index.html) |
| `robonix-liaison` | 人机交互入口 | [API](https://robonix.syswonder.org/api/rust/robonix_liaison/index.html) |
| `robonix-codegen` | 能力约定 / IDL 代码生成 | [API](https://robonix.syswonder.org/api/rust/robonix_codegen/index.html) |
| `rbnx`（robonix-cli） | 开发 / 部署 CLI | [API](https://robonix.syswonder.org/api/rust/rbnx/index.html) |

本地预览：`cargo doc --no-deps --workspace`（产物在 `target/doc/`，对应到 `api/rust/`）。

## Python 接口文档

由 Sphinx（autodoc + napoleon + furo 主题）生成。

| 包 | 角色 | 文档 |
|---|---|---|
| `robonix-api` | Python SDK（primitive / service / skill、atlas、lifecycle、Channel） | [API](https://robonix.syswonder.org/api/python/_autosummary/robonix_api.html) |
| `scene`（scene_service） | 场景 / 语义地图服务的 Python 实现 | [API](https://robonix.syswonder.org/api/python/_autosummary/scene_service.html) |

> scene 是服务，它的**对外** API 是能力约定（见 [能力约定参考](contracts.md) 的 `robonix/system/scene/*`）；这里收录的是它的 Python 实现文档。
>
> 本地预览：`pip install sphinx furo && sphinx-build -b html apidoc/python src/api/python`（在 `docs/` 下运行）。
