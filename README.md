# Robonix Book

**Language:** this book (`docs/`, mdBook sources under `docs/src/`) is maintained in **Chinese**. English prose belongs in code-tree READMEs (e.g. `rust/`), not here.

Local build and serve:

```bash
cargo install mdbook
cargo install mdbook-mermaid --version 0.17.0
cargo install mdbook-toc                       # [toc] 页内目录预处理器
# 架构图使用 D2 预渲染 SVG；如需重新渲染：curl -fsSL https://d2lang.com/install.sh | sh -s --
./scripts/build-highlight.sh                   # 生成带 ROS IDL 高亮的 theme/highlight.js（首次必跑）
mdbook serve --open
```

> 直接 `mdbook serve` 也能跑，但缺两样：没装 `mdbook-toc` 会报 preprocessor not found；没跑过 `build-highlight.sh` 则 `.msg`/`.srv` 代码块不会高亮。CI（`.github/workflows/mdbook.yml`）也是这套步骤。
