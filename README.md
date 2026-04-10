# Robonix Book

**Language:** this book (`docs/`, mdBook sources under `docs/src/`) is maintained in **Chinese**. English prose belongs in code-tree READMEs (e.g. `rust/`), not here.

Local build and serve:

```bash
cargo install mdbook --git https://github.com/Sunshine40/mdBook --branch search-non-english --force
cargo install mdbook-mermaid --version 0.14.0
# 架构图使用 D2 预渲染 SVG；如需重新渲染：curl -fsSL https://d2lang.com/install.sh | sh -s --
mdbook serve --open
```
