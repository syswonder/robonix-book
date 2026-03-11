# robonix 手册

本地部署：

```bash
cargo install mdbook mdbook-toc mdbook-mermaid
# 首次或需 RIDL 语法高亮时运行
./scripts/build-highlight.sh
# 或直接 mdbook serve --open
mdbook serve --open
```

RIDL 代码块需在 `theme/highlight.js` 生成后才有语法高亮，运行 `./scripts/build-highlight.sh` 即可。