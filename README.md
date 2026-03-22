# Robonix Book

Local build and serve:

```bash
# for searching in Chinese support
cargo install mdbook --git https://github.com/Sunshine40/mdBook --branch search-non-english --force
cargo install mdbook-mermaid
# Run once for RIDL syntax highlighting
./scripts/build-highlight.sh
# Or directly:
mdbook serve --open
```

RIDL code blocks need `theme/highlight.js` for syntax highlighting; run `./scripts/build-highlight.sh` to generate it.