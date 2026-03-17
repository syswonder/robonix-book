# Robonix Book

Local build and serve:

```bash
cargo install mdbook mdbook-toc mdbook-mermaid
# Run once for RIDL syntax highlighting
./scripts/build-highlight.sh
# Or directly:
mdbook serve --open
```

RIDL code blocks need `theme/highlight.js` for syntax highlighting; run `./scripts/build-highlight.sh` to generate it.