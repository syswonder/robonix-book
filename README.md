# Robonix Book

Robonix Book is the Chinese documentation site for Robonix. The current handbook is built with Docusaurus and aligned with the `dev-next` branch of [`syswonder/robonix`](https://github.com/syswonder/robonix/tree/dev-next). The archived mdBook source and tooling remain under `archive/mdbook/` for history and URL migration checks; contributors should edit the Markdown files under `docs/`.

## Local preview

Prerequisites:

- Node.js 20 or newer;
- npm, which is included with the official Node.js distribution;
- GNU Make.

Run the following commands from the repository root:

```bash
make install
make dev
```

Open <http://127.0.0.1:3000/>. Docusaurus watches `docs/`, `src/`, `static/`, `sidebars.ts`, and `docusaurus.config.ts` and refreshes the development preview after a saved change.

Before opening a pull request, run:

```bash
make check
```

This command checks fenced Bash, JavaScript, JSON, Python, TOML, and YAML examples; validates local image paths and alternative text; runs the TypeScript check; builds the production site in `build/`; verifies internal links, legacy mdBook URLs and fragments; and checks the generated search index. Use `make help` to list every supported target.

The normal preview covers the handbook. To reproduce the deployed artifact with the Rust and Python API trees, use a clean Robonix checkout at the revision in `ROBONIX_SOURCE_REVISION`:

```bash
make api-install ROBONIX_SOURCE=/absolute/path/to/robonix
make full-check \
  ROBONIX_SOURCE=/absolute/path/to/robonix \
  API_PYTHON=.venv-api/bin/python
make full-serve \
  ROBONIX_SOURCE=/absolute/path/to/robonix \
  API_PYTHON=.venv-api/bin/python
```

## Contributing

Read the [documentation contribution guide](docs/contributing/documentation.md) and [writing standard](STYLE.md) before changing technical instructions. Commands, configuration fields, paths, expected output, and implementation status must be verified against the exact Robonix source revision described by the page.

## License

Robonix Book is licensed under the [Mulan Permissive Software License, Version 2](LICENSE). Notices for incorporated third-party source files are listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
