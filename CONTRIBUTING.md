# Contributing to Robonix documentation

Robonix documentation is maintained in the open. Before changing a page, read the Chinese [documentation contribution guide](docs/contributing/documentation.md) and [writing standard](STYLE.md).

The published book currently documents the experimental `dev-next` branch of `syswonder/robonix`. The `dev` branch is the recommended, more stable development baseline; the Robonix source `main` branch is reserved for milestone releases. Always state and verify the source branch and commit described by a pull request.

## Contribution path

- Open a pull request directly for a typo, broken link, or narrow clarification that does not change behavior.
- Open an issue first for incorrect commands or behavior, new information architecture, cross-repository changes, compatibility policy, or documentation for code that has not merged.
- Use one branch and one pull request for one independently reviewable change.
- Fill every section of the pull request template. Link the issue when one exists.

## Local verification

Install the locked dependencies once, then start the editable preview:

```bash
make install
make dev
```

Open <http://127.0.0.1:3000/> and inspect every changed page. Before opening a pull request, run:

```bash
make check
```

The production site is written to `build/`. For command, configuration, interface, or hardware instructions, also execute the documented procedure against the stated Robonix commit. Include the evidence in the pull request.

Pull requests require review from the paths' code owners. Changes to interfaces, deployment, compatibility, credentials, or physical robot safety require both documentation and domain review. The author cannot be the only approver.

Generative AI use is allowed, but it must be disclosed in the pull request. The contributor remains responsible for source verification, command execution, licensing, privacy, and the final text.
