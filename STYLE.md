# Robonix documentation writing standard

This repository uses Chinese for the user-facing book. Keep identifiers, commands, filenames, contract paths, and product names in their canonical form.

## Write for an executable outcome

- Start a task page with its audience, prerequisites, and observable outcome.
- Use numbered steps only when order matters. Put one user action in each step.
- State the working directory before a command when it is not evident.
- Follow a command with the expected result or the next verification command.
- Separate required steps from optional alternatives and troubleshooting.
- Never describe planned behavior as implemented behavior.

## Use verified technical language

- Verify commands with the target commit's CLI help or actual execution.
- Verify configuration fields against the parser, `config.spec`, manifest schema, or standard contract.
- Use the same term for the same concept throughout the book. In Chinese prose, use the Chinese term first and add the canonical English name in parentheses on first use, for example 原语（Primitive）、服务（Service）、技能（Skill）、能力约定（Contract）、软件包（Package）和机器人部署仓库（Robot Deployment）。
- Keep code identifiers and abbreviations such as `provider_id` and RTDL in their canonical form, and define each Robonix-specific abbreviation on first use.
- Avoid conversational history, rhetorical questions, marketing superlatives, and instructions aimed at a particular colleague.

## Format commands and examples

- Use backticks for commands, paths, filenames, fields, environment variables, endpoints, and literal UI labels.
- Use fenced code blocks with a language tag where one is available.
- Use descriptive uppercase placeholders such as `ROBOT_IP` and `PACKAGE_DIR`. Explain every placeholder immediately before or after the example.
- Do not use `xxx`, fake IP addresses that look operational, or repository URLs that readers may mistake for real resources.
- Preserve only the relevant portion of command output. Mark omitted output explicitly and never invent success messages.

## Structure pages

- Use one `#` title, then a logical `##` and `###` hierarchy without skipped levels.
- Use sentence-style Chinese headings that describe the reader's task; do not number headings manually.
- Put procedures in operation guides, rationale in concept pages, and exact fields or types in reference pages.
- Link to the authoritative definition instead of copying a long block that can drift.
- Use tables for repeated properties, lists for short sets, and callouts only for a decision, risk, compatibility constraint, or expected result.

## Images and diagrams

- Add an image only when it clarifies a UI state, physical connection, geometry, or data flow that prose cannot express precisely.
- Provide concise alternative text and a caption that explains what the reader must notice.
- Crop secrets, personal information, browser chrome, and unrelated windows.
- For UI changes, verify both desktop and narrow-screen rendering.

## Safety and compatibility

- Physical-motion procedures must state prerequisites such as emergency stop availability, speed limits, raised wheels, or a cleared test area.
- Credential examples must use placeholders; never include a real token or secret.
- Compatibility notes must name the affected source branch, version or commit, platform, and replacement path.
- Deprecated behavior must be described as supported, warning, or removed according to current code; do not use vague terms such as “old” or “new.”

For editorial questions not covered here, follow the [Google Developer Documentation Style Guide](https://developers.google.com/style), prioritizing Robonix-specific terminology and reader clarity.
