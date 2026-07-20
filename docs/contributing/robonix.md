# 贡献 Robonix 代码

本页说明如何向 [`syswonder/robonix`](https://github.com/syswonder/robonix) 提交代码、能力约定、系统组件和示例。若要修改本手册，请阅读[参与维护 Robonix 文档](./documentation.md)。

## 1. 准备分支

Robonix 的日常开发集成分支是 `dev`。从最新的 `dev` 创建一个只解决单一问题的分支：

```bash
git clone https://github.com/syswonder/robonix.git
cd robonix
git switch dev
git pull --ff-only origin dev
git switch -c fix/short-description
```

分支名中的 `<type>` 可使用 `feat`、`fix`、`docs`、`refactor`、`test`、`perf`、`ci`、`build` 或 `chore`。例如：

```bash
git switch -c fix/pilot-cancel-state
```

开始修改前先阅读仓库根目录的 `AGENTS.md`，再阅读所改组件的 `README.md`。组件职责、标准能力约定和对外名称必须与当前实现一致；不要在一个修复中顺便重命名概念或重排无关代码。

## 2. 许可证与 SPDX 标识

Robonix 原创代码采用木兰宽松许可证第 2 版，SPDX 标识为 `MulanPSL-2.0`。新增原创源文件时，在文件开头写入与语言注释格式匹配的标识。

Rust、Python、TOML 和 YAML 文件：

```rust
// SPDX-License-Identifier: MulanPSL-2.0
```

```python
# SPDX-License-Identifier: MulanPSL-2.0
```

Shell 脚本保留 shebang 为第一行，SPDX 标识写在第二行：

```bash
#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
```

从其他项目引入或修改文件时，保留其已有许可证、版权声明和 SPDX 标识，不要把第三方代码改标为 `MulanPSL-2.0`。提交者必须确认代码来源和许可证兼容性；来源或授权不清楚时，不应提交。

## 3. 代码与目录规范

| 内容 | 约定 |
|---|---|
| Rust | 使用 Rust 2024 edition 和 `rustfmt`；模块与函数使用 `snake_case`，类型使用 `CamelCase`，crate 名使用 `robonix-*` |
| Python | 支持 Python 3.10 及以上版本；使用 4 空格缩进，模块与函数使用 `snake_case` |
| 能力约定 | 放在 `capabilities/` 对应分类下，文件名采用 `<interface>.v1.toml` |
| IDL | 可复用消息、服务和动作定义放在 `capabilities/lib/` 的对应库中 |
| 测试 | Rust 单元测试放在实现附近的 `#[cfg(test)]` 模块；集成测试放在 crate 的 `tests/`；Python 修改增加所属软件包可运行的测试或 smoke test |
| 组件说明 | 修改 `system/*` 或 `tools/*` 中 Rust 组件的功能时，同步更新该组件的 `README.md` |

函数或方法主体超过五行时，仓库要求注释至少说明其行为；签名不能表达的副作用和约束也应写清楚。不要手工修改 `docs/src/reference/` 下的生成文件，应修改能力约定、IDL 或生成器输入后重新生成。

Python 软件包没有统一的全仓库格式化命令。修改 Python 时，应运行该软件包已有的测试、类型检查或 smoke test；不要假设仓库使用未配置的 Black、Ruff 或其他工具。

## 4. 构建与检查

Rust 命令从 Robonix 仓库根目录执行。提交涉及 Rust 的修改前，至少运行：

```bash
make fmt
make check
cargo build --workspace
cargo test --workspace --all-targets
```

这些命令分别执行以下检查：

| 命令 | 实际作用 |
|---|---|
| `make fmt` | 使用 `cargo fmt --all` 修改 Rust 格式 |
| `make check` | 检查 `cargo fmt --all -- --check`，并以 `-D warnings` 运行 workspace clippy |
| `cargo build --workspace` | 构建整个 Rust workspace |
| `cargo test --workspace --all-targets` | 运行 workspace 的单元测试和集成测试 |

提交前再次运行 `make check`，确认格式化没有留下未提交改动：

```bash
git status --short
```

修改 Python 软件包、能力约定、代码生成、Webots 示例或跨进程通信时，还要运行对应目录 README、构建脚本或 CI workflow 中规定的专项测试。跨 Atlas、Driver 生命周期或 Python API 单例等进程边界的改动，需要提供端到端验证，不能只以“编译通过”作为完成依据。

## 5. 提交标题

提交标题使用 [Conventional Commits 1.0.0](https://www.conventionalcommits.org/en/v1.0.0/)：

```text
<type>(optional scope): <imperative description>
```

标题使用英文祈使语气，准确描述这一个提交完成的动作。常用类型如下：

| 类型 | 用途 | 示例 |
|---|---|---|
| `feat` | 新增用户可见能力 | `feat(atlas): expose provider health query` |
| `fix` | 修复错误行为 | `fix(pilot): preserve active plan on steer` |
| `docs` | 仅修改文档 | `docs: explain package configuration` |
| `refactor` | 不改变外部行为的重构 | `refactor(executor): isolate plan state transitions` |
| `test` | 新增或修正测试 | `test(scene): cover room lookup pagination` |
| `perf` | 性能改进 | `perf(scene): reduce point-cloud copies` |
| `ci` | 持续集成配置 | `ci: check commit authorship` |
| `build` | 构建系统或依赖 | `build: pin protobuf toolchain` |
| `chore` | 其他维护工作 | `chore: refresh generated notices` |

破坏兼容性的提交在类型或 scope 后加 `!`，并在正文中加入 `BREAKING CHANGE:` 说明迁移方法。一个提交只解决一个可独立审查的问题；标题不要使用 `update`、`changes` 或 `fix stuff` 等无法说明行为的表述。

## 6. 提交者身份与尾注

Git 的 author 和 committer 记录必须是实际承担责任的人。提交前确认身份：

```bash
git config user.name
git config user.email
```

提交正文末尾的 trailer 具有不同含义，不能互换或代他人添加：

| Trailer | 何时使用 |
|---|---|
| `Signed-off-by: Name <email>` | 人类提交者以 [Developer Certificate of Origin 1.1](https://developercertificate.org/) 证明自己有权按项目许可证提交该变更；使用 `git commit -s` 添加 |
| `Co-authored-by: Name <email>` | 另一位人类实际共同编写了提交 |
| `Co-developed-by: Name <email>` | 记录共同开发者；它表示人类作者身份，紧随该共同开发者自己的 `Signed-off-by` |
| `Reviewed-by: Name <email>` | 该人明确完成了技术审查并同意留下此记录 |
| `Tested-by: Name <email>` | 该人明确测试了本提交并同意留下此记录 |
| `Acked-by: Name <email>` | 相关维护者或利益相关者明确认可该修改 |
| `Fixes: <commit> (\"<subject>\")` | 修复由某个历史提交引入的回归；使用至少 12 位 commit ID 并附原提交标题 |
| `Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL ...]` | 披露人工智能工具对提交的实质辅助；不是作者、审查者或 DCO 签署者 |

Robonix 当前不会要求每个提交都包含 `Signed-off-by`。如果选择签署，签署人必须能亲自作出 DCO 声明，姓名和邮箱必须是其真实 Git 身份。不要复制他人的签署、审查或测试尾注，也不要自行添加对方没有明确给出的认可。

## 7. 人工智能辅助规则

自 2026 年 7 月起，提交到 Robonix 的 commit 必须由人类署名并承担责任：

- 人工智能编码代理不得出现在 Git author 或 committer 中；
- 不得通过 `Co-authored-by`、`Co-developed-by`、`Signed-off-by`、`Reviewed-by`、`Tested-by`、`Acked-by` 或 `Suggested-by` 把作者、法律声明、审查或测试责任归给人工智能；
- 人类提交者必须理解并审查完整改动，确认来源与许可证，运行相应验证，并对正确性、安全性和后续维护负责；
- 人工智能对提交有实质辅助时，使用 Robonix 的 `Assisted-by` 格式披露工具和模型。

```text
Assisted-by: Codex:gpt-5.6 clang-tidy
```

`Assisted-by` 不使用邮箱。冒号前是工具或代理名称，冒号后是具体模型版本，后面只列可选的专项分析工具；无需记录 Git、编译器、编辑器等普通开发工具。

这是 Robonix 自己的贡献政策。`Assisted-by` 的语法参考了 Linux kernel 社区公开的[人工智能编码助手规范](https://docs.kernel.org/process/coding-assistants.html)和[补丁提交指南](https://docs.kernel.org/process/submitting-patches.html#using-assisted-by)，用于借鉴其“人类承担责任、工具只记录辅助”的表达方式；这些链接只是格式参考，不定义 Robonix 的项目治理。

主仓库 CI 会检查拉取请求新增 commit 的 author、committer 和相关 trailer。检查只能识别提交记录中的违规身份，不能判断未披露的工具使用；如实披露仍是人类提交者的责任。可在本地检查准备提交的范围：

```bash
python3 scripts/check_commit_authorship.py --base origin/dev --head HEAD
```

## 8. 提交拉取请求

推送分支并向 `dev` 创建拉取请求：

```bash
git push -u origin HEAD
```

拉取请求应包含：

1. 要解决的问题和用户可见影响；
2. 实现方式及非显然的取舍；
3. 实际运行过的验证命令和结果；
4. 能力约定、生成代码、配置或迁移影响；
5. 相关 issue，例如在正文中写 `Closes #123`；
6. CLI、TUI 或网页修改的截图或终端输出。

所有 required checks 通过后再请求合并。不要用新提交掩盖失败检查，也不要把凭据、模型密钥、生成的秘密文件、构建产物或无关本地改动提交到仓库。

## 9. 提交前清单

- [ ] 修改只解决一个明确问题，分支基于最新 `dev`。
- [ ] 新增原创文件包含正确的 `MulanPSL-2.0` SPDX 标识；第三方文件保留原许可证。
- [ ] 代码风格、目录、能力约定和 README 与实现同步。
- [ ] Rust 修改已运行 `make fmt`、`make check`、workspace build 和 test。
- [ ] Python 或端到端修改已运行所属组件要求的专项验证。
- [ ] commit 标题符合 Conventional Commits，正文解释动机和影响。
- [ ] Git author、committer 和责任 trailer 均为真实人类身份。
- [ ] 实质使用人工智能辅助时，以 `Assisted-by` 披露，且人类已完成逐行审查和验证。
- [ ] 拉取请求列出验证证据、兼容性影响和相关 issue。
