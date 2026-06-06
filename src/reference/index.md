# 参考

本章是 Robonix 的机读参考，分两部分：

- [契约参考](contracts.md) 与 [ROS IDL 参考](idl.md)——由 `rbnx docs` 从 `capabilities/` **自动生成**，每页顶部标明生成自 robonix 的哪个 commit。这两页是契约 + 数据结构的权威来源；[接口目录](../interface-catalog/index.md) 里的散文页讲"为什么 / 怎么组合"，原始 schema 以这里为准。
- [代码 API](api.md)——各 Rust crate / Python 包的 rustdoc / Sphinx 入口。

> 自动生成的两页请勿手改；改了契约或 IDL 后在 robonix 源码树下跑 `rbnx docs` 重新生成并提交。这两页是纯 markdown，本书的 mdBook / GitHub Pages 构建不需要 robonix 环境。
