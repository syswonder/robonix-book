# 文档平台与迁移边界

本页只记录文档平台选择及迁移触发条件。页面归属、审查要求、PR 内容和版本回补流程见[文档贡献与维护流程](contributing/documentation.md)。

## 为什么目前继续使用 mdBook

现阶段 mdBook 足以覆盖单一主要语言、单一活动实验版本、Markdown 内容、全文搜索、自动生成接口参考和 GitHub Pages 发布。它的构建链短，贡献者无需 Node.js 前端工程经验即可修改正文；当前需要的分支标识、任务卡片和本地链接检查也可以通过轻量主题与脚本完成。现在迁移会增加内容搬运和主题维护成本，却不会直接提高步骤准确性。

工具版本必须固定在 CI 中。升级 mdBook、`mdbook-mermaid` 或 `mdbook-toc` 时，应在一个独立 PR 中同时验证桌面端、移动端、搜索、Mermaid、页内目录和全部本地链接。

每个 PR 必须通过 mdBook 构建、本地页面与锚点检查，以及从 `dev-next` 重新生成的 Contract/IDL 差异检查。外部网站可能受网络、限流或临时故障影响，因此由定时任务检查并报告，不作为 PR 合并阻塞项。这与 ROS 2 文档仓库采用可复现构建和持续检查的方向一致；社区协作中的 Owner/Reviewer 分工则参考 AOSP 的公开贡献模型。参考：[ROS 2 Documentation](https://github.com/ros2/ros2_documentation)、[AOSP 参与指南](https://source.android.com/docs/setup/contribute)。

## 何时迁移到 Docusaurus

当以下任一条件成为持续需求时，启动迁移提案，而不是继续叠加主题脚本：

- 需要同时维护三个及以上可选择的 Robonix 文档版本；
- 需要维护两个及以上完整语言版本，并由社区持续翻译；
- 需要跨版本、版本感知的托管搜索；
- 需要在正文中维护交互式配置器、兼容性矩阵或组件化 API 浏览器；
- 自定义主题与构建胶水已经成为主要维护负担。

首选迁移目标是 Docusaurus，因为它原生提供[文档版本管理](https://docusaurus.io/docs/versioning)、[国际化工作流](https://docusaurus.io/docs/i18n/introduction)和[可扩展搜索集成](https://docusaurus.io/docs/search)。迁移前必须先制作内容转换原型，证明现有 URL 重定向、中文锚点、Contract/IDL 自动生成、页面编辑链接和 CI 预览均可保留；在这些验收项完成前继续使用 mdBook。
