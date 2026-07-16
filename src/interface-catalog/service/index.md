# 服务（service）

服务是**场景级算法/能力**——建图、导航、语音、声纹、记忆等。每条 `robonix/service/*` 能力约定 Robonix 都给一份**默认参考实现**，部署方可以整包替换或扩展。

默认实现分两类：**上游独立仓库**（`url:` 引入，部署时 clone 到 `rbnx-boot/cache/`，可用 [`rbnx update`](../../integration-guide/build-and-codegen.md#7-更新远端-package) 同步到最新）和 **robonix 仓库内**（`services/` 目录）。每个都带 README / CAPABILITY.md，是该服务的权威部署文档：

| 域 | 覆盖 | 能力约定数 | 默认参考实现 | 文档参考 |
|----|------|--------|---------|---------|
| [空间地图 map](map.md) | SLAM 输出、地图生命周期与持久化管理 | 15 | [`service-map-rbnx`](https://github.com/syswonder/service-map-rbnx)（上游，rtabmap） | [README](https://github.com/syswonder/service-map-rbnx/blob/main/README.md) · [CAPABILITY](https://github.com/syswonder/service-map-rbnx/blob/main/CAPABILITY.md) |
| [导航 navigation](navigation.md) | 目标式导航：下发目标、查状态、取消 | 4 | [`service-navigation-rbnx`](https://github.com/syswonder/service-navigation-rbnx)（上游，封装 Nav2） | [README](https://github.com/syswonder/service-navigation-rbnx/blob/main/README.md) · [CAPABILITY](https://github.com/syswonder/service-navigation-rbnx/blob/main/CAPABILITY.md) |
| [语音 speech](speech.md) | ASR / TTS / 对话 / 唤醒词，一元与流式 | 9 | `services/speech` | [README](https://github.com/syswonder/robonix/blob/dev-next/services/speech/README.md) |
| [声纹 voiceprint](voiceprint.md) | 注册、识别、列举、删除 | 4 | `services/voiceprint` | [README](https://github.com/syswonder/robonix/blob/dev-next/services/voiceprint/README.md) |
| [记忆 memory](memory.md) | 长期记忆的检索、写入、压缩归纳 | 3 | `services/memsearch` | [README](https://github.com/syswonder/robonix/blob/dev-next/services/memsearch/README.md) |

> 场景/语义地图 [scene](../system/scene.md) 是 **system** 组件（不在 `service/` 域），默认实现在 robonix 仓库 [`system/scene`](https://github.com/syswonder/robonix/blob/dev-next/system/scene/README.md)。
