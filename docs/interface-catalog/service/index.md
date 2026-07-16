# 服务

服务（Service）是**场景级算法或能力**，包括建图、导航、语音、声纹和记忆等。Robonix 为每个服务域提供默认参考软件包，部署方可以整包替换或扩展。能力约定、软件包清单和实现注册是三层信息；具体传输和运行时状态以各服务页为准。

默认实现分两类：**上游独立仓库**（`url:` 引入，部署时克隆到 `rbnx-boot/cache/`，可用 [`rbnx update`](../../integration-guide/build-and-codegen.md#7-更新远端软件包) 同步）和 **Robonix 仓库内实现**（`services/` 目录）。Mapping / Navigation 的说明固定到已核对的 `main` commit；仓库内实现固定到 Robonix `8c2551ce`，避免可变分支或过期 README 造成漂移。

| 域 | 覆盖 | 能力约定数 | 默认参考实现 | 维护者 | 文档参考 |
|----|------|--------|---------|------|---------|
| [空间地图](map.md) | SLAM 输出、地图生命周期与持久化管理 | 15 | [`service-map-rbnx`](https://github.com/syswonder/service-map-rbnx/tree/ce4092a1bee8847d6314af957f0225c8371d9aa6)（`main@ce4092a`） | `wheatfox` | [package_manifest](https://github.com/syswonder/service-map-rbnx/blob/ce4092a1bee8847d6314af957f0225c8371d9aa6/package_manifest.yaml) · [实现注册](https://github.com/syswonder/service-map-rbnx/blob/ce4092a1bee8847d6314af957f0225c8371d9aa6/src/mapping_rbnx/atlas_bridge.py) |
| [导航](navigation.md) | 目标式导航：下发目标、查状态、取消 | 4 | [`service-navigation-rbnx`](https://github.com/syswonder/service-navigation-rbnx/tree/b1a923a25cb3bf75554b861fceb605a190ae641b)（`main@b1a923a`） | `wheatfox` | [package_manifest](https://github.com/syswonder/service-navigation-rbnx/blob/b1a923a25cb3bf75554b861fceb605a190ae641b/package_manifest.yaml) · [实现注册](https://github.com/syswonder/service-navigation-rbnx/blob/b1a923a25cb3bf75554b861fceb605a190ae641b/nav2_wrapper/atlas_bridge.py) |
| [语音](speech.md) | ASR、TTS、唤醒词；`dialog` 当前实现不可用 | 9 | `services/speech` | [@kaileliu](https://github.com/kaileliu) | [package_manifest](https://github.com/syswonder/robonix/blob/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/speech/package_manifest.yaml) · [能力约定](https://github.com/syswonder/robonix/tree/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/capabilities/service/speech) |
| [声纹](voiceprint.md) | 注册、识别、列举、删除；`delete` 已实现但 manifest 漏列 | 4 | `services/voiceprint` | [@kaileliu](https://github.com/kaileliu) | [package_manifest](https://github.com/syswonder/robonix/blob/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/voiceprint/package_manifest.yaml) · [能力约定](https://github.com/syswonder/robonix/tree/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/capabilities/service/voiceprint) |
| [记忆](memory.md) | 长期记忆的检索、写入、压缩归纳 | 3 | `services/memsearch` | [@ohhhHwH](https://github.com/ohhhHwH) | [package_manifest](https://github.com/syswonder/robonix/blob/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/services/memsearch/package_manifest.yaml) · [能力约定](https://github.com/syswonder/robonix/tree/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/capabilities/service/memory) |

> [场景理解](../system/scene.md) 属于系统组件，不在 `robonix/service/*` 能力域；默认实现位于 Robonix 仓库的 [`system/scene`](https://github.com/syswonder/robonix/tree/8c2551ce402b7afe77245a4bd4e87c9ebbc2e4c7/system/scene)。
