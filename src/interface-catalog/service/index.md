# 服务（service）

服务是**场景级算法/能力**——建图、导航、语音、声纹、记忆等。每条 `robonix/service/*` 能力约定 Robonix 都给一份默认参考实现（在 `services/` 或 example 部署里），部署方可以整包替换或扩展。

| 域 | 覆盖 | 能力约定数 | 参考实现 |
|----|------|--------|---------|
| [空间地图 map](map.md) | SLAM 输出：占据栅格、位姿、点云、里程 | 5 | `mapping_rbnx`（上游）/ FAST-LIO2 |
| [导航 navigation](navigation.md) | 目标式导航：下发目标、查状态、取消 | 4 | `examples/webots/services/simple_nav`（自研 A\*） |
| [语音 speech](speech.md) | ASR / TTS / 对话，一元与流式 | 8 | `services/speech` |
| [声纹 voiceprint](voiceprint.md) | 注册、识别、列举、删除 | 4 | `services/voiceprint` |
| [记忆 memory](memory.md) | 长期记忆的检索、写入、压缩归纳 | 3 | `services/memsearch` |
