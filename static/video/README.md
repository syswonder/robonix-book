# Console 仿真部署演示

`simulation-start-demo.mp4` 来自提供者于 2026-10-22 提供的 Ranger MuJoCo 操作录像，展示 Console 安装、部署和启动；不是 TIAGo Webots 的运行录像。

最终提交使用完整时间线的转码文件，保留 1920×1080、60 fps 画面，原始 AAC 双声道音轨直接复制。时长约 8 分 30 秒，无片段截取。转码后已完整解码检查。

| 文件 | 字节数 | SHA-256 |
|---|---:|---|
| 提供者原始 MP4 | 143069993 | `a12eacbc2d782135eee535b64da3fcb0bbfff32cbb2d050e5eea91d0875f51b9` |
| 本目录 MP4 | 38495536 | `1ba9c50f745ef6902987c83e3a4752f4f6af2c1ef18fe880675af814b3bc18ce` |

转码参数（`SOURCE_VIDEO.mp4` 代表原始文件）：

```bash
ffmpeg -i SOURCE_VIDEO.mp4 -map 0:v:0 -map '0:a:0?' \
  -c:v libx264 -preset fast -crf 18 -pix_fmt yuv420p \
  -c:a copy -movflags +faststart simulation-start-demo.mp4
```

原始文件大于 GitHub 普通文件的 100 MiB 限制，因此使用上述转码文件。
