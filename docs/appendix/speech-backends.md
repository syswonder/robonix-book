---
title: 语音后端配置
---

# 语音后端配置

Webots 快速上手默认使用本地 FunASR 完成流式语音识别，不需要额外设置 `SPEECH_BACKEND`。只有当部署者决定使用腾讯云语音服务时，才按本页切换后端。

## 使用腾讯云

Robonix 的腾讯云后端使用实时语音识别（ASR）和基础语音合成（TTS）。账号需要同时开通这两项服务。

### 1. 注册账号并开通服务

1. 按[腾讯云账号注册指南](https://cloud.tencent.com/document/product/378/17985)注册账号，并按控制台要求完成实名认证。
2. 进入[语音识别开通指南](https://cloud.tencent.com/document/product/1093/104735)指向的控制台，阅读服务条款并开通语音识别。
3. 进入[语音合成操作指引](https://cloud.tencent.com/document/product/1073/56639)指向的控制台，开通基础语音合成。
4. 在控制台的资源包页确认实时语音识别和语音合成的可用额度。语音识别的扣费顺序、免费额度和后付费开关以[腾讯云实时语音识别计费说明](https://cloud.tencent.com/document/product/1093/35686/)为准。

### 2. 创建访问密钥

进入访问管理的 [API 密钥管理](https://console.cloud.tencent.com/cam/capi)，创建用于 Robonix 部署的密钥。具体步骤和安全要求见[腾讯云访问密钥管理](https://cloud.tencent.com/document/product/598/40488)。

需要记录三个值：

| 腾讯云字段 | Robonix 配置 | 用途 |
|---|---|---|
| AppID | `TENCENT_ASR_APPID` | 实时语音识别请求地址中的账号标识；通过环境变量提供 |
| SecretId | `TENCENTCLOUD_SECRET_ID` | API 调用者标识；只通过启动环境传入 |
| SecretKey | `TENCENTCLOUD_SECRET_KEY` | API 签名密钥；只通过启动环境传入 |

SecretKey 只在创建时完整显示。不要把 AppID、SecretId、SecretKey 或包含它们的 `.env` 文件提交到 Git。生产部署应使用权限最小化的子用户密钥。

### 3. 配置部署清单

在机器人部署仓库的 `robonix_manifest.yaml` 中配置语音服务。下面的 `path` 与 Webots 部署保持一致；外部语音软件包应使用它自己的 `url`。

```yaml
service:
  - name: speech
    path: ${ROBONIX_SOURCE_PATH}/services/speech
    config:
      speech_backend: tencent
      tencent_asr_appid: "${TENCENT_ASR_APPID}"
      tencent_asr_engine: 16k_zh
      tencent_tts_voice_type: 1001
      tencent_tts_region: ap-guangzhou
      default_speaker_provider_id: audio_driver
```

`16k_zh` 是 16 kHz 中文通用引擎。腾讯云的[实时语音识别 WebSocket 文档](https://cloud.tencent.com/document/product/1093/48982)将 `16k_zh_en` 标为中英大模型引擎，使用不同的计费类别。只有基础实时识别额度时，不要把引擎改为 `16k_zh_en`。

### 4. 构建并启动

在执行构建的同一个终端选择腾讯云后端：

```bash
cd /path/to/robonix/examples/webots
SPEECH_BACKEND=tencent rbnx build
```

启动前把 AppID 和密钥放入当前终端的环境：

```bash
export TENCENT_ASR_APPID='1234567890'
export TENCENTCLOUD_SECRET_ID='AKID...'
export TENCENTCLOUD_SECRET_KEY='...'
rbnx boot
```

启动器将 `${TENCENT_ASR_APPID}` 展开后，会把 AppID、识别引擎和语音合成选项传给语音服务。SecretId 和 SecretKey 由语音服务直接从进程环境读取。

### 5. 验证

启动摘要应显示 `speech ACTIVE`。使用 Robonix Client 发起一次语音输入，确认客户端收到识别文本，再让机器人播报一句话以验证语音合成。失败时查看语音服务日志：

```bash
rbnx logs -t speech -l info
```

如果腾讯云返回 `4004` 并提示资源包耗尽，先在控制台确认当前资源包对应的产品和引擎，再核对部署清单中的 `tencent_asr_engine`。基础中文实时识别应使用 `16k_zh`；`16k_zh_en` 的额度不能由基础引擎资源包抵扣。

## Whisper 当前限制

Whisper 只用于本地后端的单次语音识别接口，不是 Robonix Client 连续语音交互的默认流式识别路径。当前 Whisper 集成尚不完善，默认模型 `whisper-large-v3` 的权重和构建缓存体积超过 20 GB。Webots 快速上手因此设置 `disable_whisper: true`，使用 FunASR 完成流式识别。当前版本不建议在快速上手或常规部署中启用 Whisper。
