# 多平台部署：x86 与 Jetson（arm64）

Robonix 同一套源码可以部署到 **x86_64** 和 **NVIDIA Jetson（arm64 / JetPack）** 两类平台，
每类又分 **docker** 与 **native（无 docker）** 两种运行方式。本章讲清楚：

- 四种部署 target 的区别与适用场景；
- 每个包如何选择 target（构建期）与运行方式（运行期）；
- Jetson 平台的系统前置（JetPack 版本、系统 Python / ROS2、CUDA torch）；
- 怎么切换、怎么配置、怎么启动。

## 1. 部署 target 矩阵

| target | 架构 | 运行方式 | 典型场景 |
|---|---|---|---|
| `x86-docker`   | x86_64 | docker | 默认；webots 仿真、x86 工作站，镜像内带 ROS2 + CUDA(cu128) torch |
| `x86-native`   | x86_64 | 宿主直接跑 | x86 上已装好系统 ROS2 + torch，不想用 docker |
| `jetson-docker`| arm64  | docker | Jetson 上用 L4T 基础镜像（`docker/Dockerfile.jetson`） |
| `jetson-native`| arm64  | 宿主直接跑 | Jetson 上用 **JetPack 自带的** ROS2 + CUDA torch，**不**用 docker（小车默认） |

> **关键原则**：所有模型权重一律在**构建期**拉取，运行期（`start.sh`）只加载、绝不下载。
> docker target 在 `Dockerfile` 的 `RUN` 里 bake 进镜像；native target 在 `build.sh` 里预取到包的
> HF 缓存（`rbnx-build/data/hf`）。两者都是 build 时，机制不同而已。

## 2. 一个包怎么选 target

target 由 **per-target 的 package manifest** 决定，命名 `package_manifest.<target>.yaml`。每个这样的
manifest 的 `build:` / `start:` 行通过环境变量把 target 传给脚本：

- **构建期**：`RBNX_BUILD_TARGET=<target>`（脚本里 `TARGET="${RBNX_BUILD_TARGET:-x86-docker}"`，
  默认 `x86-docker`）。
- **运行期**：`ROBONIX_<PKG>_PLATFORM=jetson_orin`（或在部署 manifest 里设），脚本据此决定
  native/docker；也可用 `ROBONIX_<PKG>_FORCE=native|docker` 直接强制。

例如 scene 的 `scripts/build.sh`：

```bash
TARGET="${RBNX_BUILD_TARGET:-x86-docker}"
case "$TARGET" in
  jetson-native) ... 宿主 python + pip --user + 预取 CLIP 权重 ... ;;
  jetson-docker) SCENE_DOCKERFILE="docker/Dockerfile.jetson"; docker build ... ;;
  x86-docker)    SCENE_DOCKERFILE="docker/Dockerfile";        docker build ... ;;
esac
```

在部署 manifest（`robonix_manifest.yaml`）里，针对某个包指定平台即可切换，例如：

```yaml
  - name: scene
    path: .../system/scene
    manifest: package_manifest.jetson-native.yaml
    config:
      ROBONIX_SCENE_PLATFORM: jetson_orin     # 运行期走 native
```

`manifest:` 同时选择这个 target 的 `build:` 与 `start:`。例如 Scene 的 `package_manifest.jetson-native.yaml` 已在构建命令中设置 `RBNX_BUILD_TARGET=jetson-native`，并在启动命令中选择 native 运行方式，不需要在部署之外再执行一套不同的构建命令。

## 3. Jetson 平台系统前置

小车实测环境：**Jetson Orin，JetPack 6 / L4T r36.4.3 / CUDA 12.6 / aarch64 / Python 3.10 / ROS2 Humble**。

### 3.1 JetPack

- 刷 **JetPack 6.x**（对应 L4T r36.4.x）。确认版本：`cat /etc/nv_tegra_release`。
- JetPack 自带 CUDA 12.6、cuDNN、TensorRT。**不要**另装通用 CUDA。

### 3.2 系统 ROS2 与 Python

- 装 **ROS2 Humble**（arm64 deb）。Robonix 的 native 包直接 source `/opt/ros/humble/setup.bash`。
- 系统 **Python 3.10**（JetPack 默认）。native 包用 `uv` 建 venv，arch 轮子自动解析——
  **唯一例外是 torch（见 3.3）**。

### 3.3 CUDA torch（最容易踩的坑）

PyPI 的 aarch64 `torch` 轮子是按某个 CUDA 版本编译的（如 cu130），**与 JetPack 的 CUDA 12.6 驱动不匹配**
→ `torch.cuda.is_available()` 返回 False → 推理悄悄退回 CPU（语音 ASR、scene 感知会非常慢）。

正确做法是用 **JetPack 配套的 CUDA torch**：

- 优先用宿主已装好的、能跑 CUDA 的 torch（`python3 -c "import torch; print(torch.cuda.is_available())"` 为 True）。
- 或从 NVIDIA 的 Jetson pip 索引装匹配版本（注意官方域名换过，**当前是 `https://pypi.jetson-ai-lab.io/jp6/cu126`**；
  旧的 `pypi.jetson-ai-lab.dev` 可能失效，且部分 jp6/cu126 包有被下架的情况，必要时换 cu129 索引并核对驱动）。

**robonix 已经替你自动处理**：`scene` / `speech` / `voiceprint` 的 `build.sh` 在 Jetson 上（检测
`/etc/nv_tegra_release`）会判断 venv 里的 torch 是否能用 CUDA，不能就**自动复用宿主 python 的 JetPack
torch**（软链 `torch/torchaudio/torchvision/...` 进 venv，其余依赖仍来自 uv）。可用
`<PKG>_SKIP_JETSON_TORCH=1` 关闭该行为。

## 4. 各包的平台支持现状

| 包 | x86-docker | x86-native | jetson-docker | jetson-native | 备注 |
|---|---|---|---|---|---|
| scene      | ✅ | —  | ✅ | ✅ | x86-native 尚未提供 target |
| mapping    | ✅ | ✅ | ✅ | ✅ | rtabmap，native 用系统 ROS2 |
| nav2       | ✅ | ✅ | ✅ | ✅ | 系统 nav2 |
| speech     | ✅ | ✅ | ✅ | ✅ | uv venv + Jetson torch 自愈 |
| voiceprint | ✅ | ✅ | ✅ | ✅ | uv venv + Jetson torch 自愈 |
| memory     | ✅ | ✅ | ✅ | ✅ | uv venv（embedding 模型 build 期拉） |

> 注：用 `uv` 的纯 Python 包（speech/voiceprint/memory）天然支持四种 target——arch 轮子自动解析，
> docker 与否只影响是否进容器。只有**硬编码了 x86/CUDA 轮子**的包（scene 的 cu128 torch）需要专门的
> per-arch manifest 与 Dockerfile。scene 目前缺 `x86-native`（x86 上一般走 docker）。

## 5. 怎么启动

```bash
# 1) 在 robonix_manifest.yaml 的 package 条目中选择 manifest:
#    manifest: package_manifest.jetson-native.yaml
# 2) 构建并启动整个部署
rbnx build -f robonix_manifest.yaml
rbnx boot -f robonix_manifest.yaml
rbnx caps          # 确认各 provider ACTIVE
```

切回 x86 + docker：删除该条目的 `manifest:`，让包使用默认的 `package_manifest.yaml`。
