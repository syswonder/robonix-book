# AI 机器人配置助手

Robonix Console 右侧的 `AI 机器人配置助手` 用一句话完成配置相关的工作：检查当前配置有没有问题、修改参数、添加或停用软件包、回滚版本、启动或停止 Robonix、查看运行日志、按能力在目录中查找软件包、查看云端用量。

第一次打开 Robonix Console 时，桌面端的助手面板默认展开；收起后，右下角的 `AI 机器人配置助手` 按钮可以再次打开它。手机上助手以全屏面板显示。

## 模型服务

助手使用这台机器人为 Pilot 配置的模型服务，即 `设置` 页 `机器人环境变量` 中的 `VLM 终端 URL`、`VLM KEY` 和 `VLM 模型` 三项（对应 `VLM_BASE_URL`、`VLM_API_KEY`、`VLM_MODEL`）。三项未填写时，助手面板提示先填写，并提供 `去设置` 的链接。模型服务需兼容 OpenAI 的 Chat Completions 接口，并支持工具调用。

## 使用方式

在面板底部的输入框中输入要做的事，按回车发送；空白对话中也可以直接点示例。助手先调用工具读取所需的信息，面板中逐条列出它调用的工具，然后给出回答。

<div className="shot-row">
  <figure>
    <img src="/img/hub/11-assistant-empty.webp" alt="打开助手后的空白对话，列出几个示例。" />
    <figcaption>打开助手</figcaption>
  </figure>
  <figure>
    <img src="/img/hub/11-assistant-check.webp" alt="助手调用工具检查部署后给出回答。" />
    <figcaption>检查当前配置</figcaption>
  </figure>
  <figure>
    <img src="/img/hub/11-assistant-approve.webp" alt="助手修改配置前显示确认卡片，附带修改前后的对比。" />
    <figcaption>修改前请求同意</figcaption>
  </figure>
</div>

会改变机器人状态的操作，例如修改配置、添加软件包、回滚版本、启动或停止 Robonix、上传到云端、更新 Robonix Console，助手不会直接执行，而是先显示确认卡片：写明要做的操作，修改部署清单时附带修改前后的对比。点 `同意` 后才执行，点 `拒绝` 则放弃。修改配置时，助手把结果保存为新版本，可以在 `历史` 页回滚。设置机器人环境变量时，值由用户在确认卡片中填写，模型看不到这个值。

删除部署目录、用导入覆盖历史以及账户相关的操作不对助手开放。点面板标题栏的刷新图标开始新的对话；对话只保存在当前浏览器标签页中。

[在线演示](https://hub.robonix.ai/demo/) 中的助手使用 Robonix Hub 提供的模型，每个访客有使用额度；演示中同意的修改只保存在访客自己的浏览器中。
