# ridlc 开发手册

> RIDL 自定义语法（`.ridl` 文件）已弃用。详见 [RFC001 弃用通知](../rfc/RFC001-RIDL.md)。

## 当前状态

- `.ridl` 文件不再继续开发，保留在 `robonix-interfaces/ridl/` 作为参考
- ROS IDL（`.msg` / `.srv` / `.action`）是规范数据类型定义格式
- `ridlc` 已改造为跨传输代码生成工具，当前支持：
  - `--lang arrow`：从 ROS `.msg` 生成 Apache Arrow（`pyarrow.schema`）Python 模块
  - `--lang python`：旧版 Python/ROS2 工作区生成（已弃用）

## Arrow schema 生成

从 ROS `.msg` 文件生成 `pyarrow.Schema` 定义：

```bash
cd rust
cargo run -p ridlc -- --lang arrow \
  -I robonix-interfaces/lib/common_interfaces \
  -I robonix-interfaces/lib/robonix_msg \
  -I robonix-interfaces/lib/rcl_interfaces \
  -o robonix-interfaces/robonix_arrow
```

生成结果按 ROS 包组织（每包一个 `.py` 文件），包含所有消息类型对应的 Arrow schema。例如 `sensor_msgs.py` 导出 `Image`、`LaserScan`、`PointCloud2` 等。

### 在 Python 中使用

```python
import sys
sys.path.insert(0, "robonix-interfaces/robonix_arrow")

from sensor_msgs import Image, LaserScan
from geometry_msgs import Twist, PoseStamped

import pyarrow as pa
batch = pa.record_batch(data, schema=Image)
```

### 架构

```
.msg 文件  ──→  msg_parser（共享解析器）──→  arrow_gen  ──→  .py（pyarrow schema）
                        │
                        └──→  rust_gen   ──→  .rs（Rust 结构体，已弃用）
                        └──→  python_gen ──→  .py（ROS2 工作区，已弃用）
```

`msg_parser` 模块负责 ROS IDL 解析、跨包类型解析和命名空间推断。后端生成器消费解析后的 `MsgSpec` 类型。

## 规范数据定义

- Robonix 自定义类型：`robonix-interfaces/lib/robonix_msg/`
- 上游 ROS 类型：`robonix-interfaces/lib/common_interfaces/`、`robonix-interfaces/lib/rcl_interfaces/`
- 生成的 Arrow schema：`robonix-interfaces/robonix_arrow/`

## 参考

- [RFC001: RIDL](../rfc/RFC001-RIDL.md)
- [RFC006: 多传输支持](../rfc/RFC006-Multi-Transport.md)
- [ridlc README](https://github.com/syswonder/robonix/tree/main/rust/crates/ridlc)
