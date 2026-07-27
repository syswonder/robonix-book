from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "normalize-reference.py"
SPEC = importlib.util.spec_from_file_location("normalize_reference", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
NORMALIZE_REFERENCE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NORMALIZE_REFERENCE)


class GroupLegacyDriverContractsTest(unittest.TestCase):
    def test_groups_namespace_drivers_regardless_of_description(self) -> None:
        text = """# 能力约定参考（自动生成）

## shared

| 能力约定 ID | 接口含义 | kind | mode | 载荷（IDL） | 能力约定 TOML |
|---|---|---|---|---|---|
| `robonix/primitive/hand/driver` | - | primitive | `rpc` | driver | hand |
| `robonix/primitive/quadruped/driver` | Lifecycle control. | primitive | `rpc` | driver | quadruped |
| `robonix/lifecycle/driver` | Shared lifecycle. | system | `rpc` | driver | lifecycle |

## primitive
"""

        normalized = NORMALIZE_REFERENCE.group_legacy_driver_contracts(text)
        warning, primary = normalized.split("## primitive", maxsplit=1)
        compatibility = warning.split(
            ":::warning[后向兼容：已有命名空间 Driver]", maxsplit=1
        )[1]

        self.assertIn("robonix/primitive/hand/driver", warning)
        self.assertIn("robonix/primitive/quadruped/driver", warning)
        self.assertNotIn(
            "| `robonix/lifecycle/driver` | Shared lifecycle.", compatibility
        )
        self.assertIn("robonix/lifecycle/driver", normalized)
        self.assertNotIn("robonix/primitive/hand/driver", primary)


if __name__ == "__main__":
    unittest.main()
