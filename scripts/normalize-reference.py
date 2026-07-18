#!/usr/bin/env python3
"""Normalize rbnx-generated Markdown and preserve declared legacy URL anchors."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


ANCHOR_HEADING = re.compile(
    r'^<a id="(?P<anchor>[^"]+)"></a>\n(?P<heading>#{2,6} .+)$',
    re.MULTILINE,
)
GENERATED_BANNER = re.compile(
    r'^> 由 robonix .+?自动生成，请勿手改。重新生成：`rbnx docs`。$',
    re.MULTILINE,
)
IDL_INDEX_BLOCK = re.compile(
    r'\n+<details className="idl-package-index">.*?</details>\n+',
    re.DOTALL,
)
RETIRED_IDL_FRAGMENT_COMPATIBILITY = {
    'soma-srv-getsensorextrinsics-srv': 'soma/srv/GetSensorExtrinsics.srv',
}
LEGACY_DRIVER_ROW = re.compile(
    r'^\| `robonix/(?:primitive|service)/[^`]+/driver` '
    r'\| Legacy-compatible lifecycle[^\n]+$',
    re.MULTILINE,
)


def idl_packages(text: str) -> list[str]:
    packages: list[str] = []
    in_fence = False
    for line in text.splitlines():
        if line.startswith('```'):
            in_fence = not in_fence
            continue
        if not in_fence and re.fullmatch(r'## [A-Za-z0-9_]+', line):
            packages.append(line.removeprefix('## '))
    return packages


def append_retired_idl_fragment_anchors(text: str) -> str:
    """Keep archived fragment URLs valid without advertising retired APIs."""
    missing = [
        (anchor, label)
        for anchor, label in RETIRED_IDL_FRAGMENT_COMPATIBILITY.items()
        if f'id="{anchor}"' not in text
    ]
    if not missing:
        return text

    lines = [
        '',
        '## 已移除接口的 URL 兼容锚点',
        '',
        '> 下列锚点只用于保持旧版文档 URL 可跳转，不表示接口仍存在或可在新部署中使用。',
        '',
    ]
    for anchor, label in missing:
        lines.extend(
            [
                f'<span id="{anchor}"></span>',
                f'- `{label}` 已从当前固定修订的 IDL 目录移除。',
            ]
        )
    return text.rstrip() + '\n' + '\n'.join(lines) + '\n'


def group_legacy_driver_contracts(text: str) -> str:
    """Keep legacy Driver contracts out of the primary interface tables."""
    if '# 能力约定参考（自动生成）' not in text[:160]:
        return text
    if ':::warning[后向兼容：已有命名空间 Driver]' in text:
        return text

    rows = LEGACY_DRIVER_ROW.findall(text)
    if not rows:
        return text

    text = LEGACY_DRIVER_ROW.sub('', text)
    text = re.sub(r'\n{3,}', '\n\n', text)
    text = text.replace('|---|---|---|---|---|---|\n\n|', '|---|---|---|---|---|---|\n|')
    marker = '\n## primitive\n'
    if marker not in text:
        return text

    compatibility = (
        '\n:::warning[后向兼容：已有命名空间 Driver]\n'
        '下表只记录仍由旧软件包自行维护的命名空间 Driver 和 Driver TOML。'
        '这些能力约定目前仍可使用，但计划迁移到共享 '
        '`robonix/lifecycle/driver`；同一提供方不能同时注册两种 Driver。'
        '维护和迁移步骤见[软件包与部署清单规范]'
        '(../integration-guide/packaging-spec.md#42-已有命名空间-driver-的兼容流程)。\n\n'
        '| 能力约定 ID | 接口含义 | kind | mode | 载荷（IDL） | 能力约定 TOML |\n'
        '|---|---|---|---|---|---|\n'
        + '\n'.join(rows)
        + '\n:::\n'
    )
    return text.replace(marker, compatibility + marker, 1)


def compact_markdown_tables(text: str) -> str:
    """Remove blank lines accidentally left between rows of one Markdown table."""
    return re.sub(r'(?m)(^\|[^\n]*\|\n)\n(?=\|)', r'\1', text)


def normalize(text: str) -> str:
    text = '\n'.join(line.rstrip() for line in text.splitlines()) + '\n'
    text = re.sub(r'^\[toc\]\n+', '', text, flags=re.MULTILINE)
    text = re.sub(
        r'\(\{([A-Za-z0-9_, ]+)\}\)',
        lambda match: '('
        + ', '.join(f'`{name.strip()}`' for name in match.group(1).split(','))
        + ')',
        text,
    )
    text = GENERATED_BANNER.sub(
        '> 由 `rbnx docs` 自动生成，请勿手改。',
        text,
    )
    text = ANCHOR_HEADING.sub(
        lambda match: f'{match.group("heading")} {{/* #{match.group("anchor")} */}}',
        text,
    )
    text = group_legacy_driver_contracts(text)
    text = compact_markdown_tables(text)
    if '# ROS IDL 参考（自动生成）' in text[:120]:
        if not text.startswith('---\n'):
            text = '---\nhide_table_of_contents: true\n---\n\n' + text
        text = IDL_INDEX_BLOCK.sub('\n\n', text)
        packages = idl_packages(text)
        links = ' · '.join(f'[{name}](#{name.lower()})' for name in packages)
        first_package = text.find('\n## ')
        index = (
            '\n<details className="idl-package-index">\n'
            '<summary>展开 ROS package 索引</summary>\n\n'
            f'{links}\n\n</details>\n'
        )
        text = text[:first_package] + index + text[first_package:]
        text = append_retired_idl_fragment_anchors(text)
    return text


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('paths', nargs='+', type=Path)
    args = parser.parse_args()

    for path in args.paths:
        original = path.read_text(encoding='utf-8')
        updated = normalize(original)
        path.write_text(updated, encoding='utf-8')


if __name__ == '__main__':
    main()
