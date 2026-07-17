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
