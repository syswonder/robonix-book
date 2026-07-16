#!/usr/bin/env python3
"""Verify that every page from the archived mdBook still has a static URL."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


LINK = re.compile(r'\[[^]]+\]\((?P<path>[^)#]+\.md)\)')


def expected_output(markdown_path: str, build: Path) -> Path:
    source = Path(markdown_path)
    if source.name == 'README.md':
        return build / 'index.html'
    return build / source.with_suffix('.html')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('build', type=Path)
    parser.add_argument(
        '--summary',
        type=Path,
        default=Path('archive/mdbook/src/SUMMARY.md'),
    )
    args = parser.parse_args()

    links = sorted(set(LINK.findall(args.summary.read_text(encoding='utf-8'))))
    missing = [
        (path, expected_output(path, args.build))
        for path in links
        if not expected_output(path, args.build).is_file()
    ]
    if missing:
        for source, output in missing:
            print(f'missing legacy URL for {source}: {output}')
        raise SystemExit(1)
    print(f'legacy URL check: {len(links)} pages available')


if __name__ == '__main__':
    main()
