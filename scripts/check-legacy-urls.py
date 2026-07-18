#!/usr/bin/env python3
"""Verify that every page from the archived mdBook still has a static URL."""

from __future__ import annotations

import argparse
from html.parser import HTMLParser
import re
from pathlib import Path
from urllib.parse import unquote, urlsplit


PAGE_LINK = re.compile(r'\[[^]]+\]\((?P<path>[^)#]+\.md)\)')
MARKDOWN_LINK = re.compile(
    r'(?<!!)\[[^]]*\]\((?P<target><[^>]+>|[^\s)]+)'
    r'(?:\s+["\'][^)]*["\'])?\)'
)


class IdCollector(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.ids: set[str] = set()

    def handle_starttag(self, _tag: str, attrs: list[tuple[str, str | None]]) -> None:
        for name, value in attrs:
            if name == 'id' and value:
                self.ids.add(value)


def expected_output(markdown_path: str, build: Path) -> Path:
    source = Path(markdown_path)
    if source.name == 'README.md':
        return build / 'index.html'
    return build / source.with_suffix('.html')


def archived_fragment_links(source_root: Path) -> list[tuple[Path, str, str]]:
    links: list[tuple[Path, str, str]] = []
    for source in source_root.rglob('*.md'):
        text = source.read_text(encoding='utf-8')
        for match in MARKDOWN_LINK.finditer(text):
            target = match.group('target').strip('<>')
            parsed = urlsplit(target)
            if parsed.scheme or parsed.netloc or not parsed.fragment:
                continue
            path = unquote(parsed.path)
            if not path.endswith('.md'):
                continue
            resolved = (source.parent / path).resolve()
            try:
                relative = resolved.relative_to(source_root.resolve())
            except ValueError:
                continue
            links.append((source, relative.as_posix(), unquote(parsed.fragment)))
    return links


def html_ids(path: Path) -> set[str]:
    parser = IdCollector()
    parser.feed(path.read_text(encoding='utf-8'))
    return parser.ids


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('build', type=Path)
    parser.add_argument(
        '--summary',
        type=Path,
        default=Path('archive/mdbook/src/SUMMARY.md'),
    )
    args = parser.parse_args()

    links = sorted(set(PAGE_LINK.findall(args.summary.read_text(encoding='utf-8'))))
    missing = [
        (path, expected_output(path, args.build))
        for path in links
        if not expected_output(path, args.build).is_file()
    ]
    if missing:
        for source, output in missing:
            print(f'missing legacy URL for {source}: {output}')
        raise SystemExit(1)

    source_root = args.summary.parent
    fragment_links = archived_fragment_links(source_root)
    ids_by_output: dict[Path, set[str]] = {}
    missing_fragments: list[tuple[Path, str, str, Path]] = []
    for source, target, fragment in fragment_links:
        output = expected_output(target, args.build)
        ids = ids_by_output.setdefault(output, html_ids(output) if output.is_file() else set())
        if fragment not in ids:
            missing_fragments.append((source, target, fragment, output))
    if missing_fragments:
        for source, target, fragment, output in missing_fragments:
            print(
                'missing legacy fragment '
                f'{source.relative_to(source_root)} -> {target}#{fragment}: {output}'
            )
        raise SystemExit(1)

    print(
        f'legacy URL check: {len(links)} pages and '
        f'{len(fragment_links)} archived fragments available'
    )


if __name__ == '__main__':
    main()
