#!/usr/bin/env python3
"""Validate local links and fragments in an already-built mdBook."""

from __future__ import annotations

import argparse
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit


IGNORED_SCHEMES = {"data", "http", "https", "mailto", "tel", "javascript"}


class PageParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.ids: set[str] = set()
        self.links: list[str] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        values = dict(attrs)
        if values.get("id"):
            self.ids.add(values["id"] or "")
        if tag == "a" and values.get("name"):
            self.ids.add(values["name"] or "")
        if tag == "a" and values.get("href"):
            self.links.append(values["href"] or "")


def parse_page(path: Path) -> PageParser:
    parser = PageParser()
    parser.feed(path.read_text(encoding="utf-8"))
    return parser


def target_file(root: Path, source: Path, raw_path: str) -> Path:
    decoded = unquote(raw_path)
    if decoded.startswith("/"):
        candidate = root / decoded.lstrip("/")
    elif decoded:
        candidate = source.parent / decoded
    else:
        candidate = source

    if decoded.endswith("/") or candidate.is_dir():
        candidate = candidate / "index.html"
    return candidate.resolve()


def main() -> int:
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("book", type=Path, help="built mdBook directory")
    argument_parser.add_argument(
        "--allow-missing-prefix",
        action="append",
        default=[],
        help="book-relative generated subtree that may be absent (repeatable)",
    )
    args = argument_parser.parse_args()

    root = args.book.resolve()
    allowed_missing = tuple(prefix.strip("/") for prefix in args.allow_missing_prefix)
    pages = sorted(
        page for page in root.rglob("*.html") if "api" not in page.relative_to(root).parts[:1]
    )
    if not pages:
        raise SystemExit(f"no HTML pages found under {root}")

    parsed = {page.resolve(): parse_page(page) for page in pages}
    errors: list[str] = []

    for source, page in parsed.items():
        for href in page.links:
            url = urlsplit(href)
            if url.scheme.lower() in IGNORED_SCHEMES or url.netloc:
                continue
            destination = target_file(root, source, url.path)
            try:
                destination.relative_to(root)
            except ValueError:
                errors.append(f"{source.relative_to(root)}: link escapes book root: {href}")
                continue
            if not destination.exists():
                relative = destination.relative_to(root).as_posix()
                if any(
                    relative == prefix or relative.startswith(prefix + "/")
                    for prefix in allowed_missing
                ):
                    continue
                errors.append(f"{source.relative_to(root)}: missing target: {href}")
                continue
            if url.fragment and destination.suffix.lower() == ".html":
                target_page = parsed.get(destination)
                if target_page is None:
                    target_page = parse_page(destination)
                    parsed[destination] = target_page
                fragment = unquote(url.fragment)
                if fragment not in target_page.ids:
                    errors.append(
                        f"{source.relative_to(root)}: missing fragment "
                        f"{fragment!r} in {destination.relative_to(root)}"
                    )

    if errors:
        print("local link validation failed:")
        for error in errors:
            print(f"  - {error}")
        return 1

    print(f"local link validation passed: {len(pages)} HTML pages")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
