#!/usr/bin/env python3
"""Check every internal link and fragment in the generated Docusaurus site."""

from __future__ import annotations

import argparse
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urljoin, urlsplit


IGNORED_SCHEMES = {"data", "javascript", "mailto", "tel"}


class PageParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.hrefs: list[str] = []
        self.anchors: set[str] = set()

    def handle_starttag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        values = dict(attrs)
        anchor = values.get("id")
        if anchor:
            self.anchors.add(anchor)
        if tag == "a" and values.get("href"):
            self.hrefs.append(values["href"] or "")


def parse_page(path: Path) -> PageParser:
    parser = PageParser()
    parser.feed(path.read_text(encoding="utf-8"))
    return parser


def page_route(page: Path, build: Path) -> str:
    relative = page.relative_to(build)
    if relative.name == "index.html":
        parent = relative.parent.as_posix()
        return "/" if parent == "." else f"/{parent}/"
    return f"/{relative.with_suffix('').as_posix()}"


def output_for_path(path: str, build: Path) -> Path | None:
    relative = unquote(path).lstrip("/")
    if not relative:
        return build / "index.html"

    direct = build / relative
    candidates = [direct]
    if path.endswith("/"):
        candidates.insert(0, direct / "index.html")
    elif not direct.suffix:
        candidates.extend([direct.with_suffix(".html"), direct / "index.html"])

    return next((candidate for candidate in candidates if candidate.is_file()), None)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("build", type=Path)
    args = parser.parse_args()

    pages = sorted(args.build.rglob("*.html"))
    parsed = {page: parse_page(page) for page in pages}
    failures: list[str] = []
    checked = 0

    for source, document in parsed.items():
        source_route = page_route(source, args.build)
        for href in document.hrefs:
            target = urlsplit(href)
            if target.scheme in IGNORED_SCHEMES or target.netloc:
                continue

            resolved = urlsplit(urljoin(source_route, href))
            output = output_for_path(resolved.path, args.build)
            checked += 1
            if output is None:
                failures.append(f"{source_route}: missing target {href}")
                continue

            if resolved.fragment and output.suffix == ".html":
                target_page = parsed.get(output)
                if target_page is None:
                    target_page = parse_page(output)
                    parsed[output] = target_page
                fragment = unquote(resolved.fragment)
                if fragment not in target_page.anchors:
                    failures.append(
                        f"{source_route}: missing fragment #{fragment} in {href}"
                    )

    if failures:
        for failure in failures:
            print(failure)
        raise SystemExit(1)
    print(f"built link check: {checked} internal links and fragments available")


if __name__ == "__main__":
    main()
