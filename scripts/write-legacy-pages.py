#!/usr/bin/env python3
"""Publish legacy mdBook index URLs without redirecting canonical routes."""

from __future__ import annotations

import shutil
import sys
from pathlib import Path


LEGACY_INDEXES = (
    "integration-guide",
    "interface-catalog",
    "interface-catalog/primitive",
    "interface-catalog/service",
    "interface-catalog/system",
    "reference",
)


def main() -> int:
    build_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "build")
    for route in LEGACY_INDEXES:
        source = build_dir / f"{route}.html"
        target = build_dir / route / "index.html"
        if not source.is_file():
            raise FileNotFoundError(f"canonical page was not built: {source}")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
    print(f"legacy index pages: {len(LEGACY_INDEXES)} copied from canonical pages")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
