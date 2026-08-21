# Copyright 2026 Assessment Student
# SPDX-License-Identifier: Apache-2.0

"""Validate local links in repository Markdown documents."""

from pathlib import Path
import re
from urllib.parse import unquote


PACKAGE_ROOT = Path(__file__).parents[1]
LINK_PATTERN = re.compile(r'\[[^\]]*\]\(([^)]+)\)')


def test_relative_markdown_links_resolve():
    """Ensure every repository-relative Markdown link has a target."""
    documents = [
        path
        for path in PACKAGE_ROOT.rglob('*.md')
        if '.git' not in path.parts
    ]

    missing = []
    for document in documents:
        content = document.read_text(encoding='utf-8')
        for target in LINK_PATTERN.findall(content):
            path_text = unquote(target.split('#', 1)[0]).strip()
            if not path_text or '://' in path_text:
                continue
            resolved = (document.parent / path_text).resolve()
            if not resolved.exists():
                missing.append(
                    f'{document.relative_to(PACKAGE_ROOT)} -> {target}'
                )

    assert not missing, 'Broken Markdown links:\n' + '\n'.join(missing)
