#!/usr/bin/env python3
# Copyright 2026 Ângelo Moisés Alves
# Licensed under the MIT license, see the LICENSE file for details.
"""Check the links between the Markdown files of the repository.

For every [text](target) in a tracked .md file whose target is not a URL, the file
it points to has to exist, and so does the heading it points to with #anchor (the
anchors are made the way GitHub makes them). Links to the web are not followed:
a link checker that needs the network fails for reasons that are not the fault of
the commit.

    python3 extras/tools/check-links.py

Exits with 1 and says what is wrong if a link is broken.
"""
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")
HEADING = re.compile(r"^(#{1,6})\s+(.*?)\s*#*\s*$", re.M)
FENCE = re.compile(r"^(```|~~~).*?^\1", re.M | re.S)


def slug(heading):
    """The anchor GitHub makes from a heading: lower case, no punctuation, hyphens for spaces."""
    heading = re.sub(r"[`*_]", "", heading.strip().lower())
    heading = re.sub(r"[^\w\- ]", "", heading)
    return heading.replace(" ", "-")


def anchors(text):
    """All the anchors of a document; a heading that repeats gets -1, -2... as on GitHub."""
    seen, found = {}, set()
    for m in HEADING.finditer(FENCE.sub("", text)):
        base = slug(m.group(2))
        n = seen.get(base, 0)
        seen[base] = n + 1
        found.add(base if n == 0 else "%s-%d" % (base, n))
    return found


def main():
    files = subprocess.run(["git", "ls-files", "*.md"], cwd=REPO, capture_output=True, text=True,
                           check=True).stdout.split()
    problems = []
    cache = {}

    def anchors_of(path):
        if path not in cache:
            cache[path] = anchors(path.read_text(encoding="utf-8"))
        return cache[path]

    for name in files:
        source = REPO / name
        text = FENCE.sub("", source.read_text(encoding="utf-8"))
        for m in LINK.finditer(text):
            target = m.group(1)
            if re.match(r"[a-z][a-z0-9+.-]*:", target):  # http:, https:, mailto: ...
                continue
            path, _, fragment = target.partition("#")
            dest = source if not path else (source.parent / path).resolve()
            if not dest.exists():
                problems.append("%s: %s does not exist" % (name, target))
            elif fragment and dest.suffix == ".md" and fragment not in anchors_of(dest):
                problems.append("%s: %s has no heading '%s'" % (name, dest.relative_to(REPO), fragment))
    for line in problems:
        print(line)
    print("%d Markdown files, %d broken links" % (len(files), len(problems)))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
