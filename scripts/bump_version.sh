#!/bin/bash
# Set the repo-wide version in every ROS package.xml and Python setup.py under ros2/src.
# Submodules (fs_msgs, eufs_msgs) and anything under lib/ are left alone.
# Usage: scripts/bump_version.sh X.Y.Z
# Does not commit or tag; review the diff, then commit and tag by hand.
set -euo pipefail

if [ $# -ne 1 ] || ! [[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "usage: $0 X.Y.Z" >&2
    exit 2
fi

ROOT="${ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

python3 - "$ROOT" "$1" <<'PYEOF'
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1]) / "ros2" / "src"
version = sys.argv[2]
skip_dirs = {"fs_msgs", "eufs_msgs", "lib", "third_party", "build", "install"}

xml_re = re.compile(r"<version>[^<]*</version>")
py_re = re.compile(r"""\bversion(\s*=\s*)(['"])[^'"]*\2""")

for path in sorted(root.rglob("*")):
    if path.name not in ("package.xml", "setup.py") or not path.is_file():
        continue
    rel = path.relative_to(root).parts
    if skip_dirs & set(rel):
        continue
    text = path.read_bytes().decode("utf-8")
    if path.name == "package.xml":
        new, n = xml_re.subn(f"<version>{version}</version>", text, count=1)
    else:
        new, n = py_re.subn(lambda m: f"version{m.group(1)}{m.group(2)}{version}{m.group(2)}", text, count=1)
    if n == 0:
        print(f"no version field found: {path.relative_to(root.parent.parent)}")
        continue
    if new != text:
        path.write_bytes(new.encode("utf-8"))
        print(f"set {version}: {path.relative_to(root.parent.parent)}")
PYEOF
