#!/bin/bash
# Post-session hook: block Claude from finishing if source files are newer than CLAUDE.md.
# Once Claude updates CLAUDE.md its mtime becomes newer → no more blocking.

REPO="$HOME/Develop/MFE-Driverless-V1"
CLAUDE_MD="$REPO/CLAUDE.md"

LATEST_SRC=0
while IFS= read -r f; do
    mt=$(stat -c %Y "$REPO/$f" 2>/dev/null) || continue
    [ "$mt" -gt "$LATEST_SRC" ] && LATEST_SRC="$mt"
done < <(git -C "$REPO" diff --name-only 2>/dev/null | grep -E '\.(py|launch\.py|sh|cpp|hpp)$')

[ "$LATEST_SRC" -eq 0 ] && exit 0

CLAUDE_MTIME=$(stat -c %Y "$CLAUDE_MD" 2>/dev/null || echo 0)
[ "$CLAUDE_MTIME" -ge "$LATEST_SRC" ] && exit 0

CHANGED=$(git -C "$REPO" diff --name-only 2>/dev/null | grep -cE '\.(py|launch\.py|sh|cpp|hpp)$')
printf '{"decision":"block","reason":"CLAUDE.md is outdated — %d source files changed more recently. Update the Current status and Key implementation notes sections in CLAUDE.md, then you can finish."}\n' "$CHANGED"
exit 0
