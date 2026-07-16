#!/usr/bin/env bash
# Write the source revision shown in the global documentation status bar.
set -euo pipefail

if [[ $# -ne 2 ]]; then
    echo "usage: $0 <robonix-source-dir> <branch>" >&2
    exit 2
fi

source_dir=$1
branch=$2

git -C "$source_dir" rev-parse --is-inside-work-tree >/dev/null
commit=$(git -C "$source_dir" rev-parse --short=12 HEAD)
source_date=$(git -C "$source_dir" show -s --format=%cs HEAD)

python3 - "$branch" "$commit" "$source_date" > theme/build-info.js <<'PY'
import json
import sys

branch, commit, source_date = sys.argv[1:]
build = {
    "branch": branch,
    "commit": commit,
    "sourceDate": source_date,
}

print("window.ROBONIX_DOC_BUILD = " + json.dumps(build, ensure_ascii=False) + ";")
print(
    '''
document.addEventListener("DOMContentLoaded", () => {
    const target = document.getElementById("robonix-doc-revision");
    if (!target) return;
    const info = window.ROBONIX_DOC_BUILD;
    target.textContent = "源码 " + info.commit + " · " + info.sourceDate;
});'''.lstrip()
)
PY
