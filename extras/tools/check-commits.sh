#!/bin/bash
# Copyright 2026 Ângelo Moisés Alves
# Licensed under the MIT license, see the LICENSE file for details.
#
# Check every commit of a branch, not only the last one, which is all that the CI sees of a push:
# the formatting, the links between the Markdown files, and the build of four sketches. No board
# is needed. See --help.
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: check-commits.sh BASE BRANCH

Check every commit in BASE..BRANCH, oldest first, each in a temporary worktree of this repository,
and print a line for each:

  <short sha> <subject> : fmt:ok links:ok ok ok ok ok

  fmt     the formatting check of the CI: extras/tools/check-format.sh of the checkout this script is
          in, run on the tree of the commit
  links   the check of the links between the Markdown files: extras/tools/check-links.py of the
          commit (links:skip if the commit has none)
  ok      the build for rp2040:rp2040:rpipico, against the tree of the commit, of examples/BasicTest,
          examples/WebServerLedToggle, extras/bench/BenchTest and extras/bench/BenchLwip, in this
          order; FAIL(<sketch>) if one does not build

The end of the output of a check that fails follows the line, on the error output. Use it before
pushing a branch whose history you rewrote. About a minute and a half a commit.

Needs git, python3, arduino-cli with the rp2040 core and clang-format 18 (CLANG_FORMAT names another
one). The scratch files go to ${TMPDIR:-/tmp}; the worktree is removed when the script ends, however
it ends. The repository is the one this script is in.

Exit status: 0 if every commit passed, 1 if one failed, 2 if the check could not be made.
EOF
}

die() {
    echo "check-commits: $*" >&2
    exit 2
}

case ${1:-} in
    -h | --help)
        usage
        exit 0
        ;;
esac
if [[ $# -ne 2 ]]; then
    usage >&2
    exit 2
fi

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo=$(git -C "$here" rev-parse --show-toplevel 2> /dev/null) || die "$here is not in a git repository"
fqbn=rp2040:rp2040:rpipico
for tool in git python3 arduino-cli "${CLANG_FORMAT:-clang-format-18}"; do
    command -v "$tool" > /dev/null 2>&1 || die "$tool not found"
done

# Resolve the refs here, in the repository: HEAD~1 would mean something else inside the worktree
base=$(git -C "$repo" rev-parse --verify --quiet "$1^{commit}") || die "$1 is not a commit"
branch=$(git -C "$repo" rev-parse --verify --quiet "$2^{commit}") || die "$2 is not a commit"
commits=()
while IFS= read -r commit; do
    commits+=("$commit")
done < <(git -C "$repo" rev-list --reverse "$base..$branch")
if [[ ${#commits[@]} -eq 0 ]]; then
    echo "check-commits: no commits in $1..$2" >&2
    exit 0
fi

work=$(mktemp -d "${TMPDIR:-/tmp}/picoenc-check-commits.XXXXXX")
tree=$work/tree
log=$work/log.txt
cleanup() {
    git -C "$repo" worktree remove --force "$tree" > /dev/null 2>&1 || true
    git -C "$repo" worktree prune
    rm -rf "${work:?}"
}
# The EXIT trap runs on every way out of the script, and these two turn the signals into one
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
git -C "$repo" worktree add -q --detach "$tree" "$base"

step() { # step LABEL COMMAND...: run it with its output added to the log of the commit
    local label=$1
    shift
    printf '== %s\n' "$label" >> "$log"
    "$@" >> "$log" 2>&1 < /dev/null
}

failed=0
for commit in "${commits[@]}"; do
    git -C "$tree" checkout -q "$commit"
    : > "$log"
    bad=0
    # The check of the checkout this script is in, on the tree of the commit: it cannot drift from the CI
    if step format bash "$here/check-format.sh" "$tree"; then res="fmt:ok"; else res="fmt:FAIL"; bad=1; fi
    if [[ -f $tree/extras/tools/check-links.py ]]; then
        if step links python3 "$tree/extras/tools/check-links.py"; then res+=" links:ok"; else res+=" links:FAIL"; bad=1; fi
    else
        res+=" links:skip"
    fi
    for sketch in examples/BasicTest examples/WebServerLedToggle extras/bench/BenchTest extras/bench/BenchLwip; do
        if step "$sketch" arduino-cli compile --fqbn "$fqbn" --library "$tree" --build-path "$work/build" "$tree/$sketch"; then
            res+=" ok"
        else
            res+=" FAIL($sketch)"
            bad=1
        fi
    done
    printf '%-60.60s : %s\n' "$(git -C "$repo" log -1 --format='%h %s' "$commit")" "$res"
    if [[ $bad -ne 0 ]]; then
        failed=$((failed + 1))
        tail -n 30 "$log" | sed 's/^/    /' >&2
    fi
done
echo "check-commits: ${#commits[@]} commits, $failed failed" >&2
[[ $failed -eq 0 ]]
