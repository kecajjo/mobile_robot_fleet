#!/bin/bash
function flake8_test() {
    flake8_ouput=$(flake8 "${sources_py[@]}")
    if [ -n "$flake8_ouput" ]; then
        echo -e "$flake8_ouput"
        err_flk=$(echo -e "$flake8_ouput" | wc -l)
    fi
}

function clang_format() {
    local sources=("${sources_c[@]}")
    for i in "${sources[@]}"; do
        if ! diff <(clang-format "$i") "$i" >/dev/null; then
            echo "formatting errors in: \"$i\"" && err=$((err+1))
            ((err_fmt++))
        fi
    done
}

function analyze() {
    err_fmt=0
    err_flk=0

    readarray -t sources_c -d '' < <(git grep --cached -l '' | grep ".*\.\(c$\|h$\|cpp$\|hpp$\)")
    readarray -t sources_py -d '' < <(git grep --cached -l '' | grep ".*\.\(py$\)")

    clang_format
    flake8_test

    echo "======= \"$repo_root\" analysis:"
    echo "found $err_fmt file(s) with formatting errors "
    echo "found $err_flk flake8 format violations"

}

dir="$(dirname "$(readlink -f "$0")")"
repo_root=$(readlink -f "$dir/..")

declare -a sources_py
declare -a sources_c

cd "$repo_root" || exit 1
analyze
exit $((err_fmt + err_flk))