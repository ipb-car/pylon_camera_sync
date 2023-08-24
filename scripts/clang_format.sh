#!/usr/bin/env bash

set -e
sources=$(find . -regextype posix-extended -regex \
      ".*\.(cpp|cxx|cc|hpp|hxx|h)" |
        grep -vE "^./(build)/")

echo "Running clang-format on all project sources"
clang-format -Werror --dry-run --ferror-limit=1 -style=file ${sources}
