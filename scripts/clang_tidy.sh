#!/usr/bin/env bash
# @file      clang_tidy.sh
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2020 Ignacio Vizzo, all rights reserved
set -e

build_path="$1"
echo "Reading compile_commands.json from $build_path"

echo "Running clang-tidy on all project sources"
run-clang-tidy -quiet -p $build_path -j$(nproc --all)
