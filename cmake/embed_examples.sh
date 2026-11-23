#!/usr/bin/env bash
# Runs the example configuration generator (cmake/embed_text.cmake) on its own, without configuring a build.
# The arguments must mirror the embed_text_function() calls of CMakeLists.txt (keep them in sync).
#
# Usage: cmake/embed_examples.sh [output dir]    (default: ./generated)
set -eu

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
OUT=${1:-generated}

mkdir -p "$OUT"

embed() { # <yaml file> <function> <header>
  cmake -DINPUT="$ROOT/config/$1" -DOUTPUT="$OUT/$2.cpp" -DFUNCTION="$2" -DHEADER="$3" \
    -P "$ROOT/cmake/embed_text.cmake"
  echo "$OUT/$2.cpp"
}

embed pendulum_core_config.yaml get_pendulum_config_example pendulum_core/pendulum_config.hpp
embed demo.yaml get_demo_config_example demo_config.hpp
