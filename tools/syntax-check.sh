#!/usr/bin/env bash
# Type-checks the project on a host compiler against stubbed V5 SDK headers.
#
# The real SDK ships with VEXcode and can't be redistributed, so tools/stubs/
# declares the API surface the project uses. This catches redefinitions, missing
# declarations, wrong types and syntax errors — the class of bug that made the
# previous version of this repo fail to build. It does not check runtime
# behaviour; build in VEXcode for that.
set -e
cd "$(dirname "$0")/.."
g++ -std=c++17 -fsyntax-only -Wall -Wextra -Wno-unused-parameter \
    -Iinclude -Itools/stubs \
    src/*.cpp
echo "OK: all translation units type-check"
