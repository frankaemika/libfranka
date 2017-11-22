#!/bin/sh

set -e
set -x

rm -rf build-lint
mkdir build-lint
cd build-lint
cmake -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=OFF ..
cmake --build . --target check-format
cmake --build . --target check-tidy
