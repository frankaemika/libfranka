#!/bin/sh

set -e
set -x

rm -rf build-libonly
mkdir build-libonly
cd build-libonly
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=OFF \
      -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF ..
cmake --build .
