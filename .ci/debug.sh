#!/bin/sh

set -e
set -x

rm -rf build-debug
mkdir build-debug
cd build-debug
cmake -DCMAKE_BUILD_TYPE=Debug -DSTRICT=ON -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=OFF \
      -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
cmake --build .
ctest -V
