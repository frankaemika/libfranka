#!/bin/sh

set -e
set -x

rm -rf build-coverage
mkdir build-coverage
cd build-coverage
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_COVERAGE=ON -DBUILD_DOCUMENTATION=OFF \
      -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=ON ..
cmake --build .
cmake --build . --target coverage
