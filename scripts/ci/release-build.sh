#!/bin/sh

set -e

rm -rf build-release
mkdir build-release
cd build-release
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=/usr -DSTRICT=ON \
  -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=ON -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=ON ..
cmake --build .
ctest -V
cpack
