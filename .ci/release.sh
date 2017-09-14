#!/bin/sh

set -e
set -x

rm -rf build-release
mkdir build-release
cd build-release
cmake -DCMAKE_BUILD_TYPE=Release -DSTRICT=ON -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=ON \
      -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
cmake --build .
ctest -V
cpack

cd ..
rm -rf build-examples
mkdir build-examples
cd build-examples
cmake -DFranka_DIR:PATH=../build-release ../examples
cmake --build .
