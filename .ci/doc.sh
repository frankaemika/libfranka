#!/bin/sh

set -e

rm -rf build-doc
mkdir build-doc
cd build-doc
cmake -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=ON -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF ..
cmake --build . --target doc
