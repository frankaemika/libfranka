#!/bin/sh

set -e

rm -rf build-examples
mkdir build-examples
cd build-examples
cmake -DFranka_DIR:PATH=../build-release ../examples
cmake --build .
