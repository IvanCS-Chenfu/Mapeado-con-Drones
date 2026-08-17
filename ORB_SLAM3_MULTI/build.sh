#!/bin/bash

set -e

echo "===================================="
echo "Building ORB_SLAM3_MULTI Thirdparty g2o"
echo "===================================="

cd "$(dirname "$0")"

# Limpiar cache copiado desde ORB_SLAM3 original.
# Esto evita errores del tipo:
# CMakeCache.txt was created in another directory.
rm -rf Thirdparty/g2o/build
rm -rf Thirdparty/g2o/lib

cd Thirdparty/g2o

mkdir -p build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo "Creating libg2o_multi.so"

cd ../lib

cp libg2o.so libg2o_multi.so

if command -v patchelf >/dev/null 2>&1; then
    patchelf --set-soname libg2o_multi.so libg2o_multi.so
else
    echo "ERROR: patchelf not found. Install it with: sudo apt install patchelf"
    exit 1
fi

cd ../build

cd ../../..

echo "===================================="
echo "Building ORB_SLAM3_MULTI"
echo "===================================="

rm -rf build
mkdir -p build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo "===================================="
echo "ORB_SLAM3_MULTI build finished"
echo "===================================="