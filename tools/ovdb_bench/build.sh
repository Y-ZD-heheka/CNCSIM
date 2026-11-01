#!/bin/sh
# 构建 OpenVDB 对比基准（2026-10-31 实测可用的完整命令）。
#
# 依赖（全部源码编译，clang++ -O3 -march=native）：
#   · OpenVDB 14.0.0      https://github.com/AcademySoftwareFoundation/openvdb
#   · oneTBB  2023.2.0    https://github.com/uxlfoundation/oneTBB
#   · zlib 1.3.2 / zstd 1.5.7（vcpkg 装到 C:/dev/vcpkg/installed/x64-windows-release）
#
# OpenVDB 14 已不再依赖 Boost / Blosc / Imath / OpenEXR（USE_EXR=OFF 时）。
#
# 一次性准备（只需跑一次）：
#
#   cmake -B /c/dev/build-tbb23 -S /c/dev/oneTBB-2023.2.0 \
#         -G "Visual Studio 18 2026" -A x64 \
#         -DCMAKE_INSTALL_PREFIX=C:/dev/tbb-install23 -DTBB_TEST=OFF -DTBB_EXAMPLES=OFF
#   cmake --build /c/dev/build-tbb23 --config Release --target install
#
#   # 注意 -O3（不是 /O3）——git bash 会把 /O3 当成路径改写掉
#   MSYS2_ARG_CONV_EXCL="*" cmake -B /c/dev/build-ovdb14 -S /c/dev/openvdb-14.0.0 \
#         -G "Visual Studio 18 2026" -A x64 \
#         -DCMAKE_CXX_COMPILER="C:/Program Files/LLVM/bin/clang++.exe" \
#         -DCMAKE_PREFIX_PATH="C:/dev/tbb-install23;C:/dev/vcpkg/installed/x64-windows-release" \
#         -DTBB_ROOT=C:/dev/tbb-install23 \
#         -DUSE_EXR=OFF -DUSE_IMATH_HALF=OFF -DUSE_AX=OFF -DUSE_NANOVDB=OFF \
#         -DOPENVDB_BUILD_BINARIES=OFF -DOPENVDB_BUILD_UNITTESTS=OFF \
#         -DOPENVDB_BUILD_PYTHON_MODULE=OFF \
#         -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -DNDEBUG"
#   cmake --build /c/dev/build-ovdb14 --config Release
#
set -e

CLANG="C:/Program Files/LLVM/bin/clang++.exe"
OVDB_SRC=/c/dev/openvdb-14.0.0
OVDB_BUILD=/c/dev/build-ovdb14/openvdb/openvdb
TBB=/c/dev/tbb-install23
VCPKG=/c/dev/vcpkg/installed/x64-windows-release

"$CLANG" -O3 -march=native -std=c++17 -DNDEBUG \
    -I "$OVDB_SRC/openvdb" \
    -I "$OVDB_SRC/ext/imath" \
    -I "$OVDB_BUILD" \
    -I "$OVDB_BUILD/openvdb" \
    -I "$TBB/include" \
    -I "$VCPKG/include" \
    ovdb_bench.cpp -o ovdb_bench.exe \
    "$OVDB_BUILD/Release/openvdb.lib" \
    "$TBB/lib/tbb12.lib" \
    "$VCPKG/lib/z.lib" \
    "$VCPKG/lib/zstd.lib"

# 运行前把 openvdb.dll / tbb12.dll / z.dll / zstd.dll 放到同目录
echo "built: ovdb_bench.exe"
