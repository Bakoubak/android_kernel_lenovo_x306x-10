#!/bin/bash
#
# Copyright (c) 2023 - JMPFBMX
#

export LC_ALL=C
export ARCH=arm64

# Get the absolute paths for CLANG_PATH and GCC_PATH
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_OUT="${SCRIPT_DIR}/../KERNEL_OUT"
CLANG_PATH="${SCRIPT_DIR}/../prebuilt/clang/"
GCC_PATH="${SCRIPT_DIR}/../prebuilt/gcc-64/"

# Clone Toolchains if they don't exist
if [ ! -d "$CLANG_PATH" ]; then
    git clone --depth=1 git@github.com:jmpfbmx/android_prebuilts_clang_kernel_linux-x86_clang-4691093.git "$CLANG_PATH"
fi

if [ ! -d "$GCC_PATH" ]; then
    git clone --depth=1 git@github.com:jmpfbmx/android_prebuilts_gcc_linux-x86_aarch64_aarch64-linux-android-4.9.git "$GCC_PATH"
fi

# Clean previous build and create output directory
if [ -d "../KERNEL_OUT" ]; then
    rm -rf "$KERNEL_OUT"
    mkdir -p "$KERNEL_OUT"
fi

# Build the kernel
make O="$KERNEL_OUT" ARCH=arm64 amar_row_lte_defconfig

PATH="${CLANG_PATH}/bin:${PATH}:${GCC_PATH}/bin:${PATH}" \
make -j$(nproc --all) O="$KERNEL_OUT" \
                      ARCH=arm64 \
                      CC="clang" \
                      CLANG_TRIPLE=aarch64-linux-gnu- \
                      CROSS_COMPILE="${GCC_PATH}/bin/aarch64-linux-android-"
