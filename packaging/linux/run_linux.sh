#!/usr/bin/env bash
# ========================================================
# URDForge Studio - Linux One-Click Launcher
# ========================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PROJECT_ROOT}"

echo "========================================================"
echo "  URDForge Studio - 工业级 STEP CAD 到 URDF 建模与仿真工具"
echo "  平台: Linux (x86_64 / aarch64)"
echo "========================================================"

# Hardware acceleration & GPU flags for Linux (X11 / Wayland)
export QTWEBENGINE_CHROMIUM_FLAGS="--enable-gpu-rasterization --ignore-gpu-blocklist --enable-zero-copy --enable-native-gpu-memory-buffers"
export QT_AUTO_SCREEN_SCALE_FACTOR=1

# Check for standalone binary first
if [ -f "${PROJECT_ROOT}/dist/URDForge-Studio/URDForge-Studio" ]; then
    echo "[INFO] 启动独立预编译程序: dist/URDForge-Studio/URDForge-Studio"
    exec "${PROJECT_ROOT}/dist/URDForge-Studio/URDForge-Studio" "$@"
fi

# Fallback to python environment
PYTHON_CMD="python3"
if command -v conda &> /dev/null; then
    if conda info --envs | grep -q "robotarm"; then
        eval "$(conda shell.bash hook)"
        conda activate robotarm
        PYTHON_CMD="python"
    fi
fi

echo "[INFO] 使用 Python 环境启动: $(${PYTHON_CMD} --version 2>&1)"
exec ${PYTHON_CMD} app.py "$@"
