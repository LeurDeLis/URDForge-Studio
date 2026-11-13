#!/usr/bin/env bash
# ========================================================
# URDForge Studio - Linux System Dependencies & Environment Setup
# ========================================================
set -e

echo "========================================================"
echo "  URDForge Studio - Linux 依赖环境一键安装脚本"
echo "  支持系统: Ubuntu 20.04/22.04/24.04, Debian 11/12, Fedora, Arch"
echo "========================================================"

detect_pkg_manager() {
    if command -v apt-get &> /dev/null; then
        echo "apt"
    elif command -v dnf &> /dev/null; then
        echo "dnf"
    elif command -v pacman &> /dev/null; then
        echo "pacman"
    else
        echo "unknown"
    fi
}

PKG_MGR=$(detect_pkg_manager)

echo "[1/3] 正在安装系统基础图形库与 WebGL / Qt 依赖..."
case "$PKG_MGR" in
    apt)
        sudo apt-get update
        sudo apt-get install -y \
            python3 python3-pip python3-venv \
            libgl1-mesa-glx libgl1-mesa-dri libglu1-mesa \
            libegl1 libxkbcommon-x11-0 libfontconfig1 \
            libdbus-1-3 libnss3 libasound2 libxcomposite1 \
            libxdamage1 libxrandr2 libxtst6 libatk1.0-0 \
            build-essential
        ;;
    dnf)
        sudo dnf install -y \
            python3 python3-pip \
            mesa-libGL mesa-libGLU libxkbcommon-x11 \
            fontconfig dbus-libs nss alsa-lib \
            libXcomposite libXdamage libXrandr
        ;;
    pacman)
        sudo pacman -Sy --noconfirm \
            python python-pip mesa glu \
            libxkbcommon-x11 fontconfig nss alsa-lib
        ;;
    *)
        echo "[警告] 未知包管理器，请手动安装 OpenGL, libxkbcommon, NSS 及 Python3 依赖。"
        ;;
esac

echo ""
echo "[2/3] 正在配置 Python 依赖环境..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PROJECT_ROOT}"

if [ ! -d ".venv" ]; then
    echo "创建独立虚拟环境 .venv..."
    python3 -m venv .venv
fi

source .venv/bin/activate
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt

echo ""
echo "[3/3] 安装完成！"
echo "您可以通过运行 bash packaging/linux/run_linux.sh 或 python3 app.py 启动软件。"
