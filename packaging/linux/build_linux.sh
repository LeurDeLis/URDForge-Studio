#!/usr/bin/env bash
# ========================================================
# URDForge Studio - Linux Binary Bundle & .deb / .tar.gz Packaging Script
# ========================================================
set -e

VERSION="0.2.0"
APP_NAME="URDForge-Studio"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DIST_DIR="${PROJECT_ROOT}/dist"

cd "${PROJECT_ROOT}"

echo "========================================================"
echo "  URDForge Studio - Linux 打包程序 (v${VERSION})"
echo "========================================================"

PYTHON_CMD="python3"
if [ -d ".venv" ]; then
    source .venv/bin/activate
    PYTHON_CMD="python"
elif command -v conda &> /dev/null && conda info --envs | grep -q "robotarm"; then
    eval "$(conda shell.bash hook)"
    conda activate robotarm
    PYTHON_CMD="python"
fi

echo "[1/4] 安装打包依赖 (PyInstaller)..."
${PYTHON_CMD} -m pip install --upgrade pyinstaller

echo ""
echo "[2/4] 运行 PyInstaller 生成 Linux 独立运行包..."
${PYTHON_CMD} -m PyInstaller --clean --noconfirm URDForge.spec

if [ ! -d "${DIST_DIR}/${APP_NAME}" ]; then
    echo "[错误] 编译失败，未生成 ${DIST_DIR}/${APP_NAME}！"
    exit 1
fi

echo ""
echo "[3/4] 制作通用 Linux 免安装便携压缩包 (.tar.gz)..."
TAR_NAME="${APP_NAME}-v${VERSION}-Linux-x64.tar.gz"
cd "${DIST_DIR}"
tar -czvf "${TAR_NAME}" "${APP_NAME}"
echo "[成功] 已生成 Linux 便携包: ${DIST_DIR}/${TAR_NAME}"

echo ""
echo "[4/4] 检查并制作 Debian / Ubuntu 安装包 (.deb)..."
if command -v dpkg-deb &> /dev/null; then
    DEB_DIR="${DIST_DIR}/deb_build"
    rm -rf "${DEB_DIR}"
    mkdir -p "${DEB_DIR}/DEBIAN"
    mkdir -p "${DEB_DIR}/opt/${APP_NAME}"
    mkdir -p "${DEB_DIR}/usr/bin"
    mkdir -p "${DEB_DIR}/usr/share/applications"
    mkdir -p "${DEB_DIR}/usr/share/icons/hicolor/256x256/apps"

    # Copy files
    cp -r "${DIST_DIR}/${APP_NAME}/"* "${DEB_DIR}/opt/${APP_NAME}/"
    cp "${SCRIPT_DIR}/urdforge.desktop" "${DEB_DIR}/usr/share/applications/"
    
    # Symlink in /usr/bin
    cat << 'EOF' > "${DEB_DIR}/usr/bin/urdforge-studio"
#!/bin/bash
exec /opt/URDForge-Studio/URDForge-Studio "$@"
EOF
    chmod +x "${DEB_DIR}/usr/bin/urdforge-studio"

    # DEBIAN/control file
    cat << EOF > "${DEB_DIR}/DEBIAN/control"
Package: urdforge-studio
Version: ${VERSION}
Section: science
Priority: optional
Architecture: amd64
Maintainer: URDForge Architectural Team <dev@urdforge.org>
Depends: libc6, libgl1-mesa-glx, libegl1, libxkbcommon-x11-0, libnss3, libasound2
Description: Industrial STEP CAD to URDF & ROS2 Modeling Tool
 A standalone native desktop software for industrial robotics modeling,
 composite inertial tensor calculation, and ROS 2 package generation.
EOF

    dpkg-deb --build "${DEB_DIR}" "${DIST_DIR}/urdforge-studio_${VERSION}_amd64.deb"
    rm -rf "${DEB_DIR}"
    echo "[成功] 已生成 Debian/Ubuntu 安装包: ${DIST_DIR}/urdforge-studio_${VERSION}_amd64.deb"
else
    echo "[提示] 未检测到 dpkg-deb，跳过 .deb 打包，便携包 ${TAR_NAME} 已就绪。"
fi

echo ""
echo "========================================================"
echo "  Linux 打包构建全部完成！"
echo "========================================================"
