#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URDForge Studio - Windows Portable Zip Package Builder.

Packages dist/URDForge-Studio into a clean, ready-to-distribute portable zip:
dist/URDForge-Studio-v0.2.0-Windows-x64-Portable.zip
"""

import os
import shutil
import zipfile
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent.parent
DIST_DIR = ROOT_DIR / "dist"
SOURCE_FOLDER = DIST_DIR / "URDForge-Studio"
ZIP_OUTPUT = DIST_DIR / "URDForge-Studio-v0.2.0-Windows-x64-Portable.zip"

LAUNCHER_BAT = """@echo off
chcp 65001 >nul
title URDForge Studio Launcher
cd /d "%~dp0"

echo ========================================================
echo   URDForge Studio - 工业级 STEP CAD 到 URDF 建模与仿真工具
echo   版本: v0.2.0 (Windows x64 便携免安装版)
echo ========================================================
echo.
echo 正在启动 URDForge Studio...

if exist "URDForge-Studio.exe" (
    start "" "URDForge-Studio.exe"
) else if exist "URDForge-Studio\\URDForge-Studio.exe" (
    start "" "URDForge-Studio\\URDForge-Studio.exe"
) else (
    echo [错误] 未找到 URDForge-Studio.exe 可执行文件！
    pause
)
"""

README_TXT = """========================================================
  URDForge Studio (v0.2.0) - Windows 免安装便携版
========================================================

【软件简介】
URDForge Studio 是一款专为工业机器人、具身智能与机械臂设计的
STEP CAD 到 URDF/ROS2 快速建模与仿真工具。

【系统要求】
- 操作系统: 64 位 Windows 10 / Windows 11 (x64)
- 显卡支持: 支持 OpenGL 3.3+ 及 DirectX 11 (内置 WebGL 硬件加速)
- 内存: 推荐 4GB 及以上

【使用说明】
1. 本软件包为“免安装便携版”，解压即用，无需安装任何 Python 或第三方库。
2. 双击运行根目录下的【启动 URDForge Studio.bat】或【URDForge-Studio.exe】即可直接打开主窗口。
3. 如果模型在拖拽或旋转视口时有卡顿，可在“设置”或启动参数中开启硬件加速（默认已启用原生 GPU 光栅化优化）。
4. 导出的 ROS 2 功能包默认生成在您指定的输出目录下，包含：
   - urdf/robot.urdf (包含已修正的 XY 上视基准面及完整的 package:// 路径)
   - meshes/visual/*.stl (高清可视化网格)
   - meshes/collision/*.stl (轻量碰撞检测网格)
   - launch/display.launch.py (RViz2 可视化配置)
   - package.xml & CMakeLists.txt (工业标准构建脚本)

【联系与支持】
- 官方项目: URDForge Studio Architectural Team
- 内部版本: v0.2.0
"""


def main():
    if not SOURCE_FOLDER.exists():
        print(f"[Error] Source directory {SOURCE_FOLDER} does not exist. Run PyInstaller first!")
        return 1

    print(f"Creating portable zip: {ZIP_OUTPUT}...")
    if ZIP_OUTPUT.exists():
        ZIP_OUTPUT.unlink()

    # Create temporary packaging folder
    temp_pack_dir = DIST_DIR / "temp_portable_pack"
    if temp_pack_dir.exists():
        shutil.rmtree(temp_pack_dir)
    temp_pack_dir.mkdir(parents=True, exist_ok=True)

    target_app_dir = temp_pack_dir / "URDForge-Studio"
    print("Copying application binaries...")
    shutil.copytree(SOURCE_FOLDER, target_app_dir)

    # Write root launcher and readme
    with open(temp_pack_dir / "启动 URDForge Studio.bat", "w", encoding="utf-8") as f:
        f.write(LAUNCHER_BAT)

    with open(temp_pack_dir / "使用说明.txt", "w", encoding="utf-8") as f:
        f.write(README_TXT)

    # Also place launcher directly inside URDForge-Studio folder for convenience
    with open(target_app_dir / "启动 URDForge Studio.bat", "w", encoding="utf-8") as f:
        f.write(LAUNCHER_BAT)

    with open(target_app_dir / "使用说明.txt", "w", encoding="utf-8") as f:
        f.write(README_TXT)

    print("Archiving to ZIP (this may take 10-30 seconds)...")
    with zipfile.ZipFile(ZIP_OUTPUT, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, dirs, files in os.walk(temp_pack_dir):
            for file in files:
                file_path = Path(root) / file
                archive_rel_path = file_path.relative_to(temp_pack_dir)
                zf.write(file_path, str(archive_rel_path))

    # Clean up temp folder
    shutil.rmtree(temp_pack_dir)

    file_size_mb = ZIP_OUTPUT.stat().st_size / (1024 * 1024)
    print(f"[Success] Portable package created: {ZIP_OUTPUT} ({file_size_mb:.1f} MB)")
    return 0


if __name__ == "__main__":
    exit(main())
