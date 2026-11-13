#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URDForge Studio - Complete Windows Release Builder.

Packages all Windows delivery artifacts directly into the 'release/' folder:
1. release/URDForge-Studio-v0.2.0-Setup.exe (Graphical Installation Wizard)
2. release/URDForge-Studio-v0.2.0-Portable.exe (Portable Runner Executable)
3. release/URDForge-Studio-v0.2.0-Windows-x64-Portable.zip (Portable Zip)
4. release/URDForge-Studio/ (Extracted ready-to-run folder with launcher bat)
"""

import os
import sys
import shutil
import zipfile
import subprocess
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent.parent
DIST_DIR = ROOT_DIR / "dist"
RELEASE_DIR = ROOT_DIR / "release"
APP_DIR = DIST_DIR / "URDForge-Studio"
PORTABLE_ZIP = RELEASE_DIR / "URDForge-Studio-v0.2.0-Windows-x64-Portable.zip"

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
  URDForge Studio (v0.2.0) - Windows 发布版本说明
========================================================

【本目录包含以下发布文件】：
1. URDForge-Studio-v0.2.0-Setup.exe
   - 官方图形化安装包：一键安装到本地磁盘、自动创建桌面快捷方式与开始菜单项。
2. URDForge-Studio-v0.2.0-Portable.exe
   - 便携版启动可执行文件：双击直接启动，无需安装，随拷随用。
3. URDForge-Studio-v0.2.0-Windows-x64-Portable.zip
   - 独立便携免安装压缩包：包含全部运行环境与启动脚本，解压到任意电脑直接可用。
4. URDForge-Studio/
   - 已解压的便携版应用文件夹，内含 URDForge-Studio.exe 与 启动脚本。

【系统要求】
- 操作系统: 64 位 Windows 10 / Windows 11 (x64)
- 显卡要求: 支持 DirectX 11 / OpenGL 3.3+ (默认启用原生 GPU 硬件加速视口)
- 内存建议: 4GB 以上

【技术支持】
- URDForge Studio Architectural Team
"""


def main():
    print("========================================================")
    print("  URDForge Studio Windows Release Builder")
    print("========================================================")

    if not APP_DIR.exists() or not (APP_DIR / "URDForge-Studio.exe").exists():
        print(f"[Error] Built application not found in {APP_DIR}! Run PyInstaller first.")
        return 1

    RELEASE_DIR.mkdir(parents=True, exist_ok=True)

    # 1. Prepare release/URDForge-Studio/ folder
    target_portable_folder = RELEASE_DIR / "URDForge-Studio"
    print(f"[1/4] Updating release portable folder: {target_portable_folder}...")
    if target_portable_folder.exists():
        shutil.rmtree(target_portable_folder)
    shutil.copytree(APP_DIR, target_portable_folder)

    # Place launcher bat and readme
    with open(target_portable_folder / "启动 URDForge Studio.bat", "w", encoding="utf-8") as f:
        f.write(LAUNCHER_BAT)
    with open(target_portable_folder / "使用说明.txt", "w", encoding="utf-8") as f:
        f.write(README_TXT)

    # 2. Create release/URDForge-Studio-v0.2.0-Windows-x64-Portable.zip
    print(f"[2/4] Generating portable zip package: {PORTABLE_ZIP}...")
    with zipfile.ZipFile(PORTABLE_ZIP, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, dirs, files in os.walk(target_portable_folder):
            for file in files:
                file_path = Path(root) / file
                archive_rel_path = file_path.relative_to(RELEASE_DIR)
                zf.write(file_path, str(archive_rel_path))

    zip_size_mb = PORTABLE_ZIP.stat().st_size / (1024 * 1024)
    print(f"  -> Generated {PORTABLE_ZIP.name} ({zip_size_mb:.1f} MB)")

    # 3. Build release/URDForge-Studio-v0.2.0-Setup.exe (GUI Installer)
    print("[3/4] Building release/URDForge-Studio-v0.2.0-Setup.exe...")
    setup_script = ROOT_DIR / "packaging" / "windows" / "setup_wizard.py"
    icon_ico = str(ROOT_DIR / "assets" / "icon.ico")
    assets_data = f"{str(ROOT_DIR / 'assets')};assets"

    pyinstaller_cmd = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--clean",
        "--noconfirm",
        "--onefile",
        "--windowed",
        "--name",
        "URDForge-Studio-v0.2.0-Setup",
        "--icon",
        icon_ico,
        "--add-data",
        assets_data,
        "--distpath",
        str(RELEASE_DIR),
        str(setup_script),
    ]
    res = subprocess.run(pyinstaller_cmd, cwd=str(ROOT_DIR))
    if res.returncode != 0:
        print("[Warning] PyInstaller building Setup.exe failed!")
    else:
        print("  -> Setup.exe built successfully in release/")

    # 4. Build release/URDForge-Studio-v0.2.0-Portable.exe
    print("[4/4] Building release/URDForge-Studio-v0.2.0-Portable.exe...")
    runner_script = ROOT_DIR / "packaging" / "windows" / "portable_runner.py"
    pyinstaller_runner_cmd = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--clean",
        "--noconfirm",
        "--onefile",
        "--windowed",
        "--name",
        "URDForge-Studio-v0.2.0-Portable",
        "--icon",
        icon_ico,
        "--add-data",
        assets_data,
        "--distpath",
        str(RELEASE_DIR),
        str(runner_script),
    ]
    res_runner = subprocess.run(pyinstaller_runner_cmd, cwd=str(ROOT_DIR))
    if res_runner.returncode != 0:
        print("[Warning] PyInstaller building Portable.exe failed!")
    else:
        print("  -> Portable.exe built successfully in release/")

    # Write root readme in release/
    with open(RELEASE_DIR / "使用说明.txt", "w", encoding="utf-8") as f:
        f.write(README_TXT)

    # Also check if Inno Setup is available to produce an optional inno setup exe
    iss_file = ROOT_DIR / "packaging" / "windows" / "setup_windows.iss"
    iscc_candidates = [
        Path("C:/Program Files (x86)/Inno Setup 6/ISCC.exe"),
        Path("C:/Program Files/Inno Setup 6/ISCC.exe"),
    ]
    for iscc in iscc_candidates:
        if iscc.exists():
            print(f"Found Inno Setup at {iscc}, compiling additional Inno Setup installer...")
            subprocess.run([str(iscc), str(iss_file)], check=False)
            break

    print("\n========================================================")
    print("  [完成] release 目录内容清单:")
    for item in sorted(RELEASE_DIR.iterdir()):
        if item.is_file():
            size_mb = item.stat().st_size / (1024 * 1024)
            print(f"  - {item.name} ({size_mb:.1f} MB)")
        else:
            print(f"  - [文件夹] {item.name}/")
    print("========================================================")
    return 0


if __name__ == "__main__":
    sys.exit(main())
