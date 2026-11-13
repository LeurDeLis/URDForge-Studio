@echo off
chcp 65001 >nul
title URDForge Studio Windows Build Script
cd /d "%~dp0\..\.."

echo ========================================================
echo   URDForge Studio Windows 打包工具
echo   编译可执行程序 (.exe) 与制作便携免安装压缩包 (.zip)
echo ========================================================
echo.

set PYTHON_CMD=python
if exist "D:\Anaconda3\envs\robotarm\python.exe" (
    set PYTHON_CMD="D:\Anaconda3\envs\robotarm\python.exe"
)

echo [1/3] 检查构建环境...
%PYTHON_CMD% -c "import PySide6; print('PySide6 version:', PySide6.__version__)"
if errorlevel 1 (
    echo [错误] 未找到正确的 Python 环境或 PySide6 依赖！
    pause
    exit /b 1
)

echo.
echo [2/3] 正在通过 PyInstaller 构建可执行文件目录 (dist/URDForge-Studio)...
%PYTHON_CMD% -s -m PyInstaller --clean --noconfirm URDForge.spec
if errorlevel 1 (
    echo [错误] PyInstaller 构建失败！
    pause
    exit /b 1
)

echo.
echo [3/3] 正在打包 Windows 免安装便携版 (.zip)...
%PYTHON_CMD% packaging\windows\build_windows_portable.py
if errorlevel 1 (
    echo [错误] 制作便携压缩包失败！
    pause
    exit /b 1
)

echo.
echo ========================================================
echo   [完成] Windows 软件构建已全部完成！
echo   - 可执行程序目录: dist\URDForge-Studio\URDForge-Studio.exe
echo   - 免安装便携版本: dist\URDForge-Studio-v0.2.0-Windows-x64-Portable.zip
echo ========================================================
pause
