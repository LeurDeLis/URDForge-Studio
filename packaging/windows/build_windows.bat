@echo off
chcp 65001 >nul
title URDForge Studio Windows Release Builder
cd /d "%~dp0\..\.."

echo ========================================================
echo   URDForge Studio Windows 打包与发布工具
echo   编译可执行程序并输出至 release/ 目录
echo ========================================================
echo.

set PYTHON_CMD=python
if exist "D:\Anaconda3\envs\robotarm\python.exe" (
    set PYTHON_CMD="D:\Anaconda3\envs\robotarm\python.exe"
)

echo [1/3] 检查 Python 与依赖环境...
%PYTHON_CMD% -c "import PySide6, uvicorn, fastapi; print('All core modules available.')"
if errorlevel 1 (
    echo [错误] Python 依赖检查未通过！
    pause
    exit /b 1
)

echo.
echo [2/3] 正在通过 PyInstaller 编译主程序 (dist/URDForge-Studio)...
%PYTHON_CMD% -m PyInstaller --clean --noconfirm URDForge.spec
if errorlevel 1 (
    echo [错误] 主程序 PyInstaller 编译失败！
    pause
    exit /b 1
)

echo.
echo [3/3] 正在构建 release/ 目录发布包（安装包exe、便携版exe、便携压缩包）...
%PYTHON_CMD% packaging\windows\build_windows_release.py
if errorlevel 1 (
    echo [错误] release 打包构建失败！
    pause
    exit /b 1
)

echo.
echo ========================================================
echo   [完成] release/ 发布包已全部就绪！
echo ========================================================
pause
