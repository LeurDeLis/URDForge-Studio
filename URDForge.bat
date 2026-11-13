@echo off
title URDForge Studio
cd /d "%~dp0"
set PYTHONPATH=src;%PYTHONPATH%
set QTWEBENGINE_CHROMIUM_FLAGS=--disable-direct-composition --enable-gpu-rasterization --ignore-gpu-blocklist --enable-zero-copy --enable-native-gpu-memory-buffers

if exist "D:\Anaconda3\envs\robotarm\python.exe" (
    "D:\Anaconda3\envs\robotarm\python.exe" app.py %*
) else (
    python app.py %*
)
