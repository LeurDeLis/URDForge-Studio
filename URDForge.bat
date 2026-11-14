@echo off
title URDForge Studio
cd /d "%~dp0"
set PYTHONPATH=src;%PYTHONPATH%

if exist "%~dp0release\URDForge-Studio.exe" (
    start "" "%~dp0release\URDForge-Studio.exe"
    exit /b 0
)

if exist "D:\Anaconda3\envs\robotarm\python.exe" (
    "D:\Anaconda3\envs\robotarm\python.exe" app.py %*
) else (
    python app.py %*
)
