# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller build specification for URDForge Studio standalone desktop executable."""

import os
import sys
from pathlib import Path
from PyInstaller.utils.hooks import collect_all

block_cipher = None
ROOT_DIR = Path.cwd()

datas = [
    (str(ROOT_DIR / "src" / "urdforge" / "server" / "static"), "urdforge/server/static"),
    (str(ROOT_DIR / "src" / "urdforge" / "server" / "static"), "src/urdforge/server/static"),
    (str(ROOT_DIR / "tests" / "fixtures"), "tests/fixtures"),
    (str(ROOT_DIR / "assets"), "assets"),
]
binaries = []
hiddenimports = [
    "OCP",
    "open3d",
    "numpy",
    "scipy",
    "scipy.spatial.transform",
    "PySide6.QtWebEngineWidgets",
    "PySide6.QtWebEngineCore",
    "PySide6.QtWidgets",
    "PySide6.QtCore",
    "PySide6.QtGui",
    "python_multipart",
    "multipart",
    "urdforge",
    "urdforge.core",
    "urdforge.core.math_types",
    "urdforge.core.physics",
    "urdforge.core.mesher",
    "urdforge.core.step_parser",
    "urdforge.core.features",
    "urdforge.core.kinematics",
    "urdforge.core.robot_model",
    "urdforge.exporter",
    "urdforge.exporter.ros_package",
    "urdforge.server",
    "urdforge.server.app",
    "urdforge.utils",
    "urdforge.utils.naming",
]

# Fully collect all backend ASGI modules and dependencies
for pkg in ("uvicorn", "fastapi", "starlette", "pydantic", "pydantic_core", "anyio", "sniffio", "h11", "click"):
    try:
        pkg_datas, pkg_binaries, pkg_hidden = collect_all(pkg)
        datas += pkg_datas
        binaries += pkg_binaries
        hiddenimports += pkg_hidden
    except Exception as e:
        print(f"[Warning] Failed to collect_all({pkg}): {e}")

a = Analysis(
    ["app.py"],
    pathex=[
        str(ROOT_DIR),
        str(ROOT_DIR / "src"),
    ],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="URDForge-Studio",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,  # Windowed desktop application (no cmd popup)
    icon=str(ROOT_DIR / "assets" / "icon.ico"),
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="URDForge-Studio",
)
