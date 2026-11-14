#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URDForge Studio - Portable Standalone Launcher.

Self-contained portable launcher for URDForge Studio.
Launches the application directly without requiring installation.
"""

import os
import sys
import zipfile
import subprocess
from pathlib import Path


def main():
    exe_path = Path(sys.argv[0]).resolve()
    exe_dir = exe_path.parent

    # 1. Check if local directory already contains URDForge-Studio.exe
    candidates = [
        exe_dir / "URDForge-Studio.exe",
        exe_dir / "URDForge-Studio" / "URDForge-Studio.exe",
        exe_dir.parent / "dist" / "URDForge-Studio" / "URDForge-Studio.exe",
    ]
    for cand in candidates:
        if cand.exists():
            subprocess.Popen([str(cand)] + sys.argv[1:], cwd=str(cand.parent))
            return 0

    # 2. Check if embedded payload or adjacent portable zip exists to extract to LocalAppData
    local_app_data = Path(os.environ.get("LOCALAPPDATA", os.path.expanduser("~"))) / "URDForge-Studio-Portable"
    target_exe = local_app_data / "URDForge-Studio" / "URDForge-Studio.exe"
    if not target_exe.exists():
        target_exe = local_app_data / "URDForge-Studio.exe"

    # Find payload zip
    payload_zip = None
    if hasattr(sys, "_MEIPASS"):
        bundled = Path(sys._MEIPASS) / "payload.zip"
        if bundled.exists():
            payload_zip = bundled

    if not payload_zip:
        for cand in [
            exe_dir / "URDForge-Studio-v0.2.0-Windows-x64-Portable.zip",
            exe_dir / "payload.zip",
        ]:
            if cand.exists():
                payload_zip = cand
                break

    if payload_zip:
        need_extract = False
        if not target_exe.exists():
            need_extract = True
        elif payload_zip.stat().st_mtime > target_exe.stat().st_mtime:
            need_extract = True

        if need_extract:
            local_app_data.mkdir(parents=True, exist_ok=True)
            with zipfile.ZipFile(payload_zip, "r") as zf:
                zf.extractall(local_app_data)

            if not target_exe.exists():
                target_exe = local_app_data / "URDForge-Studio" / "URDForge-Studio.exe"

    if target_exe.exists():
        subprocess.Popen([str(target_exe)] + sys.argv[1:], cwd=str(target_exe.parent))
        return 0

    print("[Error] Could not find or launch URDForge-Studio.exe!")
    return 1


if __name__ == "__main__":
    sys.exit(main())
