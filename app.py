#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URDForge Studio - Primary Application Launcher & Geometry Server.

Runs the FastAPI OpenCASCADE backend and automatically opens the user interface.
Zero PySide6 / Qt dependencies - light, fast, and cross-platform.

Usage:
    python app.py
    python app.py --port 8765
    python app.py --no-browser
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import threading
import time
import urllib.request
import webbrowser
from pathlib import Path
from typing import Optional

# Ensure non-null streams and UTF-8 encoding on Windows
if sys.platform == "win32":
    if sys.stdout is None:
        try:
            sys.stdout = open(os.devnull, "w", encoding="utf-8")
        except Exception:
            pass
    if sys.stderr is None:
        try:
            sys.stderr = open(os.devnull, "w", encoding="utf-8")
        except Exception:
            pass

    for stream in (sys.stdout, sys.stderr):
        if stream and hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass

CURRENT_DIR = Path(__file__).resolve().parent
SRC_DIR = CURRENT_DIR / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import uvicorn

try:
    from urdforge.server.app import app as fastapi_app
except ImportError:
    from src.urdforge.server.app import app as fastapi_app


def find_free_port(preferred_port: int = 8765) -> int:
    """Check if preferred_port is available; if not, bind to an ephemeral free port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.2)
        if s.connect_ex(("127.0.0.1", preferred_port)) != 0:
            return preferred_port

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_and_open_browser(url: str, timeout: float = 10.0) -> None:
    """Poll health endpoint until server is ready, then open the default browser."""
    start_time = time.time()
    health_url = f"{url}/api/health"

    while time.time() - start_time < timeout:
        try:
            with urllib.request.urlopen(health_url, timeout=0.2) as resp:
                if resp.status == 200:
                    time.sleep(0.15)
                    webbrowser.open(url)
                    return
        except Exception:
            pass
        time.sleep(0.15)

    # Fallback open even if health check polling timed out
    webbrowser.open(url)


def main() -> None:
    parser = argparse.ArgumentParser(description="URDForge Studio Application Launcher")
    parser.add_argument("--host", default="127.0.0.1", help="Host address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8765, help="Port (default: 8765)")
    parser.add_argument("--no-browser", action="store_true", help="Do not open browser automatically")
    args = parser.parse_args()

    port = find_free_port(args.port)
    url = f"http://{args.host}:{port}"

    print("=" * 68)
    print("  🚀 URDForge Studio - 工业级 STEP CAD 转 URDF 建模软件")
    print(f"  📡 后端几何服务地址: {url}")
    print("  🖥️ 架构方案: 独立轻量端 (Tauri + Python Backend)")
    print("=" * 68)

    if not args.no_browser:
        t = threading.Thread(target=wait_and_open_browser, args=(url,), daemon=True)
        t.start()

    config = uvicorn.Config(
        fastapi_app,
        host=args.host,
        port=port,
        log_level="info",
        access_log=False,
    )
    server = uvicorn.Server(config)
    try:
        server.run()
    except (KeyboardInterrupt, SystemExit):
        print("\n[URDForge] 服务已安全关闭。")


if __name__ == "__main__":
    main()
