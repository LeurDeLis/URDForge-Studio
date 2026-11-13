#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URDForge Studio Primary Native Desktop Application.

A standalone, native desktop software for industrial STEP CAD-to-URDF modeling.
Embeds hardware-accelerated 3D viewport without requiring external web browsers.

Usage:
    python app.py
"""

from __future__ import annotations

import json
import os
import socket
import sys
import threading
import time
import urllib.request
from pathlib import Path
from typing import Optional

# Ensure UTF-8 output encoding on Windows consoles
if sys.platform == "win32":
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass

    # Hardware acceleration & presentation smoothness flags:
    # - Disable DirectComposition overlay planes to prevent presentation flicker on Windows
    # - Enable GPU rasterization, zero-copy, and native GPU memory buffers for 60-144 FPS smooth 3D dragging
    os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = (
        "--disable-direct-composition "
        "--enable-gpu-rasterization "
        "--ignore-gpu-blocklist "
        "--enable-zero-copy "
        "--enable-native-gpu-memory-buffers"
    )

CURRENT_DIR = Path(__file__).resolve().parent
SRC_DIR = CURRENT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from PySide6.QtCore import QCoreApplication, QEventLoop, QObject, QTimer, QUrl, Qt, Signal, Slot
from PySide6.QtGui import QAction, QColor, QFont, QIcon, QKeySequence
from PySide6.QtWebEngineCore import QWebEnginePage, QWebEngineSettings
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QMainWindow,
    QMenu,
    QMenuBar,
    QMessageBox,
    QProgressBar,
    QStatusBar,
)

import uvicorn

try:
    from urdforge.server.app import app as fastapi_app
except ImportError:
    from src.urdforge.server.app import app as fastapi_app


def find_free_port(preferred_port: int = 8765) -> int:
    """Check if preferred_port is available; if not, bind to an ephemeral free port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(("127.0.0.1", preferred_port))
            return preferred_port
        except OSError:
            pass

    # Ephemeral port
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class BackendServerThread(threading.Thread):
    """Background daemon thread running the internal FastAPI geometry server."""

    def __init__(self, host: str, port: int) -> None:
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.server = None

    def run(self) -> None:
        config = uvicorn.Config(
            fastapi_app,
            host=self.host,
            port=self.port,
            log_level="warning",
            access_log=False,
        )
        self.server = uvicorn.Server(config)
        self.server.run()


class URDForgeMainWindow(QMainWindow):
    """Primary native desktop application window for URDForge Studio."""

    def __init__(self, host: str, port: int) -> None:
        super().__init__()
        self.host = host
        self.port = port
        self.base_url = f"http://{host}:{port}"
        self.current_loaded_file: Optional[str] = None

        self.setWindowTitle("URDForge Studio | 机器人建模与仿真工具 (v0.2.0)")
        self.resize(1440, 900)
        self._center_window()

        # 1. Web View
        self.view = QWebEngineView(self)
        self.setCentralWidget(self.view)

        # Configure WebEngine Settings
        settings = self.view.settings()
        settings.setAttribute(QWebEngineSettings.WebAttribute.WebGLEnabled, True)
        settings.setAttribute(QWebEngineSettings.WebAttribute.Accelerated2dCanvasEnabled, True)
        settings.setAttribute(QWebEngineSettings.WebAttribute.JavascriptEnabled, True)
        settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
        settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
        self.view.page().setBackgroundColor(QColor("#f4f6f8"))

        # 2. Status Bar
        self.status_bar = QStatusBar(self)
        self.setStatusBar(self.status_bar)
        self.status_bar.setStyleSheet(
            "background-color: #f4f6f8; color: #66758d; font-size: 12px; border-top: 1px solid #d6dde5; font-family: 'Segoe UI', 'Microsoft YaHei UI', sans-serif;"
        )
        self.status_bar.showMessage("准备就绪 · 本地几何计算内核已启动 (OpenCASCADE 7.9)")

        # 3. Create Menus
        self._setup_menus()

        # 4. Apply B1_Robotic_Arm_App Clean Studio Light Theme to Native Widgets
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f4f6f8;
            }
            QMenuBar {
                background-color: #ffffff;
                color: #14233b;
                font-size: 13px;
                border-bottom: 1px solid #d6dde5;
                font-family: 'Segoe UI', 'Microsoft YaHei UI', sans-serif;
            }
            QMenuBar::item {
                background: transparent;
                padding: 6px 12px;
                color: #14233b;
            }
            QMenuBar::item:selected {
                background-color: #eaf2fb;
                color: #1976d2;
                border-radius: 4px;
            }
            QMenu {
                background-color: #ffffff;
                color: #14233b;
                border: 1px solid #cbd5e1;
                padding: 4px;
                font-family: 'Segoe UI', 'Microsoft YaHei UI', sans-serif;
            }
            QMenu::item {
                padding: 6px 24px;
                border-radius: 4px;
                color: #14233b;
            }
            QMenu::item:selected {
                background-color: #eaf2fb;
                color: #1976d2;
            }
            QMenu::separator {
                height: 1px;
                background-color: #e2e8f0;
                margin: 4px 8px;
            }
        """)

        # 5. Wait for backend ready & Load UI
        self._wait_and_load_url()

    def _center_window(self) -> None:
        """Center the window on the active monitor."""
        screen = QApplication.primaryScreen()
        if screen:
            rect = screen.availableGeometry()
            x = (rect.width() - self.width()) // 2
            y = (rect.height() - self.height()) // 2
            self.move(x, y)

    def _setup_menus(self) -> None:
        """Create native menu bar."""
        menubar = self.menuBar()

        # --- Menu: 文件 (File) ---
        file_menu = menubar.addMenu("文件 (&F)")

        act_open = QAction("📁 打开本地 STEP 装配体... (&O)", self)
        act_open.setShortcut(QKeySequence.Open)
        act_open.triggered.connect(self.action_open_step_file)
        file_menu.addAction(act_open)

        file_menu.addSeparator()

        act_export_urdf = QAction("📄 导出当前 URDF 模型... (&U)", self)
        act_export_urdf.setShortcut(QKeySequence.Save)
        act_export_urdf.triggered.connect(self.action_export_urdf)
        file_menu.addAction(act_export_urdf)

        act_export_ros = QAction("🚀 导出 ROS 2 机器人功能包... (&R)", self)
        act_export_ros.setShortcut(QKeySequence("Ctrl+E"))
        act_export_ros.triggered.connect(self.action_export_ros)
        file_menu.addAction(act_export_ros)

        file_menu.addSeparator()

        act_exit = QAction("❌ 退出 (&X)", self)
        act_exit.setShortcut(QKeySequence.Quit)
        act_exit.triggered.connect(self.close)
        file_menu.addAction(act_exit)

        # --- Menu: 视图 (View) ---
        view_menu = menubar.addMenu("视图 (&V)")

        act_snap = QAction("🎯 切换几何特征吸附模式 (&S)", self)
        act_snap.setShortcut(QKeySequence("F2"))
        act_snap.triggered.connect(lambda: self.run_js("document.getElementById('btnToggleSnap').click()"))
        view_menu.addAction(act_snap)

        act_refresh = QAction("🔄 刷新视图 (&R)", self)
        act_refresh.setShortcut(QKeySequence.Refresh)
        act_refresh.triggered.connect(self.view.reload)
        view_menu.addAction(act_refresh)

        act_fullscreen = QAction("🖥️ 全屏模式 (&F)", self)
        act_fullscreen.setShortcut(QKeySequence.FullScreen)
        act_fullscreen.triggered.connect(self._toggle_fullscreen)
        view_menu.addAction(act_fullscreen)

        # --- Menu: 帮助 (Help) ---
        help_menu = menubar.addMenu("帮助 (&H)")

        act_about = QAction("ℹ️ 关于 URDForge Studio (&A)", self)
        act_about.triggered.connect(self.action_about)
        help_menu.addAction(act_about)

    def _toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def _wait_and_load_url(self) -> None:
        """Non-blocking timer poll to load webview once server is responsive."""
        self._poll_attempts = 0
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(100)  # Check every 100ms

        def check():
            self._poll_attempts += 1
            try:
                with urllib.request.urlopen(f"{self.base_url}/api/step/features", timeout=0.1) as r:
                    if r.status == 200:
                        self._poll_timer.stop()
                        self.view.load(QUrl(self.base_url))
                        return
            except Exception:
                pass

            if self._poll_attempts >= 50:
                self._poll_timer.stop()
                self.view.load(QUrl(self.base_url))

        self._poll_timer.timeout.connect(check)
        self._poll_timer.start()

    def run_js(self, code: str) -> None:
        """Execute a JavaScript snippet in the webview."""
        self.view.page().runJavaScript(code)

    def load_step_path(self, filepath: str | Path) -> None:
        """Directly instruct the webview to parse and load the given STEP path."""
        p = str(Path(filepath).resolve()).replace("\\", "/")
        self.current_loaded_file = p
        self.status_bar.showMessage(f"正在加载 CAD 文件: {p}...")
        js_code = f"loadStepAssembly({json.dumps(p)});"
        self.run_js(js_code)
        self.status_bar.showMessage(f"已加载: {Path(p).name}", 5000)

    # --- Actions ---

    @Slot()
    def action_open_step_file(self) -> None:
        """Native OS file dialog to open custom STEP files."""
        filepath, _ = QFileDialog.getOpenFileName(
            self,
            "选择 STEP 机器人装配体文件",
            "",
            "STEP CAD Files (*.step *.stp *.STEP *.STP);;All Files (*.*)",
        )
        if filepath:
            self.load_step_path(filepath)

    @Slot()
    def action_export_urdf(self) -> None:
        """Native OS file dialog to export URDF XML."""
        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "导出 URDF 机器人描述文件",
            "robot.urdf",
            "URDF Files (*.urdf);;XML Files (*.xml);;All Files (*.*)",
        )
        if not filepath:
            return

        try:
            with urllib.request.urlopen(f"{self.base_url}/api/robot/urdf") as res:
                urdf_xml = res.read().decode("utf-8")
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(urdf_xml)
            QMessageBox.information(self, "导出成功", f"URDF 模型已成功导出至:\n{filepath}")
        except Exception as e:
            QMessageBox.critical(self, "导出失败", f"无法导出 URDF 文件:\n{str(e)}")

    @Slot()
    def action_export_ros(self) -> None:
        """Native OS folder picker to export ROS 2 package."""
        target_dir = QFileDialog.getExistingDirectory(
            self,
            "选择 ROS 2 功能包保存目录",
            "",
        )
        if not target_dir:
            return

        try:
            req_data = json.dumps({"target_directory": target_dir}).encode("utf-8")
            req = urllib.request.Request(
                f"{self.base_url}/api/robot/export_ros",
                data=req_data,
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(req) as res:
                data = json.loads(res.read().decode("utf-8"))

            QMessageBox.information(
                self,
                "导出成功",
                f"🎉 ROS 2 机器人功能包导出成功！\n\n"
                f"• 包名称: {data.get('package_name')}\n"
                f"• 目标路径: {data.get('package_directory')}\n"
                f"• 生成文件数: {len(data.get('files_generated', []))} 个\n\n"
                f"包含完整的 CMakeLists.txt, package.xml, launch, rviz 与 mesh 目录。",
            )
        except Exception as e:
            QMessageBox.critical(self, "导出失败", f"无法导出 ROS 2 功能包:\n{str(e)}")

    @Slot()
    def action_about(self) -> None:
        """About dialog."""
        QMessageBox.about(
            self,
            "关于 URDForge Studio",
            """<h3>URDForge Studio v0.2.0</h3>
            <p><b>跨平台、免 CAD 商业授权依赖的现代化机器人建模软件</b></p>
            <ul>
              <li>直接解析工业标准 <b>STEP AP214 / AP242</b> 装配体</li>
              <li>基于 <b>OpenCASCADE (OCP)</b> 的高精度物性与质心惯性张量计算</li>
              <li>严格遵从 <b>Isaac Sim / MuJoCo / Gazebo</b> 求解器物理合法性标准</li>
              <li>独创 <b>B-Rep 特征吸附 (Feature Snapping)</b>，一键对齐 Joint 轴向</li>
              <li>集成 60 FPS 实时前向运动学（FK）动态调试视口</li>
              <li>一键生成生产级 <b>ROS / ROS 2 功能包</b></li>
            </ul>
            <p>© 2026 URDForge Architectural Team. All Rights Reserved.</p>
            """,
        )


def main() -> None:
    """Desktop Application Entry Point."""
    # Note: Do not set AA_ShareOpenGLContexts in Qt 6 on Windows, as forcing OpenGL context
    # sharing breaks D3D11 / ANGLE swapchain synchronization in QWebEngine and causes screen flicker.

    app = QApplication(sys.argv)
    app.setApplicationName("URDForge Studio")
    app.setOrganizationName("URDForge")

    port = find_free_port(8765)
    host = "127.0.0.1"

    # Start FastAPI geometry backend in background thread
    server_thread = BackendServerThread(host=host, port=port)
    server_thread.start()

    # Launch Native Window
    main_window = URDForgeMainWindow(host=host, port=port)
    main_window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
