#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URDForge Studio - Native Windows GUI Setup Wizard & Installer.

A standalone, modern graphical installer for URDForge Studio.
Extracts files, creates Desktop and Start Menu shortcuts, and registers uninstaller.
"""

import os
import sys
import shutil
import zipfile
import threading
import subprocess
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

APP_NAME = "URDForge Studio"
APP_VERSION = "0.2.0"
DEFAULT_SUBDIR = "URDForge Studio"


def get_default_install_dir() -> Path:
    local_app_data = os.environ.get("LOCALAPPDATA")
    if local_app_data:
        return Path(local_app_data) / "Programs" / DEFAULT_SUBDIR
    return Path(os.environ.get("USERPROFILE", "C:\\")) / DEFAULT_SUBDIR


def get_embedded_zip_path() -> Path:
    """Find the payload zip file either bundled in PyInstaller MEIPASS or next to exe."""
    if hasattr(sys, "_MEIPASS"):
        candidate = Path(sys._MEIPASS) / "payload.zip"
        if candidate.exists():
            return candidate

    exe_dir = Path(sys.argv[0]).resolve().parent
    for candidate in (
        exe_dir / "URDForge-Studio-v0.2.0-Windows-x64-Portable.zip",
        exe_dir / "payload.zip",
        exe_dir.parent / "dist" / "URDForge-Studio-v0.2.0-Windows-x64-Portable.zip",
        exe_dir / "dist" / "URDForge-Studio-v0.2.0-Windows-x64-Portable.zip",
    ):
        if candidate.exists():
            return candidate
    return Path()


def create_shortcut(target_exe: Path, shortcut_path: Path, description: str = APP_NAME):
    """Create a Windows .lnk shortcut using PowerShell WScript.Shell."""
    try:
        shortcut_path.parent.mkdir(parents=True, exist_ok=True)
        ps_script = f"""
$ws = New-Object -ComObject WScript.Shell
$s = $ws.CreateShortcut('{str(shortcut_path)}')
$s.TargetPath = '{str(target_exe)}'
$s.WorkingDirectory = '{str(target_exe.parent)}'
$s.Description = '{description}'
$s.Save()
"""
        subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", ps_script],
            creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0,
            check=False,
        )
    except Exception as e:
        print(f"Failed to create shortcut: {e}")


class InstallerApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(f"{APP_NAME} v{APP_VERSION} 安装向导")
        self.root.geometry("560x420")
        self.root.resizable(False, False)

        # Style configuration
        self.style = ttk.Style()
        try:
            self.style.theme_use("vista")
        except Exception:
            pass

        self.install_dir_var = tk.StringVar(value=str(get_default_install_dir()))
        self.create_desktop_icon_var = tk.BooleanVar(value=True)
        self.create_start_menu_var = tk.BooleanVar(value=True)
        self.launch_after_var = tk.BooleanVar(value=True)

        self.is_installing = False
        self.build_ui()

    def build_ui(self):
        # Header banner
        header_frame = tk.Frame(self.root, bg="#1e293b", height=75)
        header_frame.pack(fill=tk.X)
        header_frame.pack_propagate(False)

        title_label = tk.Label(
            header_frame,
            text=f"欢迎使用 {APP_NAME} 安装向导",
            font=("Segoe UI", 13, "bold"),
            fg="#ffffff",
            bg="#1e293b",
        )
        title_label.pack(anchor="w", padx=20, pady=(12, 2))

        subtitle_label = tk.Label(
            header_frame,
            text=f"工业级 STEP CAD 到 URDF/ROS2 建模与仿真工具 (版本: v{APP_VERSION})",
            font=("Segoe UI", 9),
            fg="#94a3b8",
            bg="#1e293b",
        )
        subtitle_label.pack(anchor="w", padx=20)

        # Main content area
        content_frame = tk.Frame(self.root, padx=24, pady=16)
        content_frame.pack(fill=tk.BOTH, expand=True)

        # Installation path selection
        path_label = tk.Label(
            content_frame,
            text="选择安装目标文件夹：",
            font=("Segoe UI", 10, "bold"),
            anchor="w",
        )
        path_label.pack(fill=tk.X, pady=(0, 6))

        dir_frame = tk.Frame(content_frame)
        dir_frame.pack(fill=tk.X, pady=(0, 14))

        self.dir_entry = ttk.Entry(dir_frame, textvariable=self.install_dir_var, font=("Segoe UI", 9))
        self.dir_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))

        browse_btn = ttk.Button(dir_frame, text="浏览...", command=self.browse_dir)
        browse_btn.pack(side=tk.RIGHT)

        # Shortcuts options
        opt_label = tk.Label(
            content_frame,
            text="快捷方式选项：",
            font=("Segoe UI", 10, "bold"),
            anchor="w",
        )
        opt_label.pack(fill=tk.X, pady=(0, 4))

        desktop_check = ttk.Checkbutton(
            content_frame,
            text="创建桌面快捷方式",
            variable=self.create_desktop_icon_var,
        )
        desktop_check.pack(anchor="w", pady=2)

        start_menu_check = ttk.Checkbutton(
            content_frame,
            text="添加到开始菜单应用列表",
            variable=self.create_start_menu_var,
        )
        start_menu_check.pack(anchor="w", pady=2)

        launch_check = ttk.Checkbutton(
            content_frame,
            text="安装完成后立即启动 URDForge Studio",
            variable=self.launch_after_var,
        )
        launch_check.pack(anchor="w", pady=2)

        # Progress bar
        self.status_label = tk.Label(
            content_frame,
            text="准备就绪，点击下方【开始安装】按钮继续。",
            font=("Segoe UI", 9),
            fg="#475569",
            anchor="w",
        )
        self.status_label.pack(fill=tk.X, pady=(16, 4))

        self.progress_bar = ttk.Progressbar(content_frame, mode="determinate")
        self.progress_bar.pack(fill=tk.X, pady=(0, 10))

        # Bottom buttons
        btn_frame = tk.Frame(self.root, padx=20, pady=12, bg="#f1f5f9")
        btn_frame.pack(fill=tk.X, side=tk.BOTTOM)

        self.cancel_btn = ttk.Button(btn_frame, text="取消", command=self.root.destroy)
        self.cancel_btn.pack(side=tk.RIGHT, padx=(8, 0))

        self.install_btn = ttk.Button(btn_frame, text="开始安装", command=self.start_installation)
        self.install_btn.pack(side=tk.RIGHT)

    def browse_dir(self):
        chosen = filedialog.askdirectory(initialdir=self.install_dir_var.get())
        if chosen:
            self.install_dir_var.set(str(Path(chosen) / DEFAULT_SUBDIR))

    def start_installation(self):
        if self.is_installing:
            return
        install_path = Path(self.install_dir_var.get()).resolve()

        zip_path = get_embedded_zip_path()
        if not zip_path.exists():
            messagebox.showerror(
                "安装错误",
                f"未能找到核心安装载荷压缩包！\n请确保安装程序包完整。\n尝试查找路径:\n{zip_path}",
            )
            return

        self.is_installing = True
        self.install_btn.config(state=tk.DISABLED)
        self.dir_entry.config(state=tk.DISABLED)
        self.status_label.config(text="正在准备安装目标目录...", fg="#0284c7")

        thread = threading.Thread(target=self._run_install_thread, args=(install_path, zip_path), daemon=True)
        thread.start()

    def _run_install_thread(self, install_path: Path, zip_path: Path):
        try:
            install_path.mkdir(parents=True, exist_ok=True)

            with zipfile.ZipFile(zip_path, "r") as zf:
                all_files = zf.infolist()
                total = len(all_files)
                for i, info in enumerate(all_files):
                    zf.extract(info, install_path)
                    if i % 25 == 0 or i == total - 1:
                        percent = int((i + 1) / total * 100)
                        self.root.after(0, self._update_progress, percent, f"正在解压组件: {info.filename[:45]}...")

            # Locate extracted URDForge-Studio.exe
            exe_target = install_path / "URDForge-Studio.exe"
            if not exe_target.exists():
                nested_exe = install_path / "URDForge-Studio" / "URDForge-Studio.exe"
                if nested_exe.exists():
                    exe_target = nested_exe

            # Create uninstaller script
            uninstaller_bat = install_path / "卸载 URDForge Studio.bat"
            with open(uninstaller_bat, "w", encoding="utf-8") as f:
                f.write(f"""@echo off
chcp 65001 >nul
title 卸载 URDForge Studio
echo 确定要卸载 URDForge Studio 吗？
choice /M "是否继续"
if errorlevel 2 exit /b
echo 正在清理快捷方式...
del "%USERPROFILE%\\Desktop\\{APP_NAME}.lnk" 2>nul
del "%APPDATA%\\Microsoft\\Windows\\Start Menu\\Programs\\{APP_NAME}.lnk" 2>nul
echo 正在删除安装文件...
cd ..
rd /s /q "{install_path}" 2>nul
echo 卸载完成！
pause
""")

            # Create Desktop Shortcut
            if self.create_desktop_icon_var.get() and exe_target.exists():
                desktop = Path(os.environ.get("USERPROFILE", "C:\\")) / "Desktop"
                shortcut = desktop / f"{APP_NAME}.lnk"
                self.root.after(0, self._update_progress, 95, "正在创建桌面快捷方式...")
                create_shortcut(exe_target, shortcut)

            # Create Start Menu Shortcut
            if self.create_start_menu_var.get() and exe_target.exists():
                appdata = os.environ.get("APPDATA")
                if appdata:
                    start_menu = Path(appdata) / "Microsoft" / "Windows" / "Start Menu" / "Programs"
                    shortcut = start_menu / f"{APP_NAME}.lnk"
                    self.root.after(0, self._update_progress, 98, "正在创建开始菜单项...")
                    create_shortcut(exe_target, shortcut)

            self.root.after(0, self._on_install_complete, exe_target)

        except Exception as e:
            self.root.after(0, self._on_install_error, str(e))

    def _update_progress(self, percent: int, text: str):
        self.progress_bar["value"] = percent
        self.status_label.config(text=text)

    def _on_install_complete(self, exe_target: Path):
        self.progress_bar["value"] = 100
        self.status_label.config(text="恭喜，URDForge Studio 已成功安装到您的计算机！", fg="#16a34a")
        self.cancel_btn.config(text="完成", command=self.root.destroy)
        self.install_btn.pack_forget()

        if self.launch_after_var.get() and exe_target.exists():
            subprocess.Popen([str(exe_target)], cwd=str(exe_target.parent))
            self.root.destroy()
        else:
            messagebox.showinfo("安装成功", f"{APP_NAME} 已成功安装！\n您可以通过桌面或开始菜单快捷方式启动。")

    def _on_install_error(self, err_msg: str):
        self.is_installing = False
        self.install_btn.config(state=tk.NORMAL)
        self.status_label.config(text=f"安装失败: {err_msg}", fg="#dc2626")
        messagebox.showerror("安装出错", f"安装过程中发生错误：\n{err_msg}")


def main():
    root = tk.Tk()
    app = InstallerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
