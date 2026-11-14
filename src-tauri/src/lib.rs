use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::process::{Child, Command};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tauri::{Manager, RunEvent};

#[cfg(target_os = "windows")]
use std::os::windows::process::CommandExt;

#[cfg(target_os = "windows")]
const CREATE_NO_WINDOW: u32 = 0x08000000;

fn find_project_root() -> PathBuf {
    if let Ok(cwd) = std::env::current_dir() {
        if cwd.join("run_backend.py").exists() || cwd.join("src").join("urdforge").exists() {
            return cwd;
        }
    }
    if let Ok(exe) = std::env::current_exe() {
        let mut cur = exe.as_path();
        while let Some(parent) = cur.parent() {
            if parent.join("run_backend.py").exists() || parent.join("src").join("urdforge").exists() {
                return parent.to_path_buf();
            }
            cur = parent;
        }
    }
    PathBuf::from(".")
}

fn find_python() -> String {
    let candidates = [
        r"D:\Anaconda3\envs\robotarm\python.exe",
        r"python.exe",
        r"python",
        r"python3",
    ];
    for c in candidates {
        if Path::new(c).exists() {
            return c.to_string();
        }
    }
    "python".to_string()
}

fn start_backend() -> Option<Child> {
    let root = find_project_root();
    let runner = root.join("run_backend.py");
    let python = find_python();

    println!("[URDForge Tauri] Project root: {:?}", root);
    println!("[URDForge Tauri] Python executable: {}", python);
    println!("[URDForge Tauri] Backend runner: {:?}", runner);

    let mut cmd = Command::new(&python);
    if runner.exists() {
        cmd.arg(&runner);
    } else {
        cmd.arg("-m").arg("urdforge.server.run_backend");
    }
    cmd.arg("--port").arg("8765");
    cmd.current_dir(&root);

    #[cfg(target_os = "windows")]
    cmd.creation_flags(CREATE_NO_WINDOW);

    match cmd.spawn() {
        Ok(child) => {
            println!("[URDForge Tauri] Backend spawned with PID: {}", child.id());
            Some(child)
        }
        Err(e) => {
            eprintln!("[URDForge Tauri] Error spawning backend: {}", e);
            None
        }
    }
}

fn wait_for_port(port: u16, max_seconds: u64) -> bool {
    let addr = format!("127.0.0.1:{}", port);
    let start = std::time::Instant::now();
    let timeout = Duration::from_secs(max_seconds);

    while start.elapsed() < timeout {
        if let Ok(target) = addr.parse() {
            if TcpStream::connect_timeout(&target, Duration::from_millis(250)).is_ok() {
                return true;
            }
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    false
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let child_holder: Arc<Mutex<Option<Child>>> = Arc::new(Mutex::new(None));
    let child_holder_setup = Arc::clone(&child_holder);
    let child_holder_exit = Arc::clone(&child_holder);

    tauri::Builder::default()
        .setup(move |app| {
            if cfg!(debug_assertions) {
                app.handle().plugin(
                    tauri_plugin_log::Builder::default()
                        .level(log::LevelFilter::Info)
                        .build(),
                )?;
            }

            // Spawn backend child process
            if let Some(child) = start_backend() {
                if let Ok(mut lock) = child_holder_setup.lock() {
                    *lock = Some(child);
                }
            }

            // If main window exists, wait for backend then navigate
            if let Some(window) = app.get_webview_window("main") {
                let win = window.clone();
                std::thread::spawn(move || {
                    if wait_for_port(8765, 20) {
                        println!("[URDForge Tauri] Backend ready. Navigating window...");
                        if let Ok(target_url) = "http://127.0.0.1:8765/".parse() {
                            let _ = win.navigate(target_url);
                        }
                    } else {
                        eprintln!("[URDForge Tauri] Backend timed out.");
                    }
                });
            }

            Ok(())
        })
        .build(tauri::generate_context!())
        .expect("error while building tauri application")
        .run(move |_app_handle, event| {
            if let RunEvent::Exit = event {
                println!("[URDForge Tauri] Application exiting, killing backend...");
                if let Ok(mut lock) = child_holder_exit.lock() {
                    if let Some(mut child) = lock.take() {
                        let _ = child.kill();
                    }
                }
            }
        });
}
