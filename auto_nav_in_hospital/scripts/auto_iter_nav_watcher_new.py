#!/usr/bin/env python3

import subprocess
import time
import os
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import threading
import rospkg
import signal

rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path('auto_nav_in_hospital')
FILE_PATH = os.path.join(PKG_PATH, "yamls", "reserved")

queue_lock = threading.Lock()
current_process = None
last_modified = 0

def get_reserved_list():
    with open(FILE_PATH, mode='r', encoding='utf-8') as f:
        lines = [line.strip() for line in f.readlines() if line.strip() != ""]
    return lines

def remove_first_reserved():
    with queue_lock:
        lines = get_reserved_list()
        with open(FILE_PATH, mode='w', encoding='utf-8') as f:
            f.writelines([line + '\n' for line in lines[1:]])

def nav_to(target):
    global current_process
    print(f"🚀 導航到 {target}")
    current_process = subprocess.Popen(
        ["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", target]
    )
    current_process.wait()
    print(f"✅ 已抵達 {target}")
    current_process = None

def interrupt_navigation():
    global current_process
    if current_process:
        print("⚠️ 中斷目前導航")
        current_process.send_signal(signal.SIGINT)
        current_process = None

def navigation_loop():
    while True:
        with queue_lock:
            reserved = get_reserved_list()

        if reserved:
            next_target = reserved[0]
            nav_to(next_target)
            remove_first_reserved()
            time.sleep(1)
        else:
            time.sleep(1)  # Idle
            print("😴 沒有預約目的地了，回家休息")
            nav_to("home")

            # 監聽模式
            time.sleep(1)

# --- Watchdog ---

import threading

debounce_timer = None

class ReservedHandler(FileSystemEventHandler):
    def on_modified(self, event):
        global debounce_timer

        if event.src_path.endswith("reserved"):
            # 如果已有 timer，先取消
            if debounce_timer:
                debounce_timer.cancel()

            # 設定新的 debounce timer (2秒內沒變動才觸發)
            debounce_timer = threading.Timer(2.0, self.handle_change)
            debounce_timer.start()

    def handle_change(self):
        print("📥 檢測到 reserved 穩定變更，打斷當前導航")
        interrupt_navigation()

def start_watching():
    print("👀 監聽模式啟動中...")
    event_handler = ReservedHandler()
    observer = Observer()
    observer.schedule(event_handler, path=os.path.dirname(FILE_PATH), recursive=False)
    observer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()

# --- Main ---

if __name__ == "__main__":
    nav_thread = threading.Thread(target=navigation_loop, daemon=True)
    nav_thread.start()
    start_watching()
