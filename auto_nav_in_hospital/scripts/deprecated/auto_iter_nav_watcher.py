#!/usr/bin/env python3

import subprocess
import time
import os
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import threading
import rospkg
import sys
import select

rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path('auto_nav_in_hospital')
FILE_PATH = os.path.join(PKG_PATH, "yamls", "reserved")

def get_reserved_list():
    with open(FILE_PATH, mode='r', encoding='utf-8') as f:
        lines = [line.strip() for line in f.readlines() if line.strip() != ""]
    return lines

def remove_first_reserved():
    lines = get_reserved_list()
    with open(FILE_PATH, mode='w', encoding='utf-8') as f:
        f.writelines([line + '\n' for line in lines[1:]])

def nav_to(target):
    print(f"🚀 導航到 {target}")
    subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", target])
    print(f"✅ 已抵達 {target}")

def process_reserved():
    while True:
        reserved = get_reserved_list()
        if reserved:
            next_target = reserved[0]
            nav_to(next_target)
            remove_first_reserved()
            time.sleep(1)
        else:
            print("😴 沒有預約目的地了，回家休息")
            nav_to("home")
            break

# --- Watchdog ---

last_modified = 0

class ReservedHandler(FileSystemEventHandler):
    def on_modified(self, event):
        global last_modified
        if event.src_path.endswith("reserved"):
            now = time.time()
            # 防抖，避免重複觸發
            if now - last_modified > 1:
                print("📥 檢測到新預約，開始導航")
                last_modified = now
                process_reserved()
                print("👀 回到監聽中...")

def start_watching():
    print("👀 監聽模式啟動...")
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
    # 先檢查 reserved 檔案
    reserved = get_reserved_list()
    if reserved:
        print("📌 啟動時已有預約，開始導航...")
        process_reserved()

    # 導航結束後啟動監聽模式
    start_watching()
