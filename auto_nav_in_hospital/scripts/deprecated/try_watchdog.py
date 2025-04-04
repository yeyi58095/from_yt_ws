#!/usr/bin/env python3

import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os
import os
import rospkg
import rospy
import sys
import select

rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path('auto_nav_in_hospital')
FILE_PATH = os.path.join(PKG_PATH, "yamls", "reserved")
last_modified = 0

class MyHandler(FileSystemEventHandler):
    def on_modified(self, event):
        global last_modified
        now = time.time()
        # 防止 1 秒內連續觸發
        if now - last_modified > 1:
            print(f"🚀 檔案 {event.src_path} 被修改")
            last_modified = now

if __name__ == '__main__':
    event_handler = MyHandler()
    observer = Observer()
    observer.schedule(event_handler, path=FILE_PATH, recursive=False)
    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()

    observer.join()

