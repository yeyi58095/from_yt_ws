#!/usr/bin/env python3

import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import rospkg
import rospy
import sys
import select
import os


rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path('auto_nav_in_hospital')
FILE_PATH = os.path.join(PKG_PATH, "yamls", "reserved")


class MyHandler(FileSystemEventHandler):
    def on_modified(self, event):
        print(f"file{event.src_path} be modified")
        
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
    
