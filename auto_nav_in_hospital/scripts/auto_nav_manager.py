#!/usr/bin/env python3

import rospy
import yaml
import time
import os
import threading
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from nav_controller import NavController
from actionlib_msgs.msg import GoalStatus

rospy.init_node("auto_nav_manager")

# 路徑設定
PKG_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
RESERVED_PATH = os.path.join(PKG_PATH, "yamls", "reserved")
EMERGENCY_PATH = os.path.join(PKG_PATH, "yamls", "reserved_emergency")
POSITIONS_PATH = os.path.join(PKG_PATH, "yamls", "positions.yaml")

# 全域資源
controller = NavController()
queue_lock = threading.Lock()
last_modified_reserved = 0
last_modified_emergency = 0
reserved_hash = None
emergency_hash = None
current_target = None
current_source = None  # "reserved" or "emergency"
at_home = False
was_preempted = False

# 讀取清單

def get_list(path):
    with open(path, 'r', encoding='utf-8') as f:
        return [line.strip() for line in f if line.strip()]

def get_reserved_list():
    return get_list(RESERVED_PATH)

def get_emergency_list():
    return get_list(EMERGENCY_PATH)

def get_hash(path):
    try:
        with open(path, 'rb') as f:
            return hash(f.read())
    except:
        return None

def get_reserved_hash():
    return get_hash(RESERVED_PATH)

def get_emergency_hash():
    return get_hash(EMERGENCY_PATH)

# 移除第一個

def remove_first(path):
    with queue_lock:
        lines = get_list(path)
        with open(path, 'w', encoding='utf-8') as f:
            f.writelines([line + '\n' for line in lines[1:]])

def remove_first_reserved():
    remove_first(RESERVED_PATH)

def remove_first_emergency():
    remove_first(EMERGENCY_PATH)

# 取得位置座標

def get_pose(name):
    with open(POSITIONS_PATH, 'r', encoding='utf-8') as f:
        positions = yaml.safe_load(f)
    return positions.get(name)

# 自動導航 thread

def navigation_loop():
    global current_target, current_source, at_home, was_preempted
    rate = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            with queue_lock:
                emergency = get_emergency_list()
                reserved = get_reserved_list()
                update_hashes()

            if emergency:
                target = emergency[0]
                current_source = "emergency"
            elif reserved:
                target = reserved[0]
                current_source = "reserved"
            else:
                if not at_home:
                    home_pose = get_pose("home")
                    if home_pose:
                        current_target = "home"
                        current_source = None
                        rospy.loginfo("😴 沒有任務了，回家休息")
                        controller.go_to(home_pose)
                        result = controller.wait()
                        if result == GoalStatus.SUCCEEDED:
                            rospy.loginfo("✅ 已成功到達 home")
                        elif result == GoalStatus.PREEMPTED:
                            rospy.logwarn("⚠️ 回家任務被中斷（PREEMPTED）")
                        elif result == GoalStatus.ABORTED:
                            rospy.logerr("❌ 回家任務失敗（ABORTED）")
                        current_target = None
                    at_home = True
                rate.sleep()
                continue

            at_home = False
            pose = get_pose(target)
            if pose:
                current_target = target
                rospy.loginfo(f"🚀 發送導航目標：{target}")
                controller.go_to(pose)
                result = controller.wait()

                if result == GoalStatus.PREEMPTED:
                    rospy.logwarn(f"⚠️ 導航至 {target} 被中斷（PREEMPTED）")
                elif result == GoalStatus.ABORTED:
                    rospy.logerr(f"❌ 導航至 {target} 失敗（ABORTED）")
                elif result == GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"✅ 已完成導航至 {target}")

                # 如果不是被外部中斷才移除
                if not was_preempted:
                    if current_source == "emergency":
                        remove_first_emergency()
                    elif current_source == "reserved":
                        remove_first_reserved()
                else:
                    rospy.logwarn("⚠️ 任務被中斷，不移除隊列")
                was_preempted = False
                current_target = None
            else:
                rospy.logerr(f"❌ 找不到位置: {target}")
                if current_source == "emergency":
                    remove_first_emergency()
                elif current_source == "reserved":
                    remove_first_reserved()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 收到 Ctrl+C，導航執行緒已結束")

# Watchdog

def interrupt_navigation():
    global was_preempted
    rospy.logwarn("📥 檢測到 reserved 檔案變更，嘗試取消目標")
    controller.cancel()
    was_preempted = True
    time.sleep(0.1)

def update_hashes():
    global reserved_hash, emergency_hash
    reserved_hash = get_reserved_hash()
    emergency_hash = get_emergency_hash()

class WatchHandler(FileSystemEventHandler):
    def on_modified(self, event):
        global last_modified_reserved, last_modified_emergency, current_target
        if event.src_path.endswith("reserved"):
            now = time.time()
            if now - last_modified_reserved > 1:
                last_modified_reserved = now
                new_hash = get_reserved_hash()
                if new_hash != reserved_hash:
                    reserved = get_reserved_list()
                    new_first = reserved[0] if reserved else None
                    if new_first != current_target:
                        rospy.logwarn(f"📥 第一個目標從 {current_target} 改為 {new_first}，取消導航")
                        interrupt_navigation()
                    else:
                        rospy.loginfo("📝 reserved 變動，但第一目標未改變，忽略取消")

        if event.src_path.endswith("reserved_emergency"):
            now = time.time()
            if now - last_modified_emergency > 1:
                last_modified_emergency = now
                new_hash = get_emergency_hash()
                if new_hash != emergency_hash:
                    emergency = get_emergency_list()
                    new_first = emergency[0] if emergency else None
                    if new_first != current_target:
                        rospy.logwarn(f"📥 緊急目標從 {current_target} 改為 {new_first}，取消導航")
                        interrupt_navigation()
                    else:
                        rospy.loginfo("📝 reserved_emergency 變動，但第一目標未改變，忽略取消")

def start_watch():
    observer = Observer()
    handler = WatchHandler()
    observer.schedule(handler, path=os.path.dirname(RESERVED_PATH), recursive=False)
    observer.start()
    rospy.loginfo("👀 開始監聽 reserved / reserved_emergency 檔案變化...")
    try:
        while not rospy.is_shutdown():
            time.sleep(1)
    finally:
        observer.stop()
        observer.join()

if __name__ == '__main__':
    with queue_lock:
        update_hashes()
    nav_thread = threading.Thread(target=navigation_loop, daemon=True)
    nav_thread.start()
    start_watch()
