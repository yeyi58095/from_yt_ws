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
POSITIONS_PATH = os.path.join(PKG_PATH, "yamls", "positions.yaml")

# 全域資源
controller = NavController()
queue_lock = threading.Lock()
stop_flag = threading.Event()
last_modified = 0
prev_first = None
current_target = None
at_home = False
reserved_hash = None

# 讀取 reserved 清單
def get_reserved_list():
    with open(RESERVED_PATH, 'r', encoding='utf-8') as f:
        lines = [line.strip() for line in f if line.strip()]
    return lines

# 取得 reserved hash（用於偵測內容變更）
def get_reserved_hash():
    try:
        with open(RESERVED_PATH, 'rb') as f:
            return hash(f.read())
    except:
        return None

# 移除 reserved 第一行
def remove_first_reserved():
    with queue_lock:
        lines = get_reserved_list()
        with open(RESERVED_PATH, 'w', encoding='utf-8') as f:
            f.writelines([line + '\n' for line in lines[1:]])

# 取得位置座標
def get_pose(name):
    with open(POSITIONS_PATH, 'r', encoding='utf-8') as f:
        positions = yaml.safe_load(f)
    return positions.get(name)

# 自動導航 thread
def navigation_loop():
    global prev_first, at_home, current_target, reserved_hash
    rate = rospy.Rate(1)

    try:
        while not rospy.is_shutdown():
            with queue_lock:
                reserved = get_reserved_list()
                reserved_hash = get_reserved_hash()

            if reserved:
                at_home = False  # 有任務代表不在家
                current_first = reserved[0]
                pose = get_pose(current_first)
                if pose:
                    current_target = current_first
                    controller.go_to(pose)
                    result = controller.wait()

                    if result == GoalStatus.SUCCEEDED:
                        rospy.loginfo(f"✅ 已完成導航至 {current_first}")
                        remove_first_reserved()  # ✅ 僅成功才移除任務
                    elif result == GoalStatus.PREEMPTED:
                        rospy.logwarn(f"⚠️ 導航至 {current_first} 被中斷（PREEMPTED）")
                    elif result == GoalStatus.ABORTED:
                        rospy.logerr(f"❌ 導航至 {current_first} 失敗（ABORTED）")
                    else:
                        rospy.logwarn(f"⚠️ 導航至 {current_first} 結束，但狀態為 {result}")

                    with queue_lock:
                        reserved = get_reserved_list()
                        reserved_hash = get_reserved_hash()
                    prev_first = reserved[0] if reserved else None
                    current_target = None
                else:
                    rospy.logerr(f"❌ 找不到位置: {current_first}")
                    remove_first_reserved()
            else:
                if not at_home:
                    home_pose = get_pose("home")
                    if home_pose:
                        rospy.loginfo("😴 沒有任務了，回家休息")
                        current_target = "home"
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
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 收到 Ctrl+C，導航執行緒已結束")

# Watchdog 事件
def interrupt_navigation():
    global controller
    rospy.logwarn("📥 檢測到 reserved 變更，嘗試取消目標")
    controller.cancel()
    time.sleep(0.1)  # 避免 PREEMPTING race condition

class ReservedHandler(FileSystemEventHandler):
    def on_modified(self, event):
        global last_modified, prev_first, current_target, reserved_hash
        if event.src_path.endswith("reserved"):
            now = time.time()
            if now - last_modified > 1:
                last_modified = now

                new_hash = get_reserved_hash()
                if new_hash == reserved_hash:
                    return  # 內容沒變化

                with queue_lock:
                    reserved = get_reserved_list()
                current_first = reserved[0] if reserved else None

                if current_first != current_target:
                    rospy.logwarn(f"📥 第一個目標從 {current_target} 改為 {current_first}，取消導航")
                    interrupt_navigation()
                    prev_first = current_first
                else:
                    rospy.loginfo("📝 reserved 變動，但第一目標未改變，忽略取消")

# 啟動 watchdog
def start_watching():
    observer = Observer()
    observer.schedule(ReservedHandler(), path=os.path.dirname(RESERVED_PATH), recursive=False)
    observer.start()
    rospy.loginfo("👀 開始監聽 reserved 檔案變化...")
    try:
        while not rospy.is_shutdown():
            time.sleep(1)
    finally:
        observer.stop()
        observer.join()

if __name__ == '__main__':
    with queue_lock:
        reserved = get_reserved_list()
        reserved_hash = get_reserved_hash()
    prev_first = reserved[0] if reserved else None

    nav_thread = threading.Thread(target=navigation_loop, daemon=True)
    nav_thread.start()
    start_watching()
