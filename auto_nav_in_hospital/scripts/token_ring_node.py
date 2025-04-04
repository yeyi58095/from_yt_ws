#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import subprocess
import time
import threading
import os
import rospkg

rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path('auto_nav_in_hospital')
FILE_PATH = os.path.join(PKG_PATH, "yamls", "reserved")

token_holder = None
robot_name = "robot1"  # 之後多台時改掉
token_pub = None
lock = threading.Lock()

def get_reserved_list():
    with open(FILE_PATH, "r", encoding="utf-8") as f:
        lines = [line.strip() for line in f.readlines() if line.strip() != ""]
    return lines

def remove_first_reserved():
    with lock:
        lines = get_reserved_list()
        with open(FILE_PATH, "w", encoding="utf-8") as f:
            f.writelines([line + '\n' for line in lines[1:]])

def nav_to(target):
    rospy.loginfo(f"🚀 導航到 {target}")
    subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", target])
    rospy.loginfo(f"✅ 已抵達 {target}")

def token_callback(msg):
    global token_holder
    if msg.data == robot_name:
        rospy.loginfo(f"📥 收到 token，檢查任務中...")
        with lock:
            reserved = get_reserved_list()
        if reserved:
            next_target = reserved[0]
            nav_to(next_target)
            remove_first_reserved()
        else:
            rospy.loginfo("😴 沒有任務，等待中...")
        # 任務結束或沒有任務 → 把 token 丟回自己 (單機器人測試)
        time.sleep(1)
        token_pub.publish(robot_name)

if __name__ == "__main__":
    rospy.init_node("token_ring_node")
    token_pub = rospy.Publisher("/token_ring/token", String, queue_size=10)
    rospy.Subscriber("/token_ring/token", String, token_callback)
    time.sleep(1)
    rospy.loginfo("👀 Token Ring 模式啟動")
    # 一開始就發 token 給自己
    token_pub.publish(robot_name)
    rospy.spin()
