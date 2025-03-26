#!/usr/bin/env python3

import subprocess
import time
import os
import signal

def kill_gazebo():
    """ 確保 Gazebo 被完整關閉，避免殘留 """
    print("🛑 確保 Gazebo 被關閉...")
    subprocess.call("pkill -9 -f gzserver", shell=True)
    subprocess.call("pkill -9 -f gzclient", shell=True)
try:
    print("🔧 啟動初始化 all_done_empty.launch...")
    # 啟動 all_done_empty.launch
    init_proc = subprocess.Popen(
        ["roslaunch", "auto_nav_in_hospital", "all_done_empty.launch"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    # 等待模擬與 TF 初始化
    time.sleep(10)

    print("🛑 關閉初始化 launch...")
    # 結束 all_done_empty.launch
    init_proc.send_signal(signal.SIGINT)  # 相當於 Ctrl+C
    init_proc.wait()

    # 等待一點時間確保關閉乾淨
    time.sleep(2)

    print("🚀 啟動最終 SLAM 導航 all_done_auto_nav.launch...")
    # 啟動導航任務
    subprocess.call(["roslaunch", "auto_nav_in_hospital", "start_auto_nav.launch"])

except KeyboardInterrupt:
    print("⛔ 使用者中斷，清理 Gazebo...")
    kill_gazebo()

finally:
    kill_gazebo()
    print("✅ Gazebo 已關閉，程式結束。")

