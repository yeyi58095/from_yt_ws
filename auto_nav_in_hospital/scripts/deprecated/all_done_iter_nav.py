#!/usr/bin/env python3

import subprocess
import time
import os
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

def main():
    while True:
        reserved = get_reserved_list()
        if reserved:
            next_target = reserved[0]
            nav_to(next_target)
            remove_first_reserved()
            time.sleep(1)  # 稍微停一下
        else:
            print("😴 沒有預約目的地了，回家休息")
            nav_to("home")
            break

if __name__ == "__main__":
    main()
