#!/usr/bin/env python3

import sys
import os
import rospkg

rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path('auto_nav_in_hospital')
EMERGENCY_PATH = os.path.join(PKG_PATH, "yamls", "reserved_emergency")
NORMAL_PATH = os.path.join(PKG_PATH, "yamls", "reserved")

if len(sys.argv) != 2:
    print("❗️請輸入緊急目的地名稱")
    sys.exit(1)

target = sys.argv[1]

# 讀取 emergency
with open(EMERGENCY_PATH, "r", encoding="utf-8") as f:
    emergency_lines = [line.strip() for line in f.readlines() if line.strip()]

# 如果已存在就跳出
if target in emergency_lines:
    print(f"⚠️ {target} 已存在於 reserved_emergency，忽略")
    sys.exit(0)

# 如果在 reserved 中，也移除它
with open(NORMAL_PATH, "r", encoding="utf-8") as f:
    normal_lines = [line.strip() for line in f.readlines() if line.strip()]

if target in normal_lines:
    normal_lines.remove(target)
    with open(NORMAL_PATH, "w", encoding="utf-8") as f:
        for line in normal_lines:
            f.write(line + "\n")
    print(f"🧹 已從 reserved 中移除 {target}")

# 加入到 reserved_emergency 的結尾
emergency_lines.append(target)
with open(EMERGENCY_PATH, "w", encoding="utf-8") as f:
    for line in emergency_lines:
        f.write(line + "\n")

print(f"🚨 已插入緊急目的地：{target}")
