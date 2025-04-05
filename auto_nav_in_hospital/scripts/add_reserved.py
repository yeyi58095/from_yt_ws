#!/usr/bin/env python3

import sys
import os
import rospkg

rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path('auto_nav_in_hospital')
FILE_PATH = os.path.join(PKG_PATH, "yamls", "reserved")
EMERGENCY_PATH = os.path.join(PKG_PATH, "yamls", "reserved_emergency")

if len(sys.argv) != 2:
    print("❗️請輸入預約的目的地名稱")
    sys.exit(1)

target = sys.argv[1]

# 讀取 reserved
with open(FILE_PATH, "r", encoding="utf-8") as f:
    reserved_lines = [line.strip() for line in f.readlines() if line.strip()]

# 讀取 reserved_emergency
with open(EMERGENCY_PATH, "r", encoding="utf-8") as f:
    emergency_lines = [line.strip() for line in f.readlines() if line.strip()]

# 避免重複插入
if target in reserved_lines or target in emergency_lines:
    print(f"⚠️ 目的地 {target} 已經存在於 reserved 或 reserved_emergency 中")
    sys.exit(0)

# 加入到 reserved 結尾
reserved_lines.append(target)

with open(FILE_PATH, "w", encoding="utf-8") as f:
    for line in reserved_lines:
        f.write(line + "\n")

print(f"📌 已加入目的地：{target}")
