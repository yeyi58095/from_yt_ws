#!/usr/bin/env python3

import sys
import os
import rospkg
import sys
import select

rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path('auto_nav_in_hospital')
FILE_PATH = os.path.join(PKG_PATH, "yamls", "reserved")
if len(sys.argv) != 2:
    print("❗️請輸入要加入的目的地名稱")
    sys.exit(1)

target = sys.argv[1]

with open(FILE_PATH, "a", encoding="utf-8") as f:
    f.write(target + "\n")
    f.write(target + "\n")

print(f"✅ 已新增 {target} 到 reserved")
