#!/usr/bin/env python3

import sys
import os

FILE_PATH = os.path.join(os.path.dirname(__file__), "../yamls/reserved")

if len(sys.argv) != 2:
    print("❗️請輸入緊急目的地名稱")
    sys.exit(1)

target = sys.argv[1]

# 讀取原本 reserved
with open(FILE_PATH, "r", encoding="utf-8") as f:
    lines = [line.strip() for line in f.readlines() if line.strip()]

# 只在不重複時插入
if not lines or lines[0] != target:
    lines = [target] + [line for line in lines if line != target]

    with open(FILE_PATH, "w", encoding="utf-8") as f:
        for line in lines:
            f.write(line + "\n")

    # 觸發 watchdog：更新時間戳
    os.utime(FILE_PATH, None)

    print(f"🚨 已插入緊急目的地：{target}")
else:
    print(f"⚠️ {target} 已經是當前目的地，不需重複插入")
