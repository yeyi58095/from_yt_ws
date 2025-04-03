#!/usr/bin/env python3

import sys

FILE_PATH = "../yamls/reserved"

if len(sys.argv) != 2:
    print("❗️請輸入緊急目的地名稱")
    sys.exit(1)

target = sys.argv[1]

# 讀取原本 reserved
with open(FILE_PATH, "r", encoding="utf-8") as f:
    lines = [line.strip() for line in f.readlines() if line.strip() != ""]

# 把新的緊急目的地放第一個
lines = [target] + lines
lines = [target] + lines
# 寫回檔案
with open(FILE_PATH, "w", encoding="utf-8") as f:
    for line in lines:
        f.write(line + "\n")

print(f"🚨 已插入緊急目的地：{target}")
