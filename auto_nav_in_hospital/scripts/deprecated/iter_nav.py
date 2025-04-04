#!/usr/bin/env python3

import subprocess
import time
import os
import signal
import rospkg
import sys
import select

rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path('auto_nav_in_hospital')
FILE_PATH = os.path.join(PKG_PATH, "yamls", "reserved")

with open(FILE_PATH, mode='r', encoding='utf-8') as f:
	line = f.readline()

	print(line)
	subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", line.strip()])
	f.close()

	
with open(FILE_PATH, mode='r', encoding='utf-8')  as f:
	line = f.readlines()

	try:
		line = line[1:]
		f = open(FILE_PATH, mode = 'w', encoding='utf-8')
		f.writelines(line)
		f.close()

	except:
		pass
