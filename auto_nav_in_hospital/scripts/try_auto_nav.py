#!/usr/bin/env python3

import subprocess
import time
import os
import signal

print("Start Nav")
subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", "home"])
subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", "1st"])
subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", "2nd"])
subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", "3rd"])
subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", "4th"])
subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", "5th"])
subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", "6th"])
subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", "7th"])
subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", "8th"])
subprocess.call(["rosrun", "auto_nav_in_hospital", "nav_to_assigned_point.py", "9th"])
