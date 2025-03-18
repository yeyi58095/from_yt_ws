#!/usr/bin/env python3

import rospy
import rosbag

rospy.init_node("read_bag")
bag = rosbag.Bag("../bag/test.bag")

for topic, msg, t in bag.read_messages(topics="/yt"): # or topics = ["/yt", "/jenny", "/jojn"] is also available
	if topic == "/yt":
		print(msg)
		print(t)
	
