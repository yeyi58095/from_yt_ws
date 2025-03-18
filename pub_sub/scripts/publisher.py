#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64  #可以從這裡(/opt/ros/noetic/share/std_msgs/msg)看到想要的資料型別 

# Node + Topic initialization
# 一開始，我們要says it as a node 因此
rospy.init_node('publisher_node') #此參數是此node的名稱
pub = rospy.Publisher("topic", Int64,queue_size = 1) # 第一個參數是 topic，第二個參數是資料的型別，第三個則是buffer的大小。

while not rospy.is_shutdown():
	pub.publish(1)  # publish whether you want, Not only 1
	rospy.sleep(1)
