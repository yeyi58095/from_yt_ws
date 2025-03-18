#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64


def topic_callback(msg): #裡面的參數不限，就自己設，因為我這裡只有收到一個參數，就只有一個
	print(msg)
	
#一樣是指明說他是Node
rospy.init_node('subscriber_node')
rospy.Subscriber('topic', Int64, topic_callback) # 第一個參數是我要訂閱的topic，第二個則是資料型別，最後一個是callback (回應所訂閱到的內容，應該吧)

# Spinning
rospy.spin()
