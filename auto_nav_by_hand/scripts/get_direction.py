#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def callback(msg):
    linear = msg.linear.x
    angular = msg.angular.z

    if abs(linear) < 0.01 and abs(angular) < 0.01:
        direction = "停止"
    elif linear > 0:
        if angular > 0.1:
            direction = "左前"
        elif angular < -0.1:
            direction = "右前"
        else:
            direction = "前進"
    elif linear < 0:
        if angular > 0.1:
            direction = "左後"
        elif angular < -0.1:
            direction = "右後"
        else:
            direction = "後退"
    elif angular > 0:
        direction = "原地左轉"
    elif angular < 0:
        direction = "原地右轉"
    else:
        direction = "未知"

    rospy.loginfo("方向建議：%s (線速度=%.2f, 角速度=%.2f)", direction, linear, angular)

def listener():
    rospy.init_node('cmd_vel_listener', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
