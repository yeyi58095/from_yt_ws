#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_marker():
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    rospy.sleep(1)  # 等待 ROS 設定完成

    marker = Marker()
    marker.header.frame_id = "map"  # 設定座標系
    marker.header.stamp = rospy.Time.now()
    marker.ns = "marker_namespace"
    marker.id = 0
    marker.type = Marker.SPHERE  # 使用球體標記
    marker.action = Marker.ADD

    # 設定標記點的座標
    marker.pose.position.x = 1.0
    marker.pose.position.y = 2.0
    marker.pose.position.z = 0.0

    # 設定標記大小
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    # 設定顏色 (RGBA)
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.lifetime = rospy.Duration(0)  # 標記不會自動消失

    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass
