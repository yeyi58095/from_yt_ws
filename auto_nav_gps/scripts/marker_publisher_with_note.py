#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

def publish_marker():
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    rospy.sleep(1)  # 等待 ROS 設定完成

    # 建立點標記
    point_marker = Marker()
    point_marker.header.frame_id = "map"  # 座標系
    point_marker.header.stamp = rospy.Time.now()
    point_marker.ns = "marker_namespace"
    point_marker.id = 0
    point_marker.type = Marker.SPHERE  # 球形標記
    point_marker.action = Marker.ADD

    # 設定標記點的座標
    point_marker.pose.position.x = -1.1212360939791228
    point_marker.pose.position.y =  -0.4984277704808151
    point_marker.pose.position.z = 0.0

    # 設定標記大小
    point_marker.scale.x = 0.2
    point_marker.scale.y = 0.2
    point_marker.scale.z = 0.2

    # 設定顏色 (紅色)
    point_marker.color.r = 1.0
    point_marker.color.g = 0.0
    point_marker.color.b = 0.0
    point_marker.color.a = 1.0

    point_marker.lifetime = rospy.Duration(0)  # 標記不會自動消失

    # **建立名稱標記**
    text_marker = Marker()
    text_marker.header.frame_id = "map"
    text_marker.header.stamp = rospy.Time.now()
    text_marker.ns = "marker_namespace"
    text_marker.id = 1  # ID 不可重複
    text_marker.type = Marker.TEXT_VIEW_FACING  # 文字標記
    text_marker.action = Marker.ADD

    # 文字標記的位置 (放在點標記上方一點)
    text_marker.pose.position.x = point_marker.pose.position.x
    text_marker.pose.position.y = point_marker.pose.position.y
    text_marker.pose.position.z = 0.3  # 抬高一點，避免重疊

    # 設定文字大小
    text_marker.scale.z = 0.2  # 文字大小

    # 設定顏色 (白色)
    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0

    text_marker.text = "My Point"  # 文字內容

    while not rospy.is_shutdown():
        marker_pub.publish(point_marker)  # 發布點標記
        marker_pub.publish(text_marker)   # 發布文字標記
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass
