#!/usr/bin/env python3
import rospy
import yaml
from visualization_msgs.msg import Marker
import os

FILE_PATH = "../yamls/positions.yaml"

def load_positions():
    if os.path.exists(FILE_PATH):
        with open(FILE_PATH, 'r') as file:
            return yaml.safe_load(file) or {}
    return {}

def publish_markers():
    rospy.init_node('display_saved_positions', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 每秒更新 1 次

    while not rospy.is_shutdown():
        positions = load_positions()

        for i, (name, pos) in enumerate(positions.items()):
            # 點標記
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "positions"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pos['x']
            marker.pose.position.y = pos['y']
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_pub.publish(marker)

            # 文字標記
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "position_names"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pos['x']
            text_marker.pose.position.y = pos['y']
            text_marker.pose.position.z = 0.3
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = name
            marker_pub.publish(text_marker)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_markers()
    except rospy.ROSInterruptException:
        pass
