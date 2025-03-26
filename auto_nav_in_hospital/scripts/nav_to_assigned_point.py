#!/usr/bin/env python3
import rospy
import yaml
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

FILE_PATH = "../yamls/positions.yaml"

def load_positions():
    if os.path.exists(FILE_PATH):
        with open(FILE_PATH, 'r') as file:
            return yaml.safe_load(file) or {}
    return {}

def navigate_to_position(target_name):
    rospy.init_node('navigate_to_position')
    positions = load_positions()

    if target_name not in positions:
        rospy.logerr(f"位置 '{target_name}' 不存在")
        return

    pos = positions[target_name]

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pos['x']
    goal.target_pose.pose.position.y = pos['y']
    goal.target_pose.pose.position.z = pos['z']
    goal.target_pose.pose.orientation.x = pos['qx']
    goal.target_pose.pose.orientation.y = pos['qy']
    goal.target_pose.pose.orientation.z = pos['qz']
    goal.target_pose.pose.orientation.w = pos['qw']

    rospy.loginfo(f"導航到 {target_name}...")
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("到達目標點！")

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("使用方法: rosrun your_package navigate_to_position.py <標記名稱>")
    else:
        navigate_to_position(sys.argv[1])
