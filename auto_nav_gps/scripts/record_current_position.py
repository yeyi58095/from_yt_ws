#!/usr/bin/env python3

import rospy
import yaml
import os
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
import argparse

FILE_PATH = "/home/daniel/from_yt_ws/src/auto_nav/yamls/positions.yaml"

def save_position(name, position):
    """ 儲存座標到 YAML 檔案 """
    if os.path.exists(FILE_PATH):
        with open(FILE_PATH, 'r') as file:
            positions = yaml.safe_load(file) or {}
    else:
        positions = {}

    # 存入新座標
    positions[name] = {
        'x': position[0],
        'y': position[1],
        'z': position[2],
        'qx': position[3],
        'qy': position[4],
        'qz': position[5],
        'qw': position[6]
    }

    # 寫回 YAML
    with open(FILE_PATH, 'w') as file:
        yaml.dump(positions, file, default_flow_style=False)

def callback(data, args):
    """ 取得當前座標並存入 YAML """
    name = args.name
    pose = data.pose.pose

    # 轉換 quaternion
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    position = (pose.position.x, pose.position.y, pose.position.z, *quaternion)
    save_position(name, position)
    
    rospy.loginfo(f"已儲存位置: {name} -> {position}")

    # 儲存後關閉節點
    rospy.signal_shutdown("Position recorded.")

def main():
    rospy.init_node('record_current_position')
    parser = argparse.ArgumentParser(description="記錄當前座標")
    parser.add_argument("-name", type=str, required=True, help="標記名稱")
    args = parser.parse_args()

    # 訂閱一次 `/amcl_pose`，記錄後自動關閉
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback, callback_args=args)
    
    rospy.loginfo("等待當前座標...")
    rospy.spin()

if __name__ == '__main__':
    main()
