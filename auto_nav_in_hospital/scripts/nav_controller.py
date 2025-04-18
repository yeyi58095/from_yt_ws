#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class NavController:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("已連接到 move_base")

    def go_to(self, pose):  # pose = dict with x, y, z, qx, qy, qz, qw
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = pose["x"]
        goal.target_pose.pose.position.y = pose["y"]
        goal.target_pose.pose.position.z = pose["z"]

        goal.target_pose.pose.orientation.x = pose["qx"]
        goal.target_pose.pose.orientation.y = pose["qy"]
        goal.target_pose.pose.orientation.z = pose["qz"]
        goal.target_pose.pose.orientation.w = pose["qw"]

        self.client.send_goal(goal)
        rospy.loginfo("🚀 發送導航目標")

    def cancel(self):
        self.client.cancel_goal()
        rospy.logwarn("⚠️ 已取消當前目標")

    ###
    def wait(self):
        self.client.wait_for_result()
        state = self.client.get_state()
        return state
    ###
    
  

    def wait(self):
        self.client.wait_for_result()
        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("✅ 已成功到達目標！")
        elif state == GoalStatus.PREEMPTED:
            rospy.logwarn("⚠️ 導航已被中斷 (PREEMPTED)")
        elif state == GoalStatus.ABORTED:
            rospy.logerr("❌ 導航失敗 (ABORTED)")
        else:
            rospy.logwarn(f"⚠️ 未知狀態結束，state={state}")
        return state
