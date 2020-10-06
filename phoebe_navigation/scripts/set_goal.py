#!/usr/bin/env python  

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import euler_from_quaternion, quaternion_from_euler


def active_cb(extra):
    rospy.loginfo("Goal Pose Being Processed")
    
def feedback_cb(feedback):
    rospy.loginfo("Current Location: "+str(feedback))
    
def done_cb(status, result):
    if status == 2 or status == 8:
        rospy.loginfo("Gaol Canclled")
    if status == 3:
        rospy.loginfo("Gaol Reached")        
    if status == 4:
        rospy.loginfo("Gaol Aborted")
        
        
rospy.init_node('set_gaol', anonymous = True)
nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
nav_client.wait_for_server()

q = quaternion_from_euler(0, 0, 1.5708)
#Example of Nav Goal
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

goal.target_pose.pose.position.x = 2.0
goal.target_pose.pose.position.y = 5.30
goal.target_pose.pose.position.z = 0.0
# goal.target_pose.pose.orientation.x = 0.0
# goal.target_pose.pose.orientation.y = 0.0
# goal.target_pose.pose.orientation.z = 0.0
# goal.target_pose.pose.orientation.w = 1.0
goal.target_pose.pose.orientation.x = q[0]
goal.target_pose.pose.orientation.y = q[1]
goal.target_pose.pose.orientation.z = q[2]
goal.target_pose.pose.orientation.w = q[3]


# nav_client.send_goal(goal, done_cb, active_cb, feedback_cb)
nav_client.send_goal(goal, done_cb)
finished = nav_client.wait_for_result()

if not finished:
    rospy.logerr("Action Server not available")
else:
    rospy.loginfo(nav_client.get_result())
