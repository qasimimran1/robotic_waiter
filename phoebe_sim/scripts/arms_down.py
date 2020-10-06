#!/usr/bin/env python

import sys
import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

def arms_down():
    pubL = rospy.Publisher('/left_arm_controller/command', JointTrajectory, queue_size = 1)
    rospy.init_node('arms_down', anonymous = True)
   
    rate = rospy.Rate(2)
    pubR = rospy.Publisher('/right_arm_controller/command', JointTrajectory, queue_size =1)
    
    left_traj = JointTrajectory()
    left_traj.joint_names = ["left_arm_elbow_pitch_joint", 'left_arm_elbow_yaw_joint', 'left_arm_shoulder_pitch_joint', 'left_arm_shoulder_roll_joint', 'left_arm_shoulder_yaw_joint', 'left_arm_wrist_pitch_joint','left_arm_wrist_roll_joint']

    left_traj_points = JointTrajectoryPoint()
    left_traj_points.positions = [0.0, 0.0, -1.57, -1.5, 0.0, 0.0, 0.0]
    left_traj_points.time_from_start = rospy.Duration.from_sec(0.5)
    left_traj.points.append(left_traj_points)
    
    right_traj = JointTrajectory()
    right_traj.joint_names = ["right_arm_elbow_pitch_joint", 'right_arm_elbow_yaw_joint', 'right_arm_shoulder_pitch_joint', 'right_arm_shoulder_roll_joint', 'right_arm_shoulder_yaw_joint', 'right_arm_wrist_pitch_joint','right_arm_wrist_roll_joint']

    right_traj_points = JointTrajectoryPoint()
    right_traj_points.positions = [0.0, 0.0, -1.57, 1.5, 0.0, 0.0, 0.0]
    right_traj_points.time_from_start = rospy.Duration.from_sec(1)
    right_traj.points.append(right_traj_points)
    rospy.sleep(8)
    
    i = 0
    while i < 10:
        left_traj_points.time_from_start = rospy.Duration.from_sec(1+i*0.1)    
        pubL.publish(left_traj)
        rate.sleep()
        right_traj_points.time_from_start = rospy.Duration.from_sec(1+i*0.15)
        pubR.publish(right_traj)        
        i +=1
        rospy.loginfo("sent goal: %d", i)
    
    # rospy.sleep(1)
    # left_traj_points.positions = [0.0, 0.0, -1.57, -1.5, 0.0, 0.0, 0.0]
    # left_traj_points.time_from_start = rospy.Duration.from_sec(0.2)
    # left_traj.points.append(left_traj_points)
    
    # right_traj_points.positions = [0.0, 0.0, -1.57, 1.5, 0.0, 0.0, 0.0]
    # right_traj_points.time_from_start = rospy.Duration.from_sec(0.2)
    # right_traj.points.append(right_traj_points)
    
    # i = 0
    # while i < 10:
        # left_traj_points.time_from_start = rospy.Duration.from_sec(0.1)    
        # pubL.publish(left_traj)
        # rate.sleep()
        # right_traj_points.time_from_start = rospy.Duration.from_sec(0.1)
        # pubR.publish(right_traj)
        # i +=1



if __name__=='__main__':
  try:
   arms_down()
  except rospy.ROSInterruptException:
    pass