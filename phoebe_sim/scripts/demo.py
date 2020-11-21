#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson
# Author: Di Sun

import rospy, sys
from copy import deepcopy
import actionlib


from math import sin, cos


from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
# from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose, Point
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

from moveit_msgs.msg import (Grasp, GripperTranslation, MoveItErrorCodes, PlanningScene, ObjectColor,PlaceLocation)

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

from phoebe_perception.srv import target_position, target_positionResponse, target_positionRequest


# Move base using navigation stack
class MoveBaseClient(object):

  def __init__(self):
    self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base...")
    self.client.wait_for_server()

  def goto(self, x, y, theta, frame="map"):
    move_goal = MoveBaseGoal()
    move_goal.target_pose.pose.position.x = x
    move_goal.target_pose.pose.position.y = y
    move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
    move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
    move_goal.target_pose.header.frame_id = frame
    move_goal.target_pose.header.stamp = rospy.Time.now()

    # TODO wait for things to work
    self.client.send_goal(move_goal)
    self.client.wait_for_result()

# Send a trajectory to controller
class FollowTrajectoryClient(object):

  def __init__(self, name, joint_names):
    self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                               FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for %s..." % name)
    self.client.wait_for_server()
    self.joint_names = joint_names

  def move_to(self, positions, duration=5.0):
    if len(self.joint_names) != len(positions):
        print("Invalid trajectory position")
        return False
    trajectory = JointTrajectory()
    trajectory.joint_names = self.joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = positions
    trajectory.points[0].velocities = [0.0 for _ in positions]
    trajectory.points[0].accelerations = [0.0 for _ in positions]
    trajectory.points[0].time_from_start = rospy.Duration(duration)
    follow_goal = FollowJointTrajectoryGoal()
    follow_goal.trajectory = trajectory

    self.client.send_goal(follow_goal)
    self.client.wait_for_result()

# Tools for grasping
class GraspingClient(object):

  def __init__(self):
    self.scene = PlanningSceneInterface()
    # self.pickplace = PickPlaceInterface("right_arm", "right_gripper", verbose=True, plan_only = False)
    # self.move_group = MoveGroupInterface("right_arm", "base_footprint", plan_only = False)

    self.left_arm       =  MoveGroupCommander("left_arm")
    # rospy.sleep(6.0) 
    self.right_arm      =  MoveGroupCommander("right_arm")
    # rospy.sleep(6.0)
    self.dual_arm_group = MoveGroupCommander("dual_arm")
    # rospy.sleep(6.0)
    self.grippers       =  MoveGroupCommander("grippers")

    self.get_position_client = rospy.ServiceProxy('get_position', target_position)
    # self.done_r = False


  def get_dual_trajectory(self, r_joints):  #this method takes trajectory of righ arm and maps it for left arm inverting the mirror joints
    l_joints = []
    pos = 1
    for i in r_joints:
      if pos % 2== 0:
        l_joints.append(i)
      else:
        l_joints.append(-i)

      pos += 1

    d_joints = l_joints + list(r_joints)
      
    return d_joints   


  def get_translations(self, current_pos):
    ideal_x     = 0.515000#0.515
    ideal_y     = 0.250500
    ideal_z     = 0.713000 #0.7262

    trans       = Point()
    trans.x     = current_pos.x - ideal_x
    trans.y     = current_pos.y + ideal_y
    trans.z     = current_pos.z - ideal_z
    
    return trans

  def dual_arm_set_named_target(self, named_target):
    self.dual_arm_group.set_named_target(named_target)
    plan1 = self.dual_arm_group.plan()
    rospy.sleep(2)
    # print("dual_arm plan1", plan1)
    move_success = self.dual_arm_group.go()
    return move_success
    # print "...Both Done..."
    # rospy.sleep(1)


  def pick_tray(self):
    self.dual_arm_set_named_target("both_home")
    try:
      rospy.wait_for_service('get_position', timeout=5)
      res = self.get_position_client()
      left_handle = res.points[0]
      right_handle = res.points[1]

      print('left_handle', left_handle, 'right_handle', right_handle)
      # left_arm_group.set_position_target(pose_target_l)

      pose_target_l = Pose()
      pose_target_r = Pose()

      pose_target_r.position = right_handle;
      translation = self.get_translations(right_handle)
      # pose_target_r.position.x -= 0.065
      pose_target_r.position.x -= translation.x

      print("translation.x", translation.x)
      pose_target_r.position.y -= translation.y
      pose_target_r.position.z -= translation.z
      # pose_target_r.position.z = 0.7235
      


      # q = quaternion_from_euler(-1.57, 0, -1.57) # This works from 2.35 : Gripper paraller to x-axis
      q = quaternion_from_euler(-1.57, 1.57, -1.57) # This work from 2.35 : Gripper Vertical to x-axis   (-2.36, 1.5708, -2.36)
      pose_target_r.orientation.x = q[0]
      pose_target_r.orientation.y = q[1]
      pose_target_r.orientation.z = q[2]
      pose_target_r.orientation.w = q[3]
      
      # print ("pose_target_r",pose_target_r )


      self.right_arm.set_pose_target(pose_target_r)
      self.right_arm.set_planning_time(10)
      plan1 = self.right_arm.plan()


      # if self.done_r == False :        
      if (plan1.joint_trajectory.points) :  # True if trajectory contains points          
        r_joints = plan1.joint_trajectory.points[-1].positions          
        d_joints = self.get_dual_trajectory(r_joints)

        self.dual_arm_group.set_joint_value_target(d_joints)
        plan2 = self.dual_arm_group.plan()

        if (plan2.joint_trajectory.points) :
          move_success = self.dual_arm_group.execute(plan2, wait = True)

          if move_success == True:
            rospy.sleep(2)
            self.right_arm.set_start_state_to_current_state()
            self.left_arm.set_start_state_to_current_state()
            waypoints_l =[]
            waypoints = []
            wpose = self.right_arm.get_current_pose().pose
            wpose_l = self.left_arm.get_current_pose().pose
            print("wpose", wpose)
            # Open the gripper to the full position
            self.grippers.set_named_target("both_open")
            self.grippers.plan()
            self.grippers.go()
            # Create Cartesian Path to move forward mainting the end-effector pose
            
            if(translation.x >= 0.0505):
              wpose.position.x    += (translation.x + 0.000)  # move forward in (x)
              wpose_l.position.x  += (translation.x + 0.002)
              wpose_l.position.z  += 0.001
            
            else:
              wpose.position.x    += 0.052  # move forward in (x)
              wpose_l.position.x  += 0.054
              wpose_l.position.z  += 0.001

            waypoints.append(deepcopy(wpose))
            (plan1, fraction) = self.right_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)


            waypoints_l.append(deepcopy(wpose_l))
            (plan_l, fraction) = self.left_arm.compute_cartesian_path(
                                   waypoints_l,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)
            
           
            rospy.sleep(1)
            if plan1.joint_trajectory.points:
              move_success = self.right_arm.execute(plan1, wait=True)
              if move_success == True:
                rospy.loginfo ("Right Move forward successful")
                

            if plan_l.joint_trajectory.points:
              move_success_l = self.left_arm.execute(plan_l, wait=True)
              if move_success_l == True:
                rospy.loginfo ("Left Move forward successful")
                # done_r = True

            if move_success == True and move_success_l == True:
              rospy.sleep(1)
              self.grippers.set_named_target("both_close")
              self.grippers.plan()
              self.grippers.go()
              # rospy.sleep(1)

              waypoints = []
              rospy.sleep(1)
              wpose = self.right_arm.get_current_pose().pose
              # q = quaternion_from_euler(-1.57, 1.57, -1.57) # wrist up 5 degrees = 1.66 10deg = 1.75
              # wpose.orientation.x = q[0]
              # wpose.orientation.y = q[1]
              # wpose.orientation.z = q[2]
              # wpose.orientation.w = q[3]
              wpose.position.z += 0.075  # move up in (z)
              waypoints.append(deepcopy(wpose))
              (plan1, fraction) = self.right_arm.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.01,        # eef_step
                                 0.0)

              if plan1.joint_trajectory.points:

                r_joints = plan1.joint_trajectory.points[-1].positions                          
                d_joints = self.get_dual_trajectory(r_joints)                
                self.dual_arm_group.set_joint_value_target(d_joints)                
                plan2 = self.dual_arm_group.plan()
                if (plan2.joint_trajectory.points) :

                  move_success = self.dual_arm_group.go()
                  print("move_success", move_success)

    except rospy.ROSException:
      rospy.logerr('get_position_server did not respond in 5 sec')
      return

    return move_success  

  def place_tray(self):
    waypoints = []    
    wpose = self.right_arm.get_current_pose().pose
    # q = quaternion_from_euler(-1.57, 1.57, -1.57) # wrist up 5 degrees = 1.66 10deg = 1.75
    # wpose.orientation.x = q[0]
    # wpose.orientation.y = q[1]
    # wpose.orientation.z = q[2]
    # wpose.orientation.w = q[3]
    wpose.position.z -= 0.072  # move up in (z)
    waypoints.append(deepcopy(wpose))
    (plan1, fraction) = self.right_arm.compute_cartesian_path(
                       waypoints,   # waypoints to follow
                       0.01,        # eef_step
                       0.0)

    if plan1.joint_trajectory.points:

      r_joints = plan1.joint_trajectory.points[-1].positions                          
      d_joints = self.get_dual_trajectory(r_joints)                
      self.dual_arm_group.set_joint_value_target(d_joints)                
      plan2 = self.dual_arm_group.plan()
      if (plan2.joint_trajectory.points) :

        move_success = self.dual_arm_group.go()
        print("move_success", move_success)

        self.grippers.set_named_target("both_open")
        self.grippers.plan()
        self.grippers.go()

        waypoints_l =[]
        waypoints   = []
        wpose   = self.right_arm.get_current_pose().pose
        wpose_l = self.left_arm.get_current_pose().pose

        wpose.position.x    -= 0.051  # move backward in (x)
        wpose_l.position.x  -= 0.053
        # wpose_l.position.z  += 0.001

        waypoints.append(deepcopy(wpose))
        (plan1, fraction) = self.right_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)


        waypoints_l.append(deepcopy(wpose_l))
        (plan_l, fraction) = self.left_arm.compute_cartesian_path(
                               waypoints_l,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)
              
             
        rospy.sleep(1)
        if plan1.joint_trajectory.points:
          move_success = self.right_arm.execute(plan1, wait=True)
          if move_success == True:
            rospy.loginfo ("Right Move forward successful")
            

        if plan_l.joint_trajectory.points:
          move_success_l = self.left_arm.execute(plan_l, wait=True)
          if move_success_l == True:
            rospy.loginfo ("Left Move forward successful")
                  # done_r = True  
        
        move_success = self.dual_arm_set_named_target("both_down")
        

    return move_success     


    

if __name__ == "__main__":
  # Create a node
  rospy.init_node("demo")

  # Make sure sim time is working
  while not rospy.Time.now():
    pass

  # Setup clients
  move_base = MoveBaseClient()
  rospy.sleep(1)
  ptu_action = FollowTrajectoryClient("ptu_controller", ["ptu_pan_joint", "ptu_tilt_joint"])  
  rospy.sleep(5.0)

  grasping_client = GraspingClient()
  rospy.sleep(1)
  # grasping_client.dual_arm_set_named_target("both_down")

  ns = rospy.get_namespace()
  print("namespace", ns)

  rospy.sleep(1)
  

  ptu_action.move_to([0.0, -0.02,])

  # move_base.goto(2.0, 5.35, 1.5708)




  
  #Move arms down
  # grasping_client.stow()

  # Move the base to be in front of the table
  # Demonstrates the use of the navigation stack
  # rospy.loginfo("Moving to table...")    
  # move_base.goto(2.0, 5.2, 1.5708)
  #move_base.goto(2.750, 3.118, 0.0)

  # Raise the torso using just a controller
  #rospy.loginfo("Raising torso...")
  #torso_action.move_to([0.4, ])

  # Point the head at the cube we want to pick
  # head_action.look_at(3.7, 3.18, 0.0, "map")
  ptu_action.move_to([0.0, -0.70,])
  # tray_lift = grasping_client.pick_tray()
  # if tray_lift:
  #   rospy.sleep(1)
  #   tray_place = grasping_client.place_tray()
  #   print("tray_place", tray_place)
  
  obj_in_gripper = False
  

  while not rospy.is_shutdown():
    rospy.sleep(1)

    # ptu_action.move_to(0.0, -0.75,  "base_footprint")

    # Get block to pick
    # fail_ct = 0
    # while not rospy.is_shutdown() and not obj_in_gripper:
    #   rospy.loginfo("Picking object...")
    #   grasping_client.updateScene()
    #   cube, grasps = grasping_client.getGraspableObject()
    #   if cube == None:
    #     rospy.logwarn("Perception failed.")
    #     # grasping_client.intermediate_stow()
    #     # grasping_client.stow()
    #     # head_action.look_at(1.2, 0.0, 0.0, "base_link")
    #     continue

    #   # Pick the block
    #   if grasping_client.pick(cube, grasps):
    #     obj_in_gripper = True
    #     break
    #   rospy.logwarn("Grasping failed.")
    #   # grasping_client.stow()
    #   if fail_ct > 15:
    #     fail_ct = 0
    #     break
    #   fail_ct += 1

      # Tuck the arm
      #grasping_client.tuck()

      # Lower torso
      #rospy.loginfo("Lowering torso...")
      #torso_action.move_to([0.0, ])

      # Move to second table
      #rospy.loginfo("Moving to second table...")
      #move_base.goto(-3.53, 3.75, 1.57)
      #move_base.goto(-3.53, 4.15, 1.57)

      # Raise the torso using just a controller
      #rospy.loginfo("Raising torso...")
      #torso_action.move_to([0.4, ])

      # Place the block
      # while not rospy.is_shutdown() and obj_in_gripper:
          # rospy.loginfo("Placing object...")
          # pose = PoseStamped()
          # pose.pose = cube.primitive_poses[0]
          # pose.pose.position.y *= -1.0
          # pose.pose.position.z += 0.02
          # pose.header.frame_id = cube.header.frame_id
          # if grasping_client.place(cube, pose):
              # obj_in_gripper = False
              # break
          # rospy.logwarn("Placing failed.")
          # grasping_client.intermediate_stow()
          # grasping_client.stow()
          # if fail_ct > 15:
              # fail_ct = 0
              # break
          # fail_ct += 1
      # Tuck the arm, lower the torso
      # grasping_client.intermediate_stow()
      # grasping_client.stow()
      # rospy.loginfo("Finished")
      #torso_action.move_to([0.0, ])