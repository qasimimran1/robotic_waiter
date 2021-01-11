#! /usr/bin/env python


import rospy
from copy import deepcopy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Point
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import (Grasp, GripperTranslation, MoveItErrorCodes, PlanningScene, ObjectColor,PlaceLocation)
from tf.transformations import quaternion_from_euler

import task_manager.msg




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
    ideal_x     = 0.515000 #0.515
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


  def pick_tray(self, res):
    self.dual_arm_set_named_target("both_home")
    
    # rospy.wait_for_service('get_position', timeout=5)
    # res = self.get_position_client()
    # left_handle = res.points[0]
    # right_handle = res.points[1]
    left_handle = res.handles[0]
    right_handle = res.handles[1]

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
        
          if(translation.x >= 0.0505 and translation.x <= 0.0535 ):
            wpose.position.x    += (translation.x + 0.000)  # move forward in (x)
            wpose_l.position.x  += (translation.x + 0.002)
            wpose_l.position.z  += 0.001
        
          else:
            wpose.position.x    += 0.0525  # move forward in (x)
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
                return move_success

    

      

  def place_tray(self):
    waypoints = []    
    wpose = self.right_arm.get_current_pose().pose
    # q = quaternion_from_euler(-1.57, 1.57, -1.57) # wrist up 5 degrees = 1.66 10deg = 1.75
    # wpose.orientation.x = q[0]
    # wpose.orientation.y = q[1]
    # wpose.orientation.z = q[2]
    # wpose.orientation.w = q[3]

    print ("wpose.position.z" ,wpose.position.z)
    wpose.position.z -= 0.0580  # move down in (z)
    print ("wpose.position.z after" ,wpose.position.z)
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

        wpose.position.x    -= 0.05  # move backward in (x)
        wpose_l.position.x  -= 0.05
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


class PickAction(object):
  # create messages that are used to publish feedback/result
  _feedback = task_manager.msg.PickFeedback()
  _result   = task_manager.msg.PickResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, task_manager.msg.PickAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    self.grasping_client = GraspingClient() 
    rospy.loginfo('Pick Action Server Started')
    
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    # success = True  

    
    # publish info to the console for the user
    rospy.loginfo('Starting Pick Action')

    res = self.grasping_client.pick_tray(goal)

    

    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
    

    rospy.loginfo('Executing Action')

      # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    r.sleep()
    
    if res:
        success = True
    else:
        success = False    
    
    if success:
        rospy.loginfo('Action Completed')
        self.set_status('SUCCESS')
    else:
        self.set_status('FAILURE')
      
#  update the status

  def set_status(self,status):
      if status == 'SUCCESS':
        self._feedback.status = 1
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
      elif status == 'FAILURE':
        self._feedback.status = 0
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Failed' % self._action_name)
        self._as.set_succeeded(self._result)
      else:
        rospy.logerr('Action %s: has a wrong return status' % self._action_name)




class PlaceAction(object):
  # create messages that are used to publish feedback/result
  _feedback = task_manager.msg.PlaceFeedback()
  _result   = task_manager.msg.PlaceResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, task_manager.msg.PlaceAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    self.grasping_client = GraspingClient() 
    rospy.loginfo('Place Action Server Started')
    
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    # success = True  

    
    # publish info to the console for the user
    rospy.loginfo('Starting Place Action')

    res = self.grasping_client.place_tray()

    

    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
    

    rospy.loginfo('Executing Action')

      # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    r.sleep()
    
    if res:
        success = True
    else:
        success = False    
    
    if success:
        rospy.loginfo('Action Completed')
        self.set_status('SUCCESS')
    else:
        self.set_status('FAILURE')
      
#  update the status

  def set_status(self,status):
      if status == 'SUCCESS':
        self._feedback.status = 1
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
      elif status == 'FAILURE':
        self._feedback.status = 0
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Failed' % self._action_name)
        self._as.set_succeeded(self._result)
      else:
        rospy.logerr('Action %s: has a wrong return status' % self._action_name)



if __name__ == '__main__':
  rospy.init_node('manipulation_servers')
  PickAction('pick_action')
  PlaceAction('place_action')
  rospy.spin()