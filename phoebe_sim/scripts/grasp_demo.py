#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import tf


from std_msgs.msg import String

def move_group_interface():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface', anonymous=True)
    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    


    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    dual_arm_group = moveit_commander.MoveGroupCommander("dual_arm")
    
    # left_arm_group =  moveit_commander.MoveGroupCommander("left_arm")
    # print "============ Reference frame: %s" % left_arm_group.get_planning_frame()
    # print "============End Effector: %s" % left_arm_group.get_end_effector_link()
    # left_gripper_group =  moveit_commander.MoveGroupCommander("left_gripper")
    # right_arm_group =  moveit_commander.MoveGroupCommander("right_arm")


    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    dual_arm_group.set_named_target("both_down")
    plan1= dual_arm_group.plan()
    rospy.sleep(0.5)
    dual_arm_group.go()
    # print "...Home..."
    # rospy.sleep(5)
    
    
    
    # dual_arm_group.set_named_target("both_grab3")
    # plan1= dual_arm_group.plan()
    # rospy.sleep(0.5)
    # dual_arm_group.go()
    # print "...Waiting to lift..."
    # rospy.sleep(20)
    
    
    
    # dual_arm_group.set_named_target("both_lift")
    # plan1= dual_arm_group.plan()
    # rospy.sleep(0.5)
    # dual_arm_group.go()
    # print "...Home..."
    # rospy.sleep(1)
    
   
    # right_arm_group.set_named_target("right_grab")    
    # plan2= right_arm_group.plan()
    # rospy.sleep(1)
    # left_arm_group.go()
    # right_arm_group.go()
    # print "...1st Pose Done..."    
    # rospy.sleep(1)
    # grippers_group = moveit_commander.MoveGroupCommander("grippers")
    # for i in range(2):
    
        # grippers_group.set_named_target("both_open")
        # plan2 = grippers_group.plan()
        # rospy.sleep(1)
        # grippers_group.go()
        # print "...Both Open..."  
        # rospy.sleep(2)
        
        # grippers_group.set_named_target("both_close")
        # plan2 = grippers_group.plan()
        # rospy.sleep(1)
        # grippers_group.go()
        # print "...Both Close..."
        # rospy.sleep(2)
    
    
    # roll = 1.57
    # pitch = 0
    # yaw = 0
    # q = quaternion_from_euler(roll, pitch, yaw)
    
    # goal.position.x = 0.555;
    # goal.position.y = 0.146;
    # goal.position.z = 1.156;
    # goal.orientation.w = 0.991428;
    # goal.orientation.x = 0.0085588;
    # goal.orientation.y = -0.087665;
    # goal.orientation.z = -0.0964952;
    
    
    # left_arm_group.set_named_target("random valid")    
    # plan1= left_arm_group.plan()
    # rospy.sleep(6)
    # left_arm_group.go(wait=True)
    
     # position: 
      # x: 0.0250186168009
      # y: 0.487010565504
      # z: 0.470970706717
    # orientation: 
      # x: -0.521156274313
      # y: -0.398770697363
      # z: -0.287207590558
      # w: 0.69777494122
    
    # pose_target = geometry_msgs.msg.Pose()    
    
    # pose_target.position.x = 0.0250186168009
    # pose_target.position.y =  0.487010565504
    # pose_target.position.z =  0.470970706717
    # pose_target.orientation.w = 0.69777494122
    # pose_target.orientation.x =-0.521156274313
    # pose_target.orientation.y = -0.398770697363
    # pose_target.orientation.z =-0.287207590558
    
    
    # left_arm_group.set_pose_target(pose_target)
    # plan1 = left_arm_group.plan()
    
    # rospy.sleep(6)
    # left_arm_group.go(wait=True)
    


    print "...Done..."
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    move_group_interface()
  except rospy.ROSInterruptException:
    pass