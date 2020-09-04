#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


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


    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    
    dual_arm_group.set_named_target("both_down")
    plan1= dual_arm_group.plan()
    rospy.sleep(2)
    dual_arm_group.go()
    print "...Both Down..."
    rospy.sleep(5)
    
    # dual_arm_group.set_named_target("grab_try")    
    # plan1= dual_arm_group.plan()
    # rospy.sleep(2)
    # dual_arm_group.go()
    # print "...Grab Try..."    
    # rospy.sleep(2)
    
    # dual_arm_group.set_named_target("home")
    # plan2= dual_arm_group.plan()    
    # rospy.sleep(5)
    # dual_arm_group.go()
    # print "...home..."
    # rospy.sleep(2)
    
    


    print "...Done..."
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    move_group_interface()
  except rospy.ROSInterruptException:
    pass
