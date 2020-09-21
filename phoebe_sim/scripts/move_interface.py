#!/usr/bin/env python  


import sys
import rospy
import tf
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg

def move_group_interface(name):
    moveit_commander.roscpp_initialize(sys.argv)
    node_name = name+"_control"
    rospy.init_node(node_name, anonymous=True, log_level=rospy.DEBUG)
    
    temp_arm = "_arm"
    group_arm = name + temp_arm
    group_gripper = name+"_gripper"
    home_pose = name+"_home"
    gripper_open_pose = name+"_open"
    gripper_close_pose = name+"_close"
    
    down_pose = name+"_down"
    
    
    tf_target = tf.TransformListener()
    tr_fram = group_arm + "_base_link"
    
    
    scene = moveit_commander.PlanningSceneInterface()
    
    
    
    arm_group =  moveit_commander.MoveGroupCommander(group_arm)
    gripper_group =moveit_commander.MoveGroupCommander(group_gripper)
    
    # arm_group.set_named_target(home_pose)
    # plan1= arm_group.plan()
    # rospy.sleep(0.5)
    # arm_group.go()
    # # print "...Home Done..."
    # rospy.logwarn("%s Home Done...", name )
    
    rospy.sleep(1)
    arm_group.set_named_target(down_pose)
    plan1= arm_group.plan()
    rospy.sleep(0.5)
    arm_group.go()
    # print "...Home Done..."
    rospy.logwarn("%s Down Done...", name )
    
    
    rate = rospy.Rate(10) # 10hz
    done = False
    while not rospy.is_shutdown():
        # print "In Loop ..."
        # print group_arm
        try:
            (trans,rot) = tf_target.lookupTransform('/base_footprint', name, rospy.Time(0))
            rospy.logwarn("%s TF Received",name )            
            
            
        
            if name == "left":
                pose_target = geometry_msgs.msg.Pose()
                pose_target.position.x = trans[0] - 0.08
                pose_target.position.y = trans[1] -0.008
                pose_target.position.z = trans[2] + 0.000
                pose_target.orientation.x = -0.7056760572625038           
                pose_target.orientation.y =  0.0019179159375093402         
                pose_target.orientation.z = -0.7085294918675162                    
                pose_target.orientation.w = 0.0018928705353824515
            else:
                pose_target = geometry_msgs.msg.Pose()
                pose_target.position.x = trans[0] - 0.08
                pose_target.position.y = trans[1] +0.008
                pose_target.position.z = trans[2] - 0.000
                pose_target.orientation.x =  -0.00029844929803324316          
                pose_target.orientation.y =  -0.6991166785936085                 
                pose_target.orientation.z =  -0.0002965459637467383                    
                pose_target.orientation.w =  -0.7150074773740129                 
            
            
            arm_group.set_pose_target(pose_target)            
            plan1 = arm_group.plan()
            
            
            if done == False:
            
                if plan1.joint_trajectory.points  :  # True if trajectory contains points                   
                    move_success = arm_group.execute(plan1, wait=True)
                    if move_success == True:
                        rospy.logwarn("%s Grasp Pose Achieved...", name )
                        gripper_group.set_named_target(gripper_open_pose)
                        plan2= gripper_group.plan()
                        if plan2.joint_trajectory.points:
                            # print "...Gripper Open Planning successful..."
                            rospy.logwarn("%s Gripper Open Planning successful...", name )
                            rospy.sleep(2)
                            move_success = gripper_group.execute(plan2, wait=True)
                            if move_success == True:
                                # print "...Gripper Opened..."
                                rospy.logwarn("%s Gripper Opened...", name )
                                rospy.sleep(2)
                                gripper_group.set_named_target(gripper_close_pose)
                                plan2= gripper_group.plan()
                                if plan2.joint_trajectory.points:
                                    # print "...Gripper Close Planning successful..."
                                    rospy.logwarn("%s Gripper close planning successful...", name )
                                    rospy.sleep(1)
                                    move_success = gripper_group.execute(plan2, wait=True)
                                    if move_success == True:
                                        # print "...Gripper Closed..."
                                        rospy.logwarn("%s Gripper Closed...", name )
                                        done = True   
                                        # print "Success: ", move_success
                else:
                    rospy.logerr(" %s Trajectory is empty. Planning was unsuccessful.", name)
                    
            else:
                print "%s Plan Execution success", name
            
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        rate.sleep()



if __name__=='__main__':
  try:
    args = rospy.myargv(argv=sys.argv)
    if len (args) > 1:
        name = args[1]
    else:
        name = "left"
    move_group_interface(name)
	
  except rospy.ROSInterruptException:
    pass