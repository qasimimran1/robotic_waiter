#!/usr/bin/env python  

import sys
import rospy
import tf
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import moveit_commander
import moveit_msgs.msg

# (roll about an X-axis) / (subsequent pitch about the Y-axis) / (subsequent yaw about the Z-axis),
if __name__ == '__main__':
    
    # moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('tf_listener')
    # scene = moveit_commander.PlanningSceneInterface()
    
    # left_arm_group =  moveit_commander.MoveGroupCommander("left_arm")
    # left_gripper_group =  moveit_commander.MoveGroupCommander("left_gripper")
    # right_arm_group =  moveit_commander.MoveGroupCommander("right_arm")
    
    # dual_arm_group = moveit_commander.MoveGroupCommander("dual_arm")
    
    
    # left_arm_group.set_named_target("left_down")
    # plan1= left_arm_group.plan()
    # rospy.sleep(1)
    # left_arm_group.go()
    # print "...Done..."
    # rospy.sleep(1) 
    
    # left_arm_group.set_named_target("left_home")
    # plan1= left_arm_group.plan()
    # rospy.sleep(1)
    # left_arm_group.go()
    # print "...Left Done..."
    # rospy.sleep(1) 
    
    
    # right_arm_group.set_named_target("right_home")
    # plan1= right_arm_group.plan()
    # rospy.sleep(1)
    # right_arm_group.go()
    # print "...Right Done..."
    # rospy.sleep(1) 
    
    
   
   
    # dual_arm_group.set_named_target("both_down")
    # plan1= dual_arm_group.plan()
    # rospy.sleep(1)
    # dual_arm_group.go()
    # print "...Done..."
    # rospy.sleep(1)    

    tf_target_left = tf.TransformListener()     
    
    rate = rospy.Rate(10.0)
    done_l = False    
    done_r= False
    
    while not rospy.is_shutdown():
        try:
            (trans,rot) = tf_target_left.lookupTransform('/base_footprint', '/left_arm_gripper_finger1', rospy.Time(0))
            (trans_r,rot_r) = tf_target_left.lookupTransform('/base_footprint', '/right_arm_gripper_finger1', rospy.Time(0))            
            (trans_ef_inv, rot_ef_inv) = tf_target_left.lookupTransform('/left_arm_gripper_finger1', '/base_footprint', rospy.Time(0))
            (trans_ef,rot_ef) = tf_target_left.lookupTransform('/right_arm_gripper_finger1', '/base_footprint', rospy.Time(0))
            
            print " tf recieved"
            print "Left _Base->EEF",rot           
            print "Right _Base->EEF",rot_r ,"\n\n" 
            
            print "Inverted"
            print "Left _EEF->_Base",rot_ef_inv           
            print "Right _EEF->_Base",rot_ef ,"\n\n" 
            
            
            (roll, pitch, yaw) = euler_from_quaternion (rot)
            
            print "Left : roll", roll, " pitch", pitch, " yaw", yaw ,"\n\n"
            
            (roll, pitch, yaw) = euler_from_quaternion (rot_r)
            
            print "Right : roll", roll, " pitch", pitch, " yaw", yaw ,"\n\n"
            
            # print "endEffector_trans",trans_ef
            # print "endEffector_rot" ,rot_ef ,"\n\n"
            
            
            
            
            # (trans_ef_l,rot_ef_l) = tf_target_left.lookupTransform('/left_arm_gripper_finger1', '/base_footprint', rospy.Time(0))
            # (trans_ef_inv, rot_ef_inv) = tf_target_left.lookupTransform('/left', '/left_arm_gripper_finger1', rospy.Time(0))
            
            # print "Left tf recieved \n\n"
            # print ":base_footprint_trans",trans_l
            # print "base_footprint_rot" ,rot_l
            
            # (roll, pitch, yaw) = euler_from_quaternion (rot_l)
            
            # print "roll", roll, " pitch", pitch, " yaw", yaw ,"\n\n"
            
            # print "endEffector_trans",trans_ef_l
            # print "endEffector_rot" ,rot_ef_l ,"\n\n"
            
            # print "trans_ef_inv",trans_ef_inv
            # print "rot_ef_inv" ,  rot_ef_inv
#LEFT ARM endEffector_rot [0.7053606492458526, -0.0019177679470230302, 0.7088434898691529, 0.0018930218411443711] 
#LEFT ARM base_footprint_rot [-0.7053606492458526, 0.0019177679470230302, -0.7088434898691529, 0.0018930218411443711]
#roll -0.830349172764  pitch -1.56349531097  yaw -2.31127847734



# RIGHT ARM base_footprint_rot [0.0034693680188641607, -0.7079351482635765, 0.0005972904812187683, -0.7062686688383243]
#roll 1.57079265352  pitch 1.57079265359  yaw 3.14159265359       
# endEffector_rot [0.4999999999966269, -0.4999999999966269, -0.5000018366025517, 0.49999816339744835]

            # pose_target_l = geometry_msgs.msg.Pose()
            # pose_target_l.position.x = trans[0] - 0.08
            # pose_target_l.position.y = trans[1] 
            # pose_target_l.position.z = trans[2] + 0.002
        
            # pose_target_l.orientation.x = -0.7056760572625038           
            # pose_target_l.orientation.y = 0.0019179159375093402         
            # pose_target_l.orientation.z = -0.7085294918675162                    
            # pose_target_l.orientation.w = 0.0018928705353824515
            # print pose_target_l
            
            # left_arm_group.set_pose_target(pose_target_l)            
            # plan1 = left_arm_group.plan()
            
            
            # pose_target_r = geometry_msgs.msg.Pose()
            # pose_target_r.position.x = trans_r[0] - 0.05
            # pose_target_r.position.y = trans_r[1] 
            # pose_target_r.position.z = trans_r[2] 
        
            # pose_target_r.orientation.x = 0.0019208474570231449           
            # pose_target_r.orientation.y = -0.7055435901939379         
            # pose_target_r.orientation.z = 0.0018899230233428196                    
            # pose_target_r.orientation.w = -0.7086614007212909
            # print pose_target_r
            
            # right_arm_group.set_pose_target(pose_target_r)            
            # plan2 = right_arm_group.plan()
            
            # if done_l == False:
            
                # if plan1.joint_trajectory.points  :  # True if trajectory contains points                   
                    # move_success = left_arm_group.execute(plan1, wait=True)
                    # if move_success == True:
                        # left_gripper_group.set_named_target("left_open")
                        # plan2= left_gripper_group.plan()
                        # if plan2.joint_trajectory.points:
                            # print "...Gripper Open Planning successful..."
                            # rospy.sleep(20)
                            # move_success = left_gripper_group.execute(plan2, wait=True)
                            # if move_success == True:
                                # print "...Gripper Opened..."
                                # rospy.sleep(20)
                                # left_gripper_group.set_named_target("left_close")
                                # plan2= left_gripper_group.plan()
                                # if plan2.joint_trajectory.points:
                                    # print "...Gripper Close Planning successful..."
                                    # rospy.sleep(10)
                                    # move_success = left_gripper_group.execute(plan2, wait=True)
                                    # if move_success == True:
                                        # print "...Gripper Closed..."
                                        # done_l = True   
                                        # print "Success: ", move_success
                # else:
                    # rospy.logerr("Left Trajectory is empty. Planning was unsuccessful.")
                    
            # else:
                # print "Plan Execution success"
            # if done_r == False:
                # if plan2.joint_trajectory.points  :  # True if trajectory contains points                   
                    # move_success = right_arm_group.execute(plan2, wait=True)
                    # if move_success == True:
                        # done_r = True
                        # print "Success", move_success
                # else:
                    # rospy.logerr("Right Trajectory is empty. Planning was unsuccessful.")
               
                
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        rate.sleep()
