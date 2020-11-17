#!/usr/bin/env python

import rospy, sys
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
import copy
from copy import deepcopy

from phoebe_perception.srv import target_position, target_positionResponse, target_positionRequest



def get_dual_trajectory(r_joints):  #this method takes trajectory of righ arm and maps it for left arm inverting the mirror joints
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


def get_translations(current_pos):
	ideal_x  		= 0.515000#0.515
	ideal_y 		= 0.250500
	ideal_z 		= 0.713000 #0.7262

	trans 			= geometry_msgs.msg.Point()
	trans.x 		= current_pos.x - ideal_x
	trans.y 		= current_pos.y + ideal_y
	trans.z 		= current_pos.z - ideal_z   

	return trans


def main():
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('get_position_client')

	scene = moveit_commander.PlanningSceneInterface()

	objects = scene.get_known_object_names()

	# print("objects", objects)

	# print("scene", scene)
	left_arm =  moveit_commander.MoveGroupCommander("left_arm")
	# left_gripper_group =  moveit_commander.MoveGroupCommander("left_gripper")

	right_arm =  moveit_commander.MoveGroupCommander("right_arm")
	# right_gripper =  moveit_commander.MoveGroupCommander("right_gripper")


	dual_arm_group = moveit_commander.MoveGroupCommander("dual_arm")

	grippers =  moveit_commander.MoveGroupCommander("grippers")

	get_position = rospy.ServiceProxy('get_position', target_position)
	rate = rospy.Rate(2.0)
	done_l = False
	done_r= False

	# dual_joints = dual_arm_group.get_joints()

	# print("dual_joints" , dual_joints)

	# dual_joint_values = dual_arm_group.get_current_joint_values()

	# print("dual_joint_values", dual_joint_values)

	# left_arm_group.set_named_target("left_home")
	# plan1 = left_arm_group.plan()
	# rospy.sleep(2)
	# left_arm_group.go()
	# print "...Left Done..."
	# rospy.sleep(1)

	dual_arm_group.set_named_target("both_home")
	plan1 = dual_arm_group.plan()
	rospy.sleep(2)
	# print("dual_arm plan1", plan1)
	dual_arm_group.go()
	print "...Both Done..."
	rospy.sleep(1)
	# print("scene", scene)

	while not rospy.is_shutdown():
		try:
			rospy.wait_for_service('get_position', timeout=5)
			res = get_position()
			left_handle = res.points[0]
			right_handle = res.points[1]


			# scene.remove_world_object()

			# print "right_handle" , right_handle

			# pose_target_l = []
			# pose_target_l.append(( left_handle.x - 0.03))
			# pose_target_l.append(left_handle.y)
			# pose_target_l.append(left_handle.z)


			# pose_target_r = []
			# pose_target_r.append(right_handle.x)
			# pose_target_r.append(right_handle.y)
			# pose_target_r.append(right_handle.z)

			# print(pose_target_l)

			print('left_handle', left_handle, 'right_handle', right_handle)
			# left_arm_group.set_position_target(pose_target_l)

			pose_target_l = geometry_msgs.msg.Pose()
			pose_target_r = geometry_msgs.msg.Pose()

			# pose_target_l.position = left_handle;
			# pose_target_l.position.x -= 0.06
			# pose_target_l.position.y -= 0.005
			# pose_target_l.position.z -= 0.005
			# q = quaternion_from_euler(-1.57, 0, -1.57) # Horizental Gripper : paraller to x-axis
			# # q = quaternion_from_euler(-2.432, -1.57, -0.709)  # Vertical Gripper: Vertical to x-axis
			# pose_target_l.orientation.x = q[0]
			# pose_target_l.orientation.y = q[1]
			# pose_target_l.orientation.z = q[2]
			# pose_target_l.orientation.w = q[3]
			# print ("pose_target_l",pose_target_l )


			# left_arm_group.set_pose_target(pose_target_l)
			# left_arm_group.set_planning_time(10)
			# plan1 = left_arm_group.plan()
			# rospy.sleep(2)
			# left_arm_group.go()
			# done_l = True



			# pose_target_r.position.x = 0.5232
			# pose_target_r.position.y = -0.2743
			# pose_target_r.position.z = 0.6846


			# right_gripper.set_named_target("right_open")
			# right_gripper.plan()
			# right_gripper.go()




			pose_target_r.position = right_handle;
			translation = get_translations(right_handle)
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


			right_arm.set_pose_target(pose_target_r)
			right_arm.set_planning_time(10)
			plan1 = right_arm.plan()
			# rospy.sleep(1)

			# right_arm.go()
			if done_r == False :
				# print ("target pose",pose_target_r)
				if (plan1.joint_trajectory.points) :  # True if trajectory contains points
					# last = (len(plan1.joint_trajectory.points) - 1)
					# print("joint_trajectory", (plan1.joint_trajectory.points[-1].positions) ) # getting the last position of trajectory
					r_joints = plan1.joint_trajectory.points[-1].positions					
					d_joints = get_dual_trajectory(r_joints)

					# print("l_joints", l_joints)
					# d_joints = l_joints + list(r_joints)

					# print ("d_joints", d_joints)
					dual_arm_group.set_joint_value_target(d_joints)
					plan2 = dual_arm_group.plan()

					if (plan2.joint_trajectory.points) :
						move_success = dual_arm_group.execute(plan2, wait = True)

						# eef_pose = right_arm.get_current_pose()
						# print("eef_pose", eef_pose)
						


					# move_success = right_arm.execute(plan1, wait = True)
						if move_success == True:
							rospy.sleep(2)
							right_arm.set_start_state_to_current_state()
							left_arm.set_start_state_to_current_state()
							waypoints_l =[]
							waypoints = []
							wpose = right_arm.get_current_pose().pose
							wpose_l = left_arm.get_current_pose().pose

							print("wpose", wpose)
							# Open the gripper to the full position
							grippers.set_named_target("both_open")
							grippers.plan()
							grippers.go()
							# Create Cartesian Path to move forward mainting the end-effector pose
							# waypoints = []
							# rospy.sleep(5)
							# wpose = right_arm.get_current_pose().pose
							if(translation.x >= 0.0505):
								wpose.position.x 		+= (translation.x + 0.00)  # move forward in (x)
								wpose_l.position.x 	+= (translation.x + 0.002)
								wpose_l.position.z 	+= 0.001
							
							else:
								wpose.position.x 		+= 0.051  # move forward in (x)
								wpose_l.position.x 	+= 0.053
								wpose_l.position.z 	+= 0.001

							waypoints.append(copy.deepcopy(wpose))
							(plan1, fraction) = right_arm.compute_cartesian_path(
	                                   waypoints,   # waypoints to follow
	                                   0.01,        # eef_step
	                                   0.0)


							waypoints_l.append(copy.deepcopy(wpose_l))
							(plan_l, fraction) = left_arm.compute_cartesian_path(
	                                   waypoints_l,   # waypoints to follow
	                                   0.01,        # eef_step
	                                   0.0)
	          	
							# print("plan1 len", len(plan1.joint_trajectory.points))
							# waypoints.append(copy.deepcopy(wpose))
							# (plan2, fraction) = dual_arm_group.compute_cartesian_path(
	      #                              waypoints,   # waypoints to follow
	      #                              0.005,        # eef_step
	      #                              0.0)
							rospy.sleep(1)
							if plan1.joint_trajectory.points:
								move_success = right_arm.execute(plan1, wait=True)
								if move_success == True:
									rospy.loginfo ("Right Move forward successful")
									# done_r = True
									# break;

							if plan_l.joint_trajectory.points:
								move_success_l = left_arm.execute(plan_l, wait=True)
								if move_success_l == True:
									rospy.loginfo ("Left Move forward successful")
									# done_r = True

							if move_success == True and move_success_l == True:
								rospy.sleep(1)
								grippers.set_named_target("both_close")
								grippers.plan()
								grippers.go()
								# rospy.sleep(1)

								waypoints = []
								rospy.sleep(1)
								wpose = right_arm.get_current_pose().pose
								# q = quaternion_from_euler(-1.57, 1.57, -1.57) # wrist up 5 degrees = 1.66 10deg = 1.75
								# wpose.orientation.x = q[0]
								# wpose.orientation.y = q[1]
								# wpose.orientation.z = q[2]
								# wpose.orientation.w = q[3]
								wpose.position.z += 0.07  # move up in (z)
								waypoints.append(copy.deepcopy(wpose))
								(plan1, fraction) = right_arm.compute_cartesian_path(
		                               waypoints,   # waypoints to follow
		                               0.01,        # eef_step
		                               0.0)

								if plan1.joint_trajectory.points:

									# move_success = right_arm.execute(plan1, wait=True) # for testing right arm trajectory
									# done_r = True



									r_joints = plan1.joint_trajectory.points[-1].positions
									# print ("r_joints", r_joints)					
									d_joints = get_dual_trajectory(r_joints)
									# print ("d_joints", d_joints)
									dual_arm_group.set_joint_value_target(d_joints)
									# plan2 = 0
									plan2 = dual_arm_group.plan()
									# print("planed plan len", len(plan2.joint_trajectory.points))
									# print("dual arm trajectory", (plan2.joint_trajectory.points))
									# done_r = True

									if (plan2.joint_trajectory.points) :

										move_success = dual_arm_group.go()
										print("move_success", move_success)
										done_r = True

										# traj_points = plan2.joint_trajectory.points[0:-3]
										# plan2.joint_trajectory.points = traj_points
										# print("eddited plan len", len(plan2.joint_trajectory.points))
										# move_success = dual_arm_group.execute(plan2, wait = True)
										# # done_r = True
										# move_success = right_arm.execute(plan1, wait=True)
										# if move_success == True:
										# 	rospy.loginfo ("Lift Successful")
										# 	done_r = True
											

								# r_joints = plan1.joint_trajectory.points[-1].positions					
								# d_joints = get_dual_trajectory(r_joints)
								# # print("l_joints", l_joints)
								# # d_joints = l_joints + list(r_joints)
								# # print ("d_joints", d_joints)
								# dual_arm_group.set_joint_value_target(d_joints)
								# plan2 = dual_arm_group.plan()

								# if (plan2.joint_trajectory.points) :
								# 	move_success = dual_arm_group.execute(plan2, wait = True)

								# 	if move_success == True:
								# 		rospy.loginfo("Move forward successful")
								# 		rospy.sleep(1)
								# 		grippers.set_named_target("both_close")
								# 		grippers.plan()
								# 		grippers.go()
								# 		rospy.sleep(1)

								# 		waypoints = []
								# 		rospy.sleep(2)
								# 		wpose = right_arm.get_current_pose().pose
								# 		wpose.position.z += 0.1  # move up in (z)
								# 		waypoints.append(copy.deepcopy(wpose))
								# 		(plan1, fraction) = right_arm.compute_cartesian_path(
				    #                            waypoints,   # waypoints to follow
				    #                            0.01,        # eef_step
				    #                            0.0)

								# 		if plan1.joint_trajectory.points:

								# 			r_joints = plan1.joint_trajectory.points[-1].positions					
								# 			d_joints = get_dual_trajectory(r_joints)
								# 			# print ("d_joints", d_joints)
								# 			dual_arm_group.set_joint_value_target(d_joints)
								# 			plan2 = dual_arm_group.plan()

								# 			if (plan2.joint_trajectory.points) :
								# 				move_success = dual_arm_group.execute(plan2, wait = True)
								# 				done_r = True
					# 				move_success = right_arm.execute(plan1, wait=True)
					# 				if move_success == True:
					# 					rospy.loginfo ("Lift Successful")
					# 					done_r = True
										
					# 	else:
					# 		rospy.logwarn("Cartesian Paht Planning Failed for forward movement")

          	


			else :
				print ("Execution Completed")





		except rospy.ROSException:
			rospy.logerr('get_position_server did not respond in 5 sec')
			return

		rate.sleep()



if __name__ == '__main__':
    main()
