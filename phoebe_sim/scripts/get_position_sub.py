#!/usr/bin/env python  

import sys
import rospy
# import tf
# import geometry_msgs.msg
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import moveit_commander
# import moveit_msgs.msg

# from phoebe_perception.srv import target_position, target_positionResponse, target_positionRequest
from phoebe_perception.msg import target_pose


class GetHandlesPosition(object):
	"""docstring for GetHandlesPosition"""
	def __init__(self):
		# super(GetHandlesPosition, self).__init__()
		# self.arg = arg
		sub_topic = "handles_position"
		self.handles_position_sub = rospy.Subscriber(sub_topic, target_pose, self.getPositionCb)


	def getPositionCb(self, msg):
		left_handle = msg.points[0]
		right_handle = msg.points[1]

		print("left_handle:\n ", left_handle , "right_handle:\n ", right_handle)
		


def main():
	
	rospy.init_node('get_position_sub')

	get_position = GetHandlesPosition()

	
	rate = rospy.Rate(2.0)
	rospy.spin()
	


	

	

	
    


if __name__ == '__main__':
    main()
