#!/usr/bin/env python

import sys
import copy
import rospy
from gazebo_msgs.msg import ModelStates, LinkStates
from math import sin, cos, pi

import geometry_msgs.msg
import std_msgs.msg

def get_model_state(data):
    
    phoebe_index = 0
    tray_index = 0    
    names = data.name
    
    for i in range(len(names)):
        if(names[i] == 'tray'):
            tray_index = i
    
        if(names[i] == 'phoebe'):
            phoebe_index = i
    
    
    phoebe_pose = data.pose[phoebe_index]
    tray_pose = data.pose[tray_index]
    
    phoebe_twist = data.twist[phoebe_index]
    tray_twist = data.twist[tray_index]
    
    print tray_pose, tray_twist, phoebe_pose, phoebe_twist
   
    
    
    
# def get_link_state(data):
    # rospy.loginfo("link state msg subscribed")    



def gazebo_interface():
    rospy.init_node('gazebo_interface', anonymous=True)
    
    rospy.Subscriber('/gazebo/model_states', ModelStates, get_model_state)
    
    # rospy.Subscriber('/gazebo/link_states', LinkStates , get_link_state)
    
    rospy.spin()





if __name__ == '__main__':
    try:
        gazebo_interface()
    except rospy.ROSInterruptException:
        pass