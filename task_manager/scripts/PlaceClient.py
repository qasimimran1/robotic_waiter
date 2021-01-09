#! /usr/bin/env python


import rospy

import actionlib
import task_manager.msg
from phoebe_perception.srv import target_position, target_positionResponse, target_positionRequest



def place_client():
    
    # Creates the SimpleActionClient, passing the type of the action    
    client = actionlib.SimpleActionClient('place_action', task_manager.msg.PlaceAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for PlaceAction Server")
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = task_manager.msg.PlaceGoal(1)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult
     

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('place_client')
        result = place_client()
        
        rospy.loginfo("Server Respose: %d",result.status)
        
        
        
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")