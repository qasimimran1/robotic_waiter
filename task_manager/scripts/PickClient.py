#! /usr/bin/env python


import rospy

import actionlib
import task_manager.msg
from phoebe_perception.srv import target_position, target_positionResponse, target_positionRequest



def pick_client():

    get_position_client = rospy.ServiceProxy('get_position', target_position)

    
    rospy.loginfo("Waiting for target_position Server")
    rospy.wait_for_service('get_position', timeout=5)
    res = get_position_client()


    print("handles", res)

    # Creates the SimpleActionClient, passing the type of the action
    
    client = actionlib.SimpleActionClient('pick_action', task_manager.msg.PickAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for PickAction Server")
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = task_manager.msg.PickGoal([res.points[0],res.points[1]])

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult
    # except rospy.ROSException:
    #   rospy.logerr('get_position_server did not respond in 15 sec')
    #   return   

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('pick_client')
        result = pick_client()
        
        rospy.loginfo("Server Respose: %d",result.status)
        
        
        
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")