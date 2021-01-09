#!/usr/bin/env python



import rospy
from gazebo_msgs.msg import ModelStates, LinkStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0.0
y = 0.0 
theta = 0



def get_model_state(data):
    
    phoebe_index = 0
    # tray_index = 0    
    names = data.name
    
    for i in range(len(names)):
        # if(names[i] == 'tray'):
        #     tray_index = i
    
        if(names[i] == 'phoebe'):
            phoebe_index = i
    
    
    phoebe_pose = data.pose[phoebe_index]
    # tray_pose = data.pose[tray_index]

    global x
    global y
    global theta

    x = phoebe_pose.position.x
    y = phoebe_pose.position.y
    
    # print"x: y:",x ,y

    rot_q = phoebe_pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    # print"theta: ", theta
    
    # print "phoebe_pose" ,phoebe_pose phoebe_twis



rospy.init_node("speed_controller")

sub = rospy.Subscriber('/gazebo/model_states', ModelStates, get_model_state) #rospy.Subscriber("/wheels_controller/odom", Odometry, newOdom)
pub = rospy.Publisher("/wheels_controller/cmd_vel", Twist, queue_size = 1)

speed = Twist()

rate = rospy.Rate(10)

goal = Point()
goal.x = -1.5
goal.y = 0.0
done = False

while not rospy.is_shutdown() or done == False:
    inc_x = goal.x -x
    inc_y = goal.y -y
    print"x: y:",x ,y
    print"theta: ", theta

    if abs(inc_x) <= 0.005 and abs(inc_y) <= 0.005:
        done = True
    else:
        angle_to_goal = atan2(inc_y, inc_x)
        round(angle_to_goal, 4)
        print "angle_to_goal_before" , angle_to_goal
        if abs(angle_to_goal) < 0.1 or (angle_to_goal >= -3.16 and angle_to_goal <= -3.12):
            angle_to_goal = 0
        print "angle_to_goal" , angle_to_goal

        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.10
        else:
            speed.linear.x = 0.125
            speed.angular.z = 0.0

        pub.publish(speed)
    rate.sleep()    