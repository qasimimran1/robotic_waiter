# Spawn the phoebe
roslaunch phoebe_description spawn_robot.launch

# Launch the controllers and robot state publisher
roslaunch phoebe_description phoebe_control.launch



# 1) Launch without any joint controler, to see that only fixed links are publushed in tf
# 2) launch with some controled joints, and see it only calculates those tf that link that
# 3) launch one that controles everything


# Commands to move Robot
#rostopic pub -l /pi_robot/head_tilt_joint_position_controller/command std_msgs/Float64 "data: 2.0"                
#rostopic pub -l /pi_robot/head_pan_joint_position_controller/command std_msgs/Float64 "data: 1.0"
#rostopic pub -l /phoebe/right_arm_shoulder_roll_controller/command std_msgs/Float64 "data: 2.0"

#rostopic pub -l /phoebe/right_arm_shoulder_pitch_controller/command std_msgs/Float64 "data: 2.0"
