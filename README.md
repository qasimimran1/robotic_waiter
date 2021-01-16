# robotic_waiter
Robotic Waiter Simulation


# To Compile, go to catkin_ws and run the following command
catkin_make

# To run, use following commands in separate terminals

roscore
roslaunch phoebe_sim sim_with_planning.launch
roslaunch task_manager servers.launch
rosrun task_manager tas_manager_node
roslaunch phoebe_navigation phoebe_navigation.launch

# To see the behavior tree open "Groot" in the "Monitor Mode" and connect to the server 