#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <task_manager/PTUAction.h>

typedef actionlib::SimpleActionClient<task_manager::PTUAction> PTUClient;
    

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_ptu_client");
  PTUClient _client("ptu_action", true);
  bool _aborted = false;

  ROS_INFO("Waiting for action server to start.");

  // if no server is present, fail after 5 seconds
  if (!_client.waitForServer(ros::Duration(5.0))) {
    ROS_ERROR("Can't contact PTU server");
    return 1;
  }


  // Build the message from PointPTU
  task_manager::PTUGoal msg;
  // msg.joint_values[0] = goal.pan; 
  // msg.joint_values[1] = goal.tilt; 
  msg.joint_values[0] = 0.5; 
  msg.joint_values[1] = -0.2; 

  // float msg[2];
  // msg[0] = goal.pan;
  // msg[2] = goal.tilt;
  
  _client.sendGoal(msg);
  


  while (!_aborted && !_client.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_aborted) {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    ROS_ERROR("MovePTU aborted");
    
  }

  if (_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MovePTU failed");
    
  }

  ROS_INFO("PTU Pointed");
  
  return 0; 
} 