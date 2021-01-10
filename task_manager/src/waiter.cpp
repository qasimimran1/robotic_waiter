#include <task_manager/waiter.h>


bool Waiter::movePTU(PointPTU&  goal) {

  if (!_ptuClient.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact PTU server");
    return false;
  }  

  ROS_INFO("Sending goal %f %f", goal.pan, goal.tilt);

  // Build the message from PointPTU
  task_manager::PTUGoal msg;
  msg.joint_values[0] = goal.pan;    //goal.pan; 
  msg.joint_values[1] = goal.tilt;     //goal.tilt; 
  
  
  _ptuClient.sendGoal(msg);
  


  while (!_ptuClient.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }


  if (_ptuClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MovePTU failed");
    return false;
  }

  ROS_INFO("PTU Pointed");
  return true;


}