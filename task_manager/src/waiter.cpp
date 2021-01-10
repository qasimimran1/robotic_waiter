#include <task_manager/waiter.h>


// void Waiter::init(ros::NodeHandle &nh) {

//     MyNodeHandle = &nh;
    
//    // ROS_WARN("Waiter Initialized");
// }


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

void Waiter::moveAbort(void){
  _move_aborted = true;  
}


bool Waiter::moveBase(Pose2D& goal ){
  // if no server is present, fail after 2 seconds
  if (!_moveBaseClient.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact move_base server");
    return false;
  }  

  // Reset this flag
  _move_aborted = false;

  ROS_INFO("Sending goal %f %f", goal.x, goal.y);

  // Build the message from Pose2D
  move_base_msgs::MoveBaseGoal msg;
  msg.target_pose.header.frame_id = "map";
  msg.target_pose.header.stamp = ros::Time::now();
  msg.target_pose.pose.position.x = goal.x;
  msg.target_pose.pose.position.y = goal.y;
  tf::Quaternion rot = tf::createQuaternionFromYaw(goal.theta);
  tf::quaternionTFToMsg(rot, msg.target_pose.pose.orientation);

  _moveBaseClient.sendGoal(msg);

  while (!_move_aborted && !_moveBaseClient.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_move_aborted) {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    ROS_ERROR("MoveBase aborted");
    return false;
  }

  if (_moveBaseClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MoveBase failed");
    return false;
  }

  ROS_INFO("Target reached");
  return true;

}

//=======================================================

bool Waiter::detectTray(void){
    ros::NodeHandle nh;
    ros::ServiceClient _perceptionClient = nh.serviceClient<phoebe_perception::target_position>("get_position");
    phoebe_perception::target_position srv;

    ROS_INFO("Detecting Handles");
   if(_perceptionClient.call(srv)){
        _left_handle = srv.response.points[0];
        _right_handle = srv.response.points[1];
       // ROS_WARN("left_handle: %f right_handle: %f", _left_handle.x, _right_handle.x);
        return true;
    }else
    {
        ROS_ERROR("Failed to Call Service");
        return false;
    }
    
}
//=======================================================
