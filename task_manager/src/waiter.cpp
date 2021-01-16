#include <task_manager/waiter.h>

//=======================================================
// void Waiter::init(ros::NodeHandle &nh) {

//     MyNodeHandle = &nh;
    
//    // ROS_WARN("Waiter Initialized");
// }

//=======================================================
bool Waiter::movePTU(PointPTU&  goal) {
  if (!_ptuClient.waitForServer(ros::Duration(10.0))) {
    ROS_ERROR("Can't contact PTU server");
    return false;
  }
  ROS_INFO("Sending goal %f %f", goal.pan, goal.tilt);

  // Build the message from PointPTU
  task_manager::PTUGoal msg;
  msg.joint_values[0] = goal.pan;    //goal.pan; 
  msg.joint_values[1] = goal.tilt;     //goal.tilt;  
  
  _ptuClient.sendGoal(msg);

  while (ros::ok() &&(!_ptuClient.waitForResult(ros::Duration(0.02)))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }
  if (_ptuClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MovePTU failed");
    return false;
  }

  ROS_INFO("PTU Pointed");
  return true;
}
//=======================================================
bool Waiter::detectTray(void){
    ros::NodeHandle nh;
    ros::ServiceClient _perceptionClient = nh.serviceClient<phoebe_perception::target_position>("get_position");
    phoebe_perception::target_position srv;

    ROS_INFO("Detecting Handles");
   if(_perceptionClient.call(srv)){
        _leftHandle = srv.response.points[0];
        _rightHandle = srv.response.points[1];
       // ROS_WARN("left_handle: %f right_handle: %f", _left_handle.x, _right_handle.x);
        return true;
    }else
    {
        ROS_ERROR("Failed to Call Service");
        return false;
    }    
}
//=======================================================
bool Waiter::pickTray(void){

    if (!_pickClient.waitForServer(ros::Duration(10.0))) {
    ROS_ERROR("Can't contact Pick server");
    return false;
  }
  //  lf.x =  0.57 #1.778
  //   lf.y =  0.222 #5.95
  //   lf.z = 0.6905
    
  //   rt.x =  0.57 #2.222
  //   rt.y = -0.222  #5.95
  //   rt.z = 0.6905
  _leftHandle.x = 0.515;
  _leftHandle.y = 0.2505;
  _leftHandle.z = 0.713;

  _rightHandle.x =  0.515;
  _rightHandle.y = -0.24;
  _rightHandle.z =  0.69;

  // Build the message from PickGoal
  task_manager::PickGoal msg;
  msg.handles[0] = _leftHandle;    //left; 
  msg.handles[1] = _rightHandle;     //right;
  
  _pickClient.sendGoal(msg);
  while (ros::ok() && (!_pickClient.waitForResult(ros::Duration(0.02)))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }
  if (_pickClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("Tray Pick failed");
    return false;
  }
  ROS_INFO("Tray Picked");
  return true;
}
//=======================================================
void Waiter::moveAbort(void){
  _move_aborted = true;  
}
//=======================================================
bool Waiter::moveBase(Pose2D& goal ){
  // if no server is present, fail after 2 seconds
  if (!_moveBaseClient.waitForServer(ros::Duration(10.0))) {
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

  while (ros::ok() &&(!_move_aborted && !_moveBaseClient.waitForResult(ros::Duration(0.02)))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_move_aborted) {
    // this happens only if method halt() was invoked
    _moveBaseClient.cancelAllGoals();
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
bool Waiter::placeTray(void){
    if (!_placeClient.waitForServer(ros::Duration(10.0))) {
    ROS_ERROR("Can't contact Pick server");
    return false;  } 

    task_manager::PlaceGoal msg;
    msg.dummy = 1;    //dummy data for goal; 
  
    _placeClient.sendGoal(msg); // placeAction needs no goal for now, so sending dummy data

    while (!_placeClient.waitForResult(ros::Duration(0.02))) {
        // polling at 50 Hz. No big deal in terms of CPU
    }

    if (_placeClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR("Tray Pick failed");
        return false;
    }

    ROS_INFO("Tray Picked");
    return true;
}
//=======================================================

// bool  Waiter::moveBackwards(double distance, ros::Duration duration) {

//     ros::NodeHandle nh;
//     // Advertize the publisher on the topic
//     ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/wheels_controller/cmd_vel",1000);
//     ros::Duration(0.1).sleep(); // sleep for half a second
//     geometry_msgs::Twist twist;
//     twist.linear.x = -(distance/duration.toSec());

//     ros::Time beginTime = ros::Time::now();
//     ros::Duration secondsIWantToSendMessagesFor = duration;
//     ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
//     while (ros::Time::now() < endTime ) {
//         pub.publish(twist);
//         ros::Duration(0.1).sleep();
//     }
//     return true;
// }

//=======================================================

bool Waiter::moveRobot(double distance, ros::Duration duration, int direction){ // dir 1 = forward, -1 = backward

    ros::NodeHandle nh;
    // Advertize the publisher on the topic
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/wheels_controller/cmd_vel",1000);
    ros::Duration(0.1).sleep(); // sleep for half a second
    geometry_msgs::Twist twist;
    twist.linear.x = direction*(distance/duration.toSec());

    ros::Time beginTime = ros::Time::now();
    ros::Duration secondsIWantToSendMessagesFor = duration;
    ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
    while (ros::Time::now() < endTime ) {
        pub.publish(twist);
        ros::Duration(0.1).sleep();
    }
    return true;
}