
#include <task_manager/waiter.h>


using namespace std; 




//============================================
BT::NodeStatus lookAtTray(void) {

  PointPTU goal;
  Waiter waiter;
  goal.pan = 0.0;
  goal.tilt = -0.7;
  if (waiter.movePTU(goal))
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  } 
  
}
//============================================
BT::NodeStatus lookUp(void) {

  PointPTU goal;
  Waiter waiter;
  goal.pan = 0.0;
  goal.tilt = 0.0;
  
  if (waiter.movePTU(goal))
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
//============================================

BT::NodeStatus detectHandles(void){

  //Waiter waiter; 

  // if (waiter.detectTray())
  // {
  //   return BT::NodeStatus::SUCCESS;
  // }
  // else
  // {
  //   return BT::NodeStatus::FAILURE;
  // }
  ros::Duration(1.0).sleep();
  return BT::NodeStatus::SUCCESS;  

}


//===================================================
BT::NodeStatus goToTable(void){
  Pose2D goal;
  Waiter waiter;
  goal.x = -3.0;
  goal.y = -7.43; //8.45
  goal.theta = -1.5708;
  if(waiter.moveBase(goal)){
    if (waiter.moveRobot(1.0, (ros::Duration)2.0, 1))
    {
      return BT::NodeStatus::SUCCESS;
    }    
    
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
  
}

//==================================================

BT::NodeStatus pickTray(void){

  Waiter waiter;
  
  if(waiter.pickTray()){
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }


}


//==================================================
BT::NodeStatus placeTray(void){

  Waiter waiter;
  
  if(waiter.placeTray()){
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }


}
//==================================================

BT::NodeStatus moveBack(void) {
  Waiter waiter;
 
  if(waiter.moveRobot(1.0, (ros::Duration)2.0, -1)){
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

//==================================================
// void RegisterNodes(BT::BehaviorTreeFactory& factory, Waiter& waiter) {
//     factory.registerSimpleAction("lookAtTheTray", 
//                                  std::bind(&Waiter::LookAt, &waiter));

// }

// BT::NodeStatus MoveBase::tick() {
//   // if no server is present, fail after 2 seconds
//   if (!_client.waitForServer(ros::Duration(2.0))) {
//     ROS_ERROR("Can't contact move_base server");
//     return BT::NodeStatus::FAILURE;
//   }

//   // Take the goal from the InputPort of the Node
//   Pose2D goal;
//   if (!getInput<Pose2D>("goal", goal)) {
//     // if I can't get this, there is something wrong with your BT.
//     // For this reason throw an exception instead of returning FAILURE
//     throw BT::RuntimeError("missing required input [goal]");
//   }

//   // Reset this flag
//   _aborted = false;

//   ROS_INFO("Sending goal %f %f", goal.x, goal.y);

//   // Build the message from Pose2D
//   move_base_msgs::MoveBaseGoal msg;
//   msg.target_pose.header.frame_id = "map";
//   msg.target_pose.header.stamp = ros::Time::now();
//   msg.target_pose.pose.position.x = goal.x;
//   msg.target_pose.pose.position.y = goal.y;
//   tf::Quaternion rot = tf::createQuaternionFromYaw(goal.theta);
//   tf::quaternionTFToMsg(rot, msg.target_pose.pose.orientation);

//   _client.sendGoal(msg);

//   while (!_aborted && !_client.waitForResult(ros::Duration(0.02))) {
//     // polling at 50 Hz. No big deal in terms of CPU
//   }

//   if (_aborted) {
//     // this happens only if method halt() was invoked
//     //_client.cancelAllGoals();
//     ROS_ERROR("MoveBase aborted");
//     return BT::NodeStatus::FAILURE;
//   }

//   if (_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
//     ROS_ERROR("MoveBase failed");
//     return BT::NodeStatus::FAILURE;
//   }

//   ROS_INFO("Target reached");
//   return BT::NodeStatus::SUCCESS;
// }


// //==================================================================

// BT::NodeStatus MovePTU::tick() {
//   // if no server is present, fail after 2 seconds
//   if (!_client.waitForServer(ros::Duration(2.0))) {
//     ROS_ERROR("Can't contact PTU server");
//     return BT::NodeStatus::FAILURE;
//   }

//   // Take the goal from the InputPort of the Node
//   PointPTU goal;
//   if (!getInput<PointPTU>("ptu_goal", goal)) {
//     // if I can't get this, there is something wrong with your BT.
//     // For this reason throw an exception instead of returning FAILURE
//     throw BT::RuntimeError("missing required input [goal]");
//   }

//   // Reset this flag
//   _aborted = false;

//   ROS_INFO("Sending goal %f %f", goal.pan, goal.tilt);

//   // Build the message from PointPTU
//   task_manager::PTUGoal msg;
//   msg.joint_values[0] = goal.pan; 
//   msg.joint_values[1] = goal.tilt; 
  
  
//   _client.sendGoal(msg);
  


//   while (!_aborted && !_client.waitForResult(ros::Duration(0.02))) {
//     // polling at 50 Hz. No big deal in terms of CPU
//   }

//   if (_aborted) {
//     // this happens only if method halt() was invoked
//     //_client.cancelAllGoals();
//     ROS_ERROR("MovePTU aborted");
//     return BT::NodeStatus::FAILURE;
//   }

//   if (_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
//     ROS_ERROR("MovePTU failed");
//     return BT::NodeStatus::FAILURE;
//   }

//   ROS_INFO("PTU Pointed");
//   return BT::NodeStatus::SUCCESS;
// }







