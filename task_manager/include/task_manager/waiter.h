#ifndef WAITER_H
#define WITER_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <tf/transform_datatypes.h>
#include <task_manager/PTUAction.h>
#include <task_manager/PickAction.h>
#include <task_manager/PlaceAction.h>
#include <phoebe_perception/target_position.h>
#include <geometry_msgs/Twist.h>





typedef actionlib::SimpleActionClient<task_manager::PTUAction> PTUClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<task_manager::PickAction> PickClient;
typedef actionlib::SimpleActionClient<task_manager::PlaceAction> PlaceClient;
// Custom types
struct PointPTU
{
    double pan, tilt;
};
struct Pose2D
{
    double x, y, theta;
};

class Waiter
{

public:
    Waiter(/* args */):
           _ptuClient("ptu_action", true),
           _pickClient("pick_action", true),
           _placeClient("place_action", true),
           _moveBaseClient("move_base", true)
    {

    }
    // void init(ros::NodeHandle &nh);
    bool movePTU(PointPTU&  goal);
    bool moveBase(Pose2D& goal );
    bool pickTray(void);
    bool placeTray(void);
    void moveAbort(void);
    bool detectTray(void);
    bool moveRobot(double distance, ros::Duration duration, int direction); // 1 = forward, -1 = backward
    
    
    

private:
    // ros::NodeHandle* MyNodeHandle;
    
    PTUClient _ptuClient;
    PickClient _pickClient;
    PlaceClient _placeClient;
    MoveBaseClient _moveBaseClient;    
    bool _move_aborted ;
    geometry_msgs::Point _leftHandle, _rightHandle;
    

};



#endif