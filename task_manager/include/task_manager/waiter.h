#ifndef WAITER_H
#define WITER_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <tf/transform_datatypes.h>
#include <task_manager/PTUAction.h>
#include <phoebe_perception/target_position.h>




typedef actionlib::SimpleActionClient<task_manager::PTUAction> PTUClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
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
           _moveBaseClient("move_base", true)
    {

    }
    // void init(ros::NodeHandle &nh);
    bool movePTU(PointPTU&  goal);
    bool moveBase(Pose2D& goal );
    void moveAbort(void);
    bool detectTray(void);
    
    

private:
    // ros::NodeHandle* MyNodeHandle;
    
    PTUClient _ptuClient;
    MoveBaseClient _moveBaseClient;    
    bool _move_aborted ;
    geometry_msgs::Point _left_handle, _right_handle;
    

};



#endif