#ifndef WAITER_H
#define WITER_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <tf/transform_datatypes.h>
#include <task_manager/PTUAction.h>




typedef actionlib::SimpleActionClient<task_manager::PTUAction> PTUClient;
// Custom types
struct PointPTU
{
    double pan, tilt;
};
// struct Pose2D
// {
//     double x, y, theta;
// };

class Waiter
{

public:
    Waiter(/* args */):
           _ptuClient("ptu_action", true)
    {

    }

    bool movePTU(PointPTU&  goal);
    
    

private:
    
    PTUClient _ptuClient;
    

};



#endif