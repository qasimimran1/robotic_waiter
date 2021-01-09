#ifndef BEHAVOURTREE_NODES_H
#define BEHAVOURTREE_NODES_H


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <tf/transform_datatypes.h>
#include <task_manager/PTUAction.h>


// Custom type
struct Pose2D
{
    double x, y, theta;
};

struct PointPTU
{
    double pan, tilt;
};


namespace BT
{
template <> inline
Pose2D convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        output.theta = convertFromString<double>(parts[2]);
        return output;
    }
}

template <> inline PointPTU convertFromString(StringView key)
{
    // 2 real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 2)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        PointPTU output;
        output.pan     = convertFromString<double>(parts[0]);
        output.tilt     = convertFromString<double>(parts[1]);        
        return output;
    }
}


} // end namespace BT

//----------------------------------------------------------------

class MoveBase : public BT::AsyncActionNode
{
public:

    MoveBase(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config),
          _client("move_base", true)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<Pose2D>("goal") };
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override
    {
        _aborted = true;
    }

private:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient _client;
    bool _aborted;
};



//----------------------------------------------------------------

class MovePTU : public BT::AsyncActionNode
{
public:

    MovePTU(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config),
          _client("ptu_action", true)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<PointPTU>("ptu_goal") };
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override
    {
        _aborted = true;
    }

private:
    typedef actionlib::SimpleActionClient<task_manager::PTUAction> PTUClient;
    PTUClient _client;
    bool _aborted;
};


#endif // end ifndef