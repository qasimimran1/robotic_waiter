#ifndef BEHAVOURTREE_NODES_H
#define BEHAVOURTREE_NODES_H


#include <behaviortree_cpp_v3/bt_factory.h>
#include <task_manager/waiter.h>


BT::NodeStatus lookAtTray(void);
BT::NodeStatus lookUp(void);
BT::NodeStatus detectHandles(void);
BT::NodeStatus pickTray(void);
BT::NodeStatus placeTray(void);
BT::NodeStatus moveBack(void);
BT::NodeStatus goToTable(void);

//inline void RegisterNodes(BT::BehaviorTreeFactory& factory, Waiter& waiter){

inline void RegisterNodes(BT::BehaviorTreeFactory& factory){    

    factory.registerSimpleAction("LookAtTheTray", std::bind(lookAtTray));
    factory.registerSimpleAction("DetectHandles", std::bind(detectHandles));
    factory.registerSimpleAction("PickTray", std::bind(pickTray));
    factory.registerSimpleAction("LookUp", std::bind(lookUp));
    factory.registerSimpleAction("GoToTable", std::bind(goToTable));
    factory.registerSimpleAction("MoveBack", std::bind(moveBack));

    factory.registerSimpleAction("PlaceTray", std::bind(placeTray));
                                
} 


// 





// BT::PortsList providedPorts()
//     {
//         return{ BT::InputPort<PointPTU>("ptu_goal") };
//     }

//     virtual BT::NodeStatus tick() override;

//     virtual void halt() override
//     {namespace BT
// {
// template <> inline
// Pose2D convertFromString(StringView key)
// {
//     // three real numbers separated by semicolons
//     auto parts = BT::splitString(key, ';');
//     if (parts.size() != 3)
//     {
//         throw BT::RuntimeError("invalid input)");
//     }
//     else
//     {
//         Pose2D output;
//         output.x     = convertFromString<double>(parts[0]);
//         output.y     = convertFromString<double>(parts[1]);
//         output.theta = convertFromString<double>(parts[2]);
//         return output;
//     }
// }

// template <> inline PointPTU convertFromString(StringView key)
// {
//     // 2 real numbers separated by semicolons
//     auto parts = BT::splitString(key, ';');
//     if (parts.size() != 2)
//     {
//         throw BT::RuntimeError("invalid input)");
//     }
//     else
//     {
//         PointPTU output;
//         output.pan     = convertFromString<double>(parts[0]);
//         output.tilt     = convertFromString<double>(parts[1]);        
//         return output;
//     }
// }


// } // end namespace BT

// //----------------------------------------------------------------

// class MoveBase : public BT::AsyncActionNode
// {
// public:

//     MoveBase(const std::string& name, const BT::NodeConfiguration& config)
//         : BT::AsyncActionNode(name, config),
//           _client("move_base", true)
//     {
//     }

//     // It is mandatory to define this static method.
//     static BT::PortsList providedPorts()
//     {
//         return{ BT::InputPort<Pose2D>("goal") };
//     }

//     virtual BT::NodeStatus tick() override;

//     virtual void halt() override
//     {
//         _aborted = true;
//     }

// private:
//     typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//     MoveBaseClient _client;
//     bool _aborted;
// };



// //----------------------------------------------------------------

// class MovePTU : public BT::AsyncActionNode
// {
// public:

//     MovePTU(const std::string& name, const BT::NodeConfiguration& config)
//         : BT::AsyncActionNode(name, config),
//           _client("ptu_action", true)
//     {
//     }

//     // It is mandatory to define this static method.
//     static 
//         _aborted = true;
//     }

// private:
//     typedef actionlib::SimpleActionClient<task_manager::PTUAction> PTUClient;
//     PTUClient _client;
//     bool _aborted;
// };













#endif // end ifndef