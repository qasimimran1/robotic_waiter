#include <ros/ros.h>
#include <phoebe_perception/target_position.h>


    

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_perception_client");
  ros::NodeHandle nh;
  ros::ServiceClient _perceptionClient = nh.serviceClient<phoebe_perception::target_position>("get_position");

  phoebe_perception::target_position srv;
  geometry_msgs::Point _left_handle, _right_handle;
    
    if(_perceptionClient.call(srv)){
        _left_handle = srv.response.points[0];
        _right_handle = srv.response.points[1];
        ROS_WARN("left_handle: %f right_handle: %f", _left_handle.x, _right_handle.x);
        
    }else
    {
        ROS_ERROR("Failed to Call Service");        
    }
  return 0; 
}