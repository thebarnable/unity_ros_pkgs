#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

#include <unity_ros_control/unity_ros_control_interface.h>
#include <unity_ros_control/unity_joint.h>

using namespace unity_ros_control;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unity_ros_control_interface_node");
  UnityRosControlInterface unity_ros_control_interface;

  RosData ros_data;
  unity_ros_control_interface.Load(ros_data);

  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
