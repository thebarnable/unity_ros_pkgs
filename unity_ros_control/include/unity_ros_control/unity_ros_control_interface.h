/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* --- Original: gazebo_ros_control_plugin.h ---
   Author: Dave Coleman, Jonathan Bohren
   Desc:   Gazebo plugin for ros_control that allows 'hardware_interfaces' to be plugged in
           using pluginlib
   --- Unity version: unity_ros_control_interface.h ---
   Author: Tim Stadtmann
   Desc:   Interface between Unity and ROS Control - Entry point and updates are triggered
           using special topics
*/

#ifndef _UNITY_ROS_CONTROL__UNITY_ROS_CONTROL_INTERFACE_
#define _UNITY_ROS_CONTROL__UNITY_ROS_CONTROL_INTERFACE_

// Boost
#include <memory>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/JointState.h>

// ros_control
#include <unity_ros_control/robot_hw_sim.h>
#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

// unity ros control
#include <unity_ros_control/unity_joint.h>

namespace unity_ros_control
{

// Struct with ROS-related data of the robot (in Gazebo mostly taken from SDF)
struct RosData
{
  std::string robotNamespace = "/"; // ROS namespace
  std::string robotParam = "robot_description"; // Name of urdf parameter on param server
  std::string robotSimType = "unity_ros_control/DefaultRobotHWSim"; // Name of Plugin for RobotHWSim
  std::string jointStateTopic = "/joint_states";
  std::string eStopTopic = "";
  std::string updateTopic = "/tick";
  double controlPeriod = 0.06; // == Unity physics step size
};

class UnityRosControlInterface
{
public:
  ~UnityRosControlInterface();

  // Entry point
  void Load(const RosData& rosData);

  // Called by the world update start event
  void Update(const std_msgs::Time& sim_time_ros_msg);

  // Called on world reset
  void Reset();

private:
  // Get the URDF XML from the parameter server
  std::string getURDF(const std::string& param_name) const;

  // Emergency stop callback
  void eStopCB(const std_msgs::BoolConstPtr& e_stop_active);

  // Unity joints
  std::vector<UnityJoint*> joints_;
  void unityJointCB(const sensor_msgs::JointState::ConstPtr& joint_msg);

  // Node Handles
  ros::NodeHandle model_nh_;

  // deferred load in case ros is blocking
  boost::thread deferred_load_thread_;

  // Robot simulator interface and interface loader
  // boost::shared_ptr<pluginlib::ClassLoader<unity_ros_control::RobotHWSim>> robot_hw_sim_loader_;
  // void load_robot_hw_sim_srv();
  std::shared_ptr<unity_ros_control::RobotHWSim> robot_hw_sim_;

  // Controller manager
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Timing
  ros::Duration control_period_;
  ros::Time last_update_sim_time_ros_;
  ros::Time last_write_sim_time_ros_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;
  ros::Subscriber e_stop_sub_;  // Emergency stop subscriber

  //ros::Subscriber update_sub_;

  ros::Subscriber unity_joint_sub_;
  ros::Publisher ros_joint_pub_;
};


}

#endif // _UNITY_ROS_CONTROL__UNITY_ROS_CONTROL_INTERFACE_
