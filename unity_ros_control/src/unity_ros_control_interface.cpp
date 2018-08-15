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

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Gazebo plugin for ros_control that allows 'hardware_interfaces' to be plugged in
   using pluginlib
*/

#include <boost/bind.hpp>

#include <unity_ros_control/unity_ros_control_interface.h>
#include <unity_ros_control/default_robot_hw_sim.h>

#include <urdf/model.h>

namespace unity_ros_control
{

UnityRosControlInterface::~UnityRosControlInterface()
{

}

void UnityRosControlInterface::Load(const RosData& rosData)
{
  ROS_INFO_STREAM_NAMED("unity_ros_control","Loading unity_ros_control plugin");

  if (rosData.controlPeriod <= 0)
  {
    ROS_FATAL_NAMED("unity_ros_control", "Error loading unity_ros_control plugin, control period has to be > 0.\n");
    return;
  }
  control_period_ = ros::Duration(rosData.controlPeriod);

  // Get parameters/settings for controllers from ROS param server
  model_nh_ = ros::NodeHandle(rosData.robotNamespace);

  // Initialize the emergency stop code.
  ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Setting up subscriber and publisher");
  e_stop_active_ = false;
  last_e_stop_active_ = false;
  if (rosData.eStopTopic != "")
  {
    e_stop_sub_ = model_nh_.subscribe(rosData.eStopTopic, 1, &UnityRosControlInterface::eStopCB, this);
  }

  // Initialize subscriber to unity joint states and publisher of ros control setpoint joint states
  unity_joint_sub_ = model_nh_.subscribe(rosData.jointStateTopic, 1, &UnityRosControlInterface::unityJointCB, this);
  ros_joint_pub_ = model_nh_.advertise<sensor_msgs::JointState>("/ros_joint_states", 1);

  // Wait until first unity joint state arrives
  ROS_INFO_STREAM_NAMED("ros_control_plugin","Waiting for simulation to publish first joint state...");
  sensor_msgs::JointStateConstPtr first_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>(rosData.jointStateTopic);
  if(!first_joint_state) {
    ROS_FATAL_NAMED("unity_ros_control", "Error loading unity_ros_control plugin, not receiving joint states.\n");
    return;
  }

  // Initialize initial joint state
  for(size_t i=0; i<first_joint_state->name.size(); i++) {
    this->joints_.push_back(new UnityJoint(first_joint_state->name[i], first_joint_state->position[i], first_joint_state->velocity[i], first_joint_state->effort[i]));
  }

  ROS_INFO_NAMED("unity_ros_control", "Starting unity_ros_control plugin in namespace: %s", rosData.robotNamespace.c_str());

  // Read urdf from ros parameter server, then setup actuators and mechanism control node
  const std::string urdf_string = getURDF(rosData.robotParam);
  std::vector<transmission_interface::TransmissionInfo> transmissions;
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions);

  // Load the RobotHWSim abstraction to interface the controllers with the model
  try
  {
    this->robot_hw_sim_ = std::make_shared<unity_ros_control::DefaultRobotHWSim>();

    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    if(!robot_hw_sim_->initSim(rosData.robotNamespace, model_nh_, joints_, urdf_model_ptr, transmissions))
    {
      ROS_FATAL_NAMED("unity_ros_control","Could not initialize robot simulation interface");
      return;
    }

    // Create the controller manager
    ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Loading controller_manager");
    controller_manager_.reset(new controller_manager::ControllerManager(robot_hw_sim_.get(), model_nh_));

    // Get controller names from parameter server and start them
    std::string controller_names_batch;
    model_nh_.getParam("/controller_names", controller_names_batch);
    std::vector<std::string> controller_names;
    boost::split(controller_names, controller_names_batch, [](char c){return c == ' ';});

    for(auto& controller : controller_names)
    {
      ROS_INFO_STREAM_NAMED("ros_control_plugin","Loading controller: " + controller);
      controller_manager_->loadController(controller);
      while(!controller_manager_->getControllerByName(controller)->isRunning()) {
        controller_manager_->getControllerByName(controller)->startRequest(ros::Time::now());
      }
    }

    // Set up update callback
    //ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Setting up update callback");
    //update_sub_ = model_nh_.subscribe(rosData.updateTopic, 1, &UnityRosControlInterface::Update, this);
  }
  catch(pluginlib::LibraryLoadException &ex)
  {
    ROS_FATAL_STREAM_NAMED("unity_ros_control","Failed to create robot simulation interface loader: "<<ex.what());
  }

  ROS_INFO_NAMED("unity_ros_control", "Loaded unity_ros_control.");
}

// Called by the world update start event
void UnityRosControlInterface::Update(const std_msgs::Time& sim_time_ros_msg)
{
  ros::Time sim_time_ros = sim_time_ros_msg.data;
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  robot_hw_sim_->eStopActive(e_stop_active_);

  // Check if we should update the controllers
  //ROS_INFO_STREAM(sim_period);
  //ROS_INFO_STREAM(control_period_);
  if(sim_period >= control_period_) {
    ROS_INFO("READING...");
    // Store this simulation time
    last_update_sim_time_ros_ = sim_time_ros;

    // Update the robot simulation with the state of the model
    robot_hw_sim_->readSim(sim_time_ros, sim_period);

    // Compute the controller commands
    bool reset_ctrlrs;
    if (e_stop_active_)
    {
      reset_ctrlrs = false;
      last_e_stop_active_ = true;
    }
    else
    {
      if (last_e_stop_active_)
      {
        reset_ctrlrs = true;
        last_e_stop_active_ = false;
      }
      else
      {
        reset_ctrlrs = false;
      }
    }
    ROS_INFO("UPDATING CM...");
    controller_manager_->update(sim_time_ros, sim_period, reset_ctrlrs);
  }
  // Update the model with the result of the controller computation
  ROS_INFO("WRITING...");
  robot_hw_sim_->writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  // Publish results
  sensor_msgs::JointState ros_joint_msg;
  ros_joint_msg.header.stamp = sim_time_ros;

  for(size_t i=0; i<this->joints_.size(); i++) {
    // if(this->joints_[i]->GetName() == "el_y") {
    //   ROS_INFO_STREAM("  Joint: " + this->joints_[i]->GetName());
    //   ROS_INFO_STREAM("  Position: " + std::to_string(this->joints_[i]->GetPosition()));
    //   ROS_INFO_STREAM("  Velocity: " + std::to_string(this->joints_[i]->GetVelocity()));
    //   ROS_INFO_STREAM("  Force: " + std::to_string(this->joints_[i]->GetForce()));
    // }

    ros_joint_msg.name.push_back(this->joints_[i]->GetName());
    ros_joint_msg.position.push_back(this->joints_[i]->GetPosition());
    ros_joint_msg.velocity.push_back(this->joints_[i]->GetVelocity());
    ros_joint_msg.effort.push_back(this->joints_[i]->GetForce());
  }


  ROS_INFO("PUBLISHING...");
  ros_joint_pub_.publish(ros_joint_msg);

  last_write_sim_time_ros_ = sim_time_ros;
}

// Called on world reset
void UnityRosControlInterface::Reset()
{
  // Reset timing variables to not pass negative update periods to controllers on world reset
  last_update_sim_time_ros_ = ros::Time();
  last_write_sim_time_ros_ = ros::Time();
}

// Get the URDF XML from the parameter server
std::string UnityRosControlInterface::getURDF(const std::string& param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("unity_ros_control", "unity_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("unity_ros_control", "unity_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", param_name.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("unity_ros_control", "Recieved urdf from param server, parsing...");

  return urdf_string;
}

// Emergency stop callback
void UnityRosControlInterface::eStopCB(const std_msgs::BoolConstPtr& e_stop_active)
{
  e_stop_active_ = e_stop_active->data;
}

void UnityRosControlInterface::unityJointCB(const sensor_msgs::JointState::ConstPtr& joint_msg) {
  ROS_INFO("Tick");
  for(size_t i=0; i<joint_msg->name.size(); i++) {
    for(auto joint : this->joints_) {
      if(joint_msg->name[i] == joint->GetName()) {
        joint->SetPosition(joint_msg->position[i]);
        joint->SetVelocity(joint_msg->velocity[i]);
        joint->SetForce(joint_msg->effort[i]);
      }
    }
  }

  std_msgs::Time current_time;
  current_time.data = ros::Time(joint_msg->header.stamp.sec, joint_msg->header.stamp.nsec);
  this->Update(current_time);
}

} // namespace
