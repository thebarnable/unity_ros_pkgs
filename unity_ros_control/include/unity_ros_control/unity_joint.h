#ifndef _UNITY_ROS_CONTROL___UNITY_JOINT_H_
#define _UNITY_ROS_CONTROL___UNITY_JOINT_H_

#include <string>

namespace unity_ros_control
{

class UnityJoint
{
public:
  UnityJoint(std::string name, double position, double velocity, double effort);

  double GetPosition() const;
  double GetVelocity() const;
  double GetForce() const;

  void SetPosition(const double position);
  void SetVelocity(const double velocity);
  void SetForce(const double effort);

  std::string GetName() const;

private:
  std::string name_;

  double position_ = 0;
  double velocity_ = 0;
  double effort_ = 0;
};

}

#endif // _UNITY_ROS_CONTROL___UNITY_JOINT_H_
