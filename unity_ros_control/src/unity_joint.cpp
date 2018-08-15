#include <unity_ros_control/unity_joint.h>

namespace unity_ros_control
{
  UnityJoint::UnityJoint(std::string name, double position, double velocity, double effort)
    : name_(name), position_(position), velocity_(velocity), effort_(effort) {}

  double UnityJoint::GetPosition() const {
    return this->position_;
  }
  double UnityJoint::GetVelocity() const {
    return this->velocity_;
  }
  double UnityJoint::GetForce() const {
    return this->effort_;
  }
  std::string UnityJoint::GetName() const {
    return this->name_;
  }

  void UnityJoint::SetPosition(const double position) {
    this->position_ = position;
  }
  void UnityJoint::SetVelocity(const double velocity) {
    this->velocity_ = velocity;
  }
  void UnityJoint::SetForce(const double effort) {
    this->effort_ = effort;
  }

}
