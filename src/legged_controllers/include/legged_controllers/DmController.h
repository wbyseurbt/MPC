#pragma once

#include <vector>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

namespace damiao
{

class DmController : public controller_interface::Controller<legged::HybridJointInterface>
{
public:
  DmController() = default;
  ~DmController() = default;

  bool init(legged::HybridJointInterface* robot_hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  std::vector<legged::HybridJointHandle> joints_;

  ros::Publisher state_pub_;
  ros::Time last_pub_time_;
};

}  // namespace damiao
