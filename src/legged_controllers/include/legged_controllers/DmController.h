#pragma once

#include <vector>
#include <memory>
#include <mutex>
#include <controller_interface/controller.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include "legged_controllers/DmMotorTestConfig.h"

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
  void dynamicParamCallback(legged_controllers::DmMotorTestConfig& config, uint32_t level);

  std::vector<legged::HybridJointHandle> joints_;
  std::vector<double> target_positions_;
  double command_velocity_ = 0.0;
  double command_kp_ = 30.0;
  double command_kd_ = 1.0;
  double command_ff_ = 0.0;
  bool enable_output_ = true;
  std::mutex command_mutex_;
  std::unique_ptr<dynamic_reconfigure::Server<legged_controllers::DmMotorTestConfig>> serverPtr_;

  ros::Publisher state_pub_;
  ros::Time last_pub_time_;
};

}  // namespace damiao
