
//
// Created by qiayuan on 6/24/22.
//

/********************************************************************************
Modified Copyright (c) 2024-2025, Dm Robotics.
********************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <urdf/model.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

#include "dm_hw/damiao.h"

namespace damiao
{

class DmHW : public hardware_interface::RobotHW
{
public:
  DmHW() = default;

  bool parseDmActData(XmlRpc::XmlRpcValue& act_datas, ros::NodeHandle& robot_hw_nh);

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

protected:
  // one element per serial/USB-CAN port
  std::vector<std::shared_ptr<Motor_Control>> motor_ports_{};
  // port-name → can_id → motor data (shared with Motor_Control)
  std::unordered_map<std::string, std::unordered_map<int, DmActData>> port_id2dm_data_{};

  // ros_control interfaces (subclasses may register additional ones)
  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;
  legged::HybridJointInterface hybridJointInterface_;
  legged::ContactSensorInterface contactSensorInterface_;
};

}  // namespace damiao
