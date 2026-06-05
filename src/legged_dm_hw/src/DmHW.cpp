
//
// Created by qiayuan on 1/24/22.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.
********************************************************************************/

#include "legged_dm_hw/DmHW.h"

namespace legged
{

bool DmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  // Subscribe to IMU before calling base init so the callback can be invoked
  // as soon as the spinner is running.
  odom_sub_ = root_nh.subscribe("/imu/data", 1, &DmHW::imuCallback, this);

  // Base class parses dm_actuators + serials yaml, creates Motor_Control
  // objects, and registers jointState / hybrid / imu / contact interfaces.
  if (!damiao::DmHW::init(root_nh, robot_hw_nh))
    return false;

  if (!setupImu())
    return false;

  return true;
}

bool DmHW::setupImu()
{
  imuData_.ori_cov[0] = 0.0012;
  imuData_.ori_cov[4] = 0.0012;
  imuData_.ori_cov[8] = 0.0012;

  imuData_.angular_vel_cov[0] = 0.0004;
  imuData_.angular_vel_cov[4] = 0.0004;
  imuData_.angular_vel_cov[8] = 0.0004;

  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
      "imu_link", "imu_link",
      imuData_.ori, imuData_.ori_cov,
      imuData_.angular_vel, imuData_.angular_vel_cov,
      imuData_.linear_acc, imuData_.linear_acc_cov));
  return true;
}

void DmHW::read(const ros::Time& time, const ros::Duration& period)
{
  // Read motor states via CAN (applies direction mapping)
  damiao::DmHW::read(time, period);

  // Copy latest IMU data into the interface buffers
  imuData_.ori[0] = latestImu_.orientation.x;
  imuData_.ori[1] = latestImu_.orientation.y;
  imuData_.ori[2] = latestImu_.orientation.z;
  imuData_.ori[3] = latestImu_.orientation.w;

  imuData_.angular_vel[0] = latestImu_.angular_velocity.x;
  imuData_.angular_vel[1] = latestImu_.angular_velocity.y;
  imuData_.angular_vel[2] = latestImu_.angular_velocity.z;

  imuData_.linear_acc[0] = latestImu_.linear_acceleration.x;
  imuData_.linear_acc[1] = latestImu_.linear_acceleration.y;
  imuData_.linear_acc[2] = latestImu_.linear_acceleration.z;
}

}  // namespace legged
