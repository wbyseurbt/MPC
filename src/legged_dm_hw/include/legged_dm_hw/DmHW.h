
//
// Created by qiayuan on 1/24/22.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.
********************************************************************************/

#pragma once

#include <dm_hw/DmHW.h>

#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

namespace legged
{

struct DmImuData
{
  double ori[4]{};
  double ori_cov[9]{};
  double angular_vel[3]{};
  double angular_vel_cov[9]{};
  double linear_acc[3]{};
  double linear_acc_cov[9]{};
};

class DmHW : public damiao::DmHW
{
public:
  DmHW() = default;
  ~DmHW() = default;

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;

private:
  bool setupImu();

  DmImuData imuData_{};
  ros::Subscriber odom_sub_;
  sensor_msgs::Imu latestImu_;

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) { latestImu_ = *msg; }
};

}  // namespace legged
