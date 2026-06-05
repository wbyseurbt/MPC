/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *******************************************************************************/

//
// Created by qiayuan on 12/27/20.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.
********************************************************************************/

#include <dm_hw/DmHWLoop.h>
#include "legged_dm_hw/DmHW.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "legged_dm_hw");
  ros::NodeHandle nh;
  ros::NodeHandle robot_hw_nh("~");

  // We run the ROS loop in a separate thread as external calls, such as
  // service callbacks loading controllers, can block the (main) control loop.
  ros::AsyncSpinner spinner(3);
  spinner.start();

  try
  {
    auto legged_dm_hw = std::make_shared<legged::DmHW>();
    legged_dm_hw->init(nh, robot_hw_nh);

    damiao::DmHWLoop control_loop(nh, legged_dm_hw);

    ros::waitForShutdown();
  }
  catch (const ros::Exception& e)
  {
    ROS_FATAL_STREAM("Error in the hardware interface:\n\t" << e.what());
    return 1;
  }

  return 0;
}
