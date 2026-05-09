#include "legged_controllers/DmController.h"

namespace damiao
{

bool DmController::init(legged::HybridJointInterface* robot_hw, ros::NodeHandle& nh)
{
  std::vector<std::string> joint_names;
  if (!nh.getParam("joints", joint_names) || joint_names.empty())
  {
    ROS_ERROR("DmController: 'joints' parameter missing or empty");
    return false;
  }

  for (const auto& name : joint_names)
  {
    try
    {
      joints_.push_back(robot_hw->getHandle(name));
      ROS_INFO_STREAM("DmController: joint '" << name << "' acquired");
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("DmController: cannot get handle '" << name << "': " << e.what());
      return false;
    }
  }

  state_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  return true;
}

void DmController::starting(const ros::Time& time)
{
  last_pub_time_ = time;
  ROS_INFO("DmController: started with %zu joint(s)", joints_.size());
}

// ======================================================================
//  ▼  在这里修改每个电机的五个指令参数  ▼
//
//  setCommand(pos [rad], vel [rad/s], kp, kd, force [Nm])
//
//  joints_ 的顺序和 motor_test_controllers.yaml 里 joints: 列表一致
//
//  诊断模式：kp=0, kd=1.0 —— 纯速度阻尼
//    用手拨动电机能感受到明显粘滞感 = 电机已使能
//    无反应 = 电机未使能（CAN ID 错误或通信问题）
// ======================================================================
void DmController::update(const ros::Time& time, const ros::Duration& period)
{
  if (joints_.size() > 0)
    joints_[0].setCommand(0.0, 0.0, 30.0, 1.0, 0.0);  // leg_r1_joint: kp=0 kd=1.0

  if (joints_.size() > 1)
    joints_[1].setCommand(0.0, 0.0, 30.0, 1.0, 0.0);  // leg_r2_joint

  if (joints_.size() > 2)
    joints_[2].setCommand(0.0, 0.0, 30.0, 1.0, 0.0);  // leg_r3_joint

  if (joints_.size() > 3)
    joints_[3].setCommand(0.0, 0.0, 30.0, 1.0, 0.0);  // leg_r4_joint

  if (joints_.size() > 4)
    joints_[4].setCommand(0.0, 0.0, 30.0, 1.0, 0.0);  // leg_r5_joint

  if (joints_.size() > 5)
    joints_[5].setCommand(0.0, 0.0, 30.0, 1.0, 0.0);

  if (joints_.size() > 6)
    joints_[6].setCommand(0.0, 0.0, 30.0, 1.0, 0.0);

  if (joints_.size() > 7)
    joints_[7].setCommand(0.0, 0.0, 30.0, 1.0, 0.0);

  if (joints_.size() > 8)
    joints_[8].setCommand(0.0, 0.0, 30.0, 1.0, 0.0);

  if (joints_.size() > 9)
    joints_[9].setCommand(0.0, 0.0, 30.0, 1.0, 0.0);

  // Publish feedback at 20 Hz
  if ((time - last_pub_time_).toSec() >= 0.05)
  {
    sensor_msgs::JointState msg;
    msg.header.stamp = time;
    for (const auto& j : joints_)
    {
      msg.name.push_back(j.getName());
      msg.position.push_back(j.getPosition());
      msg.velocity.push_back(j.getVelocity());
      msg.effort.push_back(j.getEffort());
    }
    state_pub_.publish(msg);
    last_pub_time_ = time;
  }
}
// ======================================================================

void DmController::stopping(const ros::Time& time)
{
  for (auto& j : joints_)
    j.setCommand(0.0, 0.0, 0.0, 0.0, 0.0);
  ROS_INFO("DmController: stopped, all commands zeroed");
}

}  // namespace damiao

PLUGINLIB_EXPORT_CLASS(damiao::DmController, controller_interface::ControllerBase)
