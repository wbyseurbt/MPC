#include "legged_controllers/DmController.h"

namespace damiao
{

void DmController::dynamicParamCallback(legged_controllers::DmMotorTestConfig& config, uint32_t /*level*/)
{
  constexpr double kDegToRad = 0.017453292519943295;
  std::lock_guard<std::mutex> lock(command_mutex_);
  enable_output_ = config.enable_output;
  command_velocity_ = config.command_velocity;
  command_kp_ = config.command_kp;
  command_kd_ = config.command_kd;
  command_ff_ = config.command_ff;

  if (target_positions_.size() > 0) target_positions_[0] = config.joint_1_deg * kDegToRad;
  if (target_positions_.size() > 1) target_positions_[1] = config.joint_2_deg * kDegToRad;
  if (target_positions_.size() > 2) target_positions_[2] = config.joint_3_deg * kDegToRad;
  if (target_positions_.size() > 3) target_positions_[3] = config.joint_4_deg * kDegToRad;
  if (target_positions_.size() > 4) target_positions_[4] = config.joint_5_deg * kDegToRad;
  if (target_positions_.size() > 5) target_positions_[5] = config.joint_6_deg * kDegToRad;
  if (target_positions_.size() > 6) target_positions_[6] = config.joint_7_deg * kDegToRad;
  if (target_positions_.size() > 7) target_positions_[7] = config.joint_8_deg * kDegToRad;
  if (target_positions_.size() > 8) target_positions_[8] = config.joint_9_deg * kDegToRad;
  if (target_positions_.size() > 9) target_positions_[9] = config.joint_10_deg * kDegToRad;
}

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
  target_positions_.assign(joints_.size(), 0.0);

  serverPtr_ = std::make_unique<dynamic_reconfigure::Server<legged_controllers::DmMotorTestConfig>>(nh);
  dynamic_reconfigure::Server<legged_controllers::DmMotorTestConfig>::CallbackType cb;
  cb = boost::bind(&DmController::dynamicParamCallback, this, _1, _2);
  serverPtr_->setCallback(cb);

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
  std::vector<double> target_positions;
  double command_velocity;
  double command_kp;
  double command_kd;
  double command_ff;
  bool enable_output;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    target_positions = target_positions_;
    command_velocity = command_velocity_;
    command_kp = command_kp_;
    command_kd = command_kd_;
    command_ff = command_ff_;
    enable_output = enable_output_;
  }

  for (size_t i = 0; i < joints_.size() && i < target_positions.size(); ++i)
  {
    if (enable_output)
    {
      joints_[i].setCommand(target_positions[i], command_velocity, command_kp, command_kd, command_ff);
    }
    else
    {
      joints_[i].setCommand(0.0, 0.0, 0.0, 0.0, 0.0);
    }
  }

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
