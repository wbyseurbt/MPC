
//
// Created by qiayuan on 1/24/22.
//

/********************************************************************************
Modified Copyright (c) 2024-2025, Dm Robotics.
********************************************************************************/

#include "dm_hw/DmHW.h"

#include <xmlrpcpp/XmlRpcException.h>

namespace damiao
{

bool DmHW::parseDmActData(XmlRpc::XmlRpcValue& act_datas, ros::NodeHandle& robot_hw_nh)
{
  ROS_ASSERT(act_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  try
  {
    for (auto it = act_datas.begin(); it != act_datas.end(); ++it)
    {
      if (!it->second.hasMember("port"))
      {
        ROS_ERROR_STREAM("DmActuator " << it->first << " has no associated port.");
        continue;
      }
      if (!it->second.hasMember("type"))
      {
        ROS_ERROR_STREAM("DmActuator " << it->first << " has no associated type.");
        continue;
      }
      if (!it->second.hasMember("can_id"))
      {
        ROS_ERROR_STREAM("DmActuator " << it->first << " has no associated CAN_ID.");
        continue;
      }
      if (!it->second.hasMember("mst_id"))
      {
        ROS_ERROR_STREAM("DmActuator " << it->first << " has no associated MST_ID.");
        continue;
      }

      std::string port = act_datas[it->first]["port"];
      std::string type = act_datas[it->first]["type"];
      int can_id = static_cast<int>(act_datas[it->first]["can_id"]);
      int mst_id = static_cast<int>(act_datas[it->first]["mst_id"]);
      int direction = 1;
      if (it->second.hasMember("direction"))
        direction = static_cast<int>(act_datas[it->first]["direction"]);

      DM_Motor_Type motorType;
      if (type == "DM4310")
        motorType = DM4310;
      else if (type == "DM4310_48V")
        motorType = DM4310_48V;
      else if (type == "DM4340")
        motorType = DM4340;
      else if (type == "DM4340_48V")
        motorType = DM4340_48V;
      else if (type == "DM6006")
        motorType = DM6006;
      else if (type == "DM8006")
        motorType = DM8006;
      else if (type == "DM8009")
        motorType = DM8009;
      else if (type == "DM10010L")
        motorType = DM10010L;
      else if (type == "DM10010")
        motorType = DM10010;
      else if (type == "DMH3510")
        motorType = DMH3510;
      else if (type == "DMH6215")
        motorType = DMH6215;
      else if (type == "DMG6220")
        motorType = DMG6220;
      else
      {
        ROS_ERROR("Unknown motor type: %s", type.c_str());
        return false;
      }

      if (port_id2dm_data_.find(port) == port_id2dm_data_.end())
        port_id2dm_data_.insert(std::make_pair(port, std::unordered_map<int, DmActData>()));

      if (port_id2dm_data_[port].find(can_id) != port_id2dm_data_[port].end())
      {
        ROS_ERROR_STREAM("Duplicate DmActuator on port " << port << " CAN ID " << can_id);
        return false;
      }

      port_id2dm_data_[port].insert(std::make_pair(can_id, DmActData{
          .name = it->first,
          .motorType = motorType,
          .can_id = can_id,
          .mst_id = mst_id,
          .pos = 0, .vel = 0, .effort = 0,
          .cmd_pos = 0, .cmd_vel = 0, .cmd_effort = 0,
          .kp = 0, .kd = 0,
          .direction = direction }));

      hardware_interface::JointStateHandle state_handle(
          port_id2dm_data_[port][can_id].name,
          &port_id2dm_data_[port][can_id].pos,
          &port_id2dm_data_[port][can_id].vel,
          &port_id2dm_data_[port][can_id].effort);
      jointStateInterface_.registerHandle(state_handle);

      hybridJointInterface_.registerHandle(legged::HybridJointHandle(
          state_handle,
          &port_id2dm_data_[port][can_id].cmd_pos,
          &port_id2dm_data_[port][can_id].cmd_vel,
          &port_id2dm_data_[port][can_id].kp,
          &port_id2dm_data_[port][can_id].kd,
          &port_id2dm_data_[port][can_id].cmd_effort));
    }
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_FATAL_STREAM("XmlRpc exception while reading dm configuration: " << e.getMessage());
    return false;
  }
  return true;
}

bool DmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  XmlRpc::XmlRpcValue xml_rpc_value;

  registerInterface(&jointStateInterface_);
  registerInterface(&hybridJointInterface_);
  registerInterface(&imuSensorInterface_);
  registerInterface(&contactSensorInterface_);

  if (!robot_hw_nh.getParam("dm_actuators", xml_rpc_value))
    ROS_WARN("No dm_actuators specified");
  else if (!parseDmActData(xml_rpc_value, robot_hw_nh))
    return false;

  if (!robot_hw_nh.getParam("serials", xml_rpc_value))
    ROS_WARN("No serials specified");
  else if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
    {
      if (!it->second.hasMember("port"))
      {
        ROS_ERROR_STREAM("Serial " << it->first << " has no associated port.");
        continue;
      }
      if (!it->second.hasMember("baudrate"))
      {
        ROS_ERROR_STREAM("Serial " << it->first << " has no associated baudrate.");
        continue;
      }
      std::string port = xml_rpc_value[it->first]["port"];
      int baudrate = static_cast<int>(xml_rpc_value[it->first]["baudrate"]);
      ROS_INFO_STREAM("Opening serial port: " << port << " @ " << baudrate);
      motor_ports_.push_back(
          std::make_shared<Motor_Control>(port, baudrate, &port_id2dm_data_[port]));
    }
  }

  return true;
}

void DmHW::read(const ros::Time& time, const ros::Duration& period)
{
  for (auto& motor_port : motor_ports_)
    motor_port->read();

  // Convert raw motor positions to joint space using direction sign
  for (auto& [port, port_data] : port_id2dm_data_)
    for (auto& [can_id, data] : port_data)
    {
      data.pos *= data.direction;
      data.vel *= data.direction;
      data.effort *= data.direction;
    }
}

void DmHW::write(const ros::Time& time, const ros::Duration& period)
{
  // Convert joint-space commands to motor space before sending
  for (auto& [port, port_data] : port_id2dm_data_)
    for (auto& [can_id, data] : port_data)
    {
      data.cmd_pos *= data.direction;
      data.cmd_vel *= data.direction;
      data.cmd_effort *= data.direction;
    }

  for (auto& motor_port : motor_ports_)
    motor_port->write();

  // Restore joint-space commands (direction is ±1, so double-multiply restores)
  for (auto& [port, port_data] : port_id2dm_data_)
    for (auto& [can_id, data] : port_data)
    {
      data.cmd_pos *= data.direction;
      data.cmd_vel *= data.direction;
      data.cmd_effort *= data.direction;
    }
}

}  // namespace damiao
