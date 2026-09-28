// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "fieldfriend_driver/communication.hpp"
#include "fieldfriend_driver/module_handler.hpp"

namespace fieldfriend_driver
{

/// Handles forwarding cmd_vel Twist commands to the esp.
class TwistHandler : public ModuleHandler
{
public:
  TwistHandler(rclcpp::Node * node, Communication & comm);

private:
  std::string build_command() const;
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void send_twist();
  void twist_timeout();

  Communication & comm_;
  rclcpp::Logger logger_;

  geometry_msgs::msg::Twist twist_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
  rclcpp::TimerBase::SharedPtr send_twist_timer_;
  rclcpp::TimerBase::SharedPtr twist_timeout_timer_;
};

}  // namespace fieldfriend_driver
