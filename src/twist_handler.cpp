// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#include "fieldfriend_driver/twist_handler.hpp"

#include <sstream>

namespace fieldfriend_driver
{

TwistHandler::TwistHandler(rclcpp::Node * node, Communication & comm)
: comm_(comm), logger_(node->get_logger())
{
  node->declare_parameter("modules.twist_handler.twist_timeout", rclcpp::PARAMETER_DOUBLE);
  const double twist_timeout = node->get_parameter("modules.twist_handler.twist_timeout").as_double();

  node->declare_parameter("modules.twist_handler.send_twist_frequency", rclcpp::PARAMETER_DOUBLE);
  const double send_twist_frequency =
    node->get_parameter("modules.twist_handler.send_twist_frequency").as_double();

  cmd_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {cmd_callback(msg);});

  send_twist_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / send_twist_frequency), [this]() {send_twist();});

  twist_timeout_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(twist_timeout), [this]() {this->twist_timeout();});
  twist_timeout_timer_->cancel();  // autostart=False
}

std::string TwistHandler::build_command() const
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(3);
  oss << "wheels.speed(" << twist_.linear.x << ", " << twist_.angular.z << ")";
  return oss.str();
}

void TwistHandler::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  twist_timeout_timer_->cancel();
  twist_timeout_timer_->reset();
  twist_ = *msg;
}

void TwistHandler::send_twist()
{
  comm_.send(build_command());
}

void TwistHandler::twist_timeout()
{
  RCLCPP_DEBUG(logger_, "Twist timeout. Stopping robot.");
  twist_timeout_timer_->cancel();
  twist_ = geometry_msgs::msg::Twist();
}

}  // namespace fieldfriend_driver
