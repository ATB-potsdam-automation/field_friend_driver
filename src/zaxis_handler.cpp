// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#include "fieldfriend_driver/zaxis_handler.hpp"

#include <sstream>

namespace fieldfriend_driver
{

ZAxisHandler::ZAxisHandler(rclcpp::Node * node, Communication & comm)
: comm_(comm), logger_(node->get_logger())
{
  subscription_ = node->create_subscription<std_msgs::msg::Float32>(
    "zaxis/target_speed", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg) {speed_callback(msg);});
}

void ZAxisHandler::speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  double speed = msg->data;
  if (speed > max_speed_ || speed < -max_speed_) {
    const double clamped = (speed > 0 ? 1.0 : (speed < 0 ? -1.0 : 0.0)) * max_speed_;
    RCLCPP_WARN(logger_, "Thresshold speed target of %f to %f", speed, clamped);
    speed = clamped;
  }
  std::ostringstream oss;
  oss << "zaxis.speed(" << (speed * steps_per_m_) << ")";
  comm_.send(oss.str());
}

}  // namespace fieldfriend_driver
