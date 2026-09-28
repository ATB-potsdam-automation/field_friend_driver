// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "fieldfriend_driver/communication.hpp"
#include "fieldfriend_driver/module_handler.hpp"

namespace fieldfriend_driver
{

/// Handles the z-axis of the endeffector.
class ZAxisHandler : public ModuleHandler
{
public:
  ZAxisHandler(rclcpp::Node * node, Communication & comm);

private:
  void speed_callback(const std_msgs::msg::Float32::SharedPtr msg);

  Communication & comm_;
  rclcpp::Logger logger_;

  const double steps_per_m_ = 1600000;
  const double max_speed_ = 0.02;  // [m/s]

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

}  // namespace fieldfriend_driver
