// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "fieldfriend_driver/communication.hpp"
#include "fieldfriend_driver/module_handler.hpp"

namespace fieldfriend_driver
{

/// Handles pushing the lizard startup configuration file to the esp on request.
class ConfigurationHandler : public ModuleHandler
{
public:
  ConfigurationHandler(rclcpp::Node * node, Communication & comm, std::string filename);

private:
  void handle_configure(const std_msgs::msg::String::SharedPtr msg);

  Communication & comm_;
  rclcpp::Logger logger_;
  std::string filename_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}  // namespace fieldfriend_driver
