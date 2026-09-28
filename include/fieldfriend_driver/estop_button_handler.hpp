// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "fieldfriend_driver/communication.hpp"
#include "fieldfriend_driver/module_handler.hpp"

namespace fieldfriend_driver
{

/// Handles a hardware emergency stop button reported as part of the core data.
class EStopButtonHandler : public ModuleHandler, public CoreObserver
{
public:
  EStopButtonHandler(rclcpp::Node * node, Communication & comm, const std::string & module_name);

  void update(const CoreData & data) override;

private:
  void publish_estop(bool emergency_stop);

  std::string topic_name_;
  std::string data_name_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

}  // namespace fieldfriend_driver
