// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "fieldfriend_driver/module_handler.hpp"
#include "fieldfriend_driver/serial_communication.hpp"

namespace fieldfriend_driver
{

/// Field friend node handler.
class FieldfriendDriver : public rclcpp::Node
{
public:
  FieldfriendDriver();

private:
  void read_data();

  std::unique_ptr<SerialCommunication> serial_communication_;
  std::vector<std::unique_ptr<ModuleHandler>> module_handlers_;
  std::unique_ptr<ModuleHandler> configuration_handler_;

  rclcpp::TimerBase::SharedPtr read_timer_;
};

}  // namespace fieldfriend_driver
