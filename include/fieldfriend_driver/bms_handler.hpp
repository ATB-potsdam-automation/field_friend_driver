// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "fieldfriend_driver/communication.hpp"
#include "fieldfriend_driver/module_handler.hpp"

namespace fieldfriend_driver
{

/// Handles the battery management system data.
class BMSHandler : public ModuleHandler, public BmsObserver
{
public:
  BMSHandler(rclcpp::Node * node, Communication & comm);

  void update(const std::vector<std::string> & words) override;

private:
  void send_request();

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  Communication & comm_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr send_request_timer_;
};

}  // namespace fieldfriend_driver
