// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "fieldfriend_driver/communication.hpp"
#include "fieldfriend_driver/module_handler.hpp"

namespace fieldfriend_driver
{

class EStopHandler;

/// Represents a single emergency stop input (hardware or software controlled).
class EStop
{
public:
  EStop(rclcpp::Node * node, const std::string & param_name, EStopHandler * handler);

  bool triggered() const {return estop_triggered_;}
  bool software_estop() const {return estop_software_;}
  const std::string & name() const {return estop_name_;}
  const std::string & message() const {return message_;}

  void callback(const std_msgs::msg::Bool::SharedPtr msg);
  void timeout_callback();

private:
  EStopHandler * handler_;

  bool estop_triggered_ = false;
  bool estop_software_ = false;
  std::string estop_name_;
  std::string message_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

/// Handles the aggregation of all emergency stop inputs.
class EStopHandler : public ModuleHandler
{
public:
  EStopHandler(rclcpp::Node * node, Communication & comm);

  /// Checks global estop state, returns true if it changed.
  bool check_global_estop();
  /// Checks software estop state, returns true if it changed.
  bool check_software_estop();

  void send_software_estop(bool value);
  void callback(EStop * estop);

private:
  rclcpp::Logger logger_;
  Communication & comm_;

  bool software_estop_triggered_ = true;
  bool global_estop_triggered_ = true;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_global_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_message_;

  std::vector<std::unique_ptr<EStop>> estop_list_;
};

}  // namespace fieldfriend_driver
