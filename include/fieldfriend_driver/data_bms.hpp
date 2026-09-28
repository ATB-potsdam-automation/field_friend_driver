// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "sensor_msgs/msg/battery_state.hpp"

namespace fieldfriend_driver
{

/// Parses one "bms" expander message (protocol of a common Smart BMS).
class DataBMS
{
public:
  explicit DataBMS(std::vector<int> bytes);

  int address() const {return bytes_.at(1);}
  int status() const {return bytes_.at(2);}
  int length() const {return bytes_.at(3);}
  std::vector<int> content() const;

  /// Throws std::runtime_error if the frame does not look valid.
  void check() const;

  sensor_msgs::msg::BatteryState get_ros_message();

private:
  int get1();
  int get2();
  int get2_signed();

  std::vector<int> bytes_;
  int cursor_ = -1;
};

}  // namespace fieldfriend_driver
