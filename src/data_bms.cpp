// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#include "fieldfriend_driver/data_bms.hpp"

namespace fieldfriend_driver
{

DataBMS::DataBMS(std::vector<int> bytes)
: bytes_(std::move(bytes))
{
}

std::vector<int> DataBMS::content() const
{
  // Equivalent to python's self.bytes[4:-3]
  if (bytes_.size() < 7) {
    return {};
  }
  return std::vector<int>(bytes_.begin() + 4, bytes_.end() - 3);
}

void DataBMS::check() const
{
  if (bytes_.empty() || bytes_.front() != 0xdd) {
    throw std::runtime_error("Cannot read data!");
  }
  if (bytes_.empty() || bytes_.back() != 0x77) {
    throw std::runtime_error("Cannot read data!");
  }
}

int DataBMS::get1()
{
  cursor_ += 1;
  return content().at(cursor_);
}

int DataBMS::get2()
{
  cursor_ += 2;
  const auto data = content();
  return (data.at(cursor_ - 1) << 8) + data.at(cursor_);
}

int DataBMS::get2_signed()
{
  const int value = get2();
  return value < 32768 ? value : value - 65536;
}

sensor_msgs::msg::BatteryState DataBMS::get_ros_message()
{
  sensor_msgs::msg::BatteryState msg;
  if (address() != 0x03) {
    return msg;
  }

  const double total_voltage = get2() / 100.0;
  const double current = get2_signed() / 100.0;
  const double residual_capacity = get2() / 100.0;
  const double nominal_capacity = get2() / 100.0;
  get2();  // cycle life
  get2();  // production date
  get2();  // balance status
  get2();  // balance status high
  get2();  // protection status
  const int version = get1();
  (void)version;  // BMS major/minor version, not exposed on BatteryState
  const int capacity_percent = get1();
  get1();  // fet status
  const int num_blocks = get1();
  (void)num_blocks;
  const int num_ntc = get1();
  double first_temperature = 0.0;
  for (int i = 0; i < num_ntc; ++i) {
    const double temperature = get2() / 10.0 - 273.15;
    if (i == 0) {
      first_temperature = temperature;
    }
  }

  msg.voltage = static_cast<float>(total_voltage);
  msg.temperature = static_cast<float>(first_temperature);
  msg.capacity = static_cast<float>(residual_capacity);
  msg.design_capacity = static_cast<float>(nominal_capacity);
  msg.percentage = static_cast<float>(capacity_percent / 100.0);
  msg.current = static_cast<float>(current);
  return msg;
}

}  // namespace fieldfriend_driver
