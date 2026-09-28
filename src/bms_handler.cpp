// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#include "fieldfriend_driver/bms_handler.hpp"

#include "fieldfriend_driver/data_bms.hpp"

namespace fieldfriend_driver
{

BMSHandler::BMSHandler(rclcpp::Node * node, Communication & comm)
: logger_(node->get_logger()), clock_(node->get_clock()), comm_(comm)
{
  publisher_ = node->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);

  comm.register_bms_observer(this);

  send_request_timer_ = node->create_wall_timer(
    std::chrono::seconds(1), [this]() {send_request();});
}

void BMSHandler::update(const std::vector<std::string> & words)
{
  std::vector<int> bytes;
  bytes.reserve(words.size());
  try {
    for (const auto & word : words) {
      bytes.push_back(std::stoi(word, nullptr, 16));
    }
    DataBMS data_bms(bytes);
    data_bms.check();
    auto msg = data_bms.get_ros_message();
    msg.header.stamp = clock_->now();
    publisher_->publish(msg);
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger_, "Cannot read data!");
  }
}

void BMSHandler::send_request()
{
  comm_.send("bms.send(0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77)");
}

}  // namespace fieldfriend_driver
