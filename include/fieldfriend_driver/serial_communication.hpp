// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "fieldfriend_driver/communication.hpp"
#include "fieldfriend_driver/core_value.hpp"

namespace fieldfriend_driver
{

/// Definition of one entry of the `read_data.list` parameter.
struct CoreDataDef
{
  std::string name;
  std::string type;  // "bool", "int" or "double"
  std::size_t pos;
  CoreValue default_value;
};

/// Handles serial communication with the esp running the lizard firmware.
class SerialCommunication : public Communication
{
public:
  explicit SerialCommunication(rclcpp::Node * node);
  ~SerialCommunication() override;

  void send(const std::string & line) override;
  void read() override;

private:
  void init_core_data(rclcpp::Node * node);
  void open_port();
  void enable();

  static int calculate_checksum(const std::string & line);
  static std::string append_checksum(const std::string & line);
  static bool validate_checksum(const std::string & line);

  void handle_core_message(const std::vector<std::string> & words);
  void handle_expander_message(const std::vector<std::string> & words);

  rclcpp::Logger logger_;
  std::string flashing_arguments_;
  int sleep_after_flash_ = 0;

  int fd_ = -1;
  std::mutex mutex_;
  std::string buffer_;

  std::vector<CoreDataDef> core_data_list_;
  CoreData core_data_;
};

}  // namespace fieldfriend_driver
