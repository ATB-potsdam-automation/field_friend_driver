// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#include "fieldfriend_driver/configuration_handler.hpp"

#include <fstream>
#include <sstream>

namespace fieldfriend_driver
{

ConfigurationHandler::ConfigurationHandler(
  rclcpp::Node * node, Communication & comm, std::string filename)
: comm_(comm), logger_(node->get_logger()), filename_(std::move(filename))
{
  subscription_ = node->create_subscription<std_msgs::msg::String>(
    "configure", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {handle_configure(msg);});
}

void ConfigurationHandler::handle_configure(const std_msgs::msg::String::SharedPtr /*msg*/)
{
  std::ifstream file(filename_);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Could not open lizard file %s", filename_.c_str());
    return;
  }

  comm_.send("!-");
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    comm_.send("!+" + line);
  }
  comm_.send("!.");
  comm_.send("core.restart()");
}

}  // namespace fieldfriend_driver
