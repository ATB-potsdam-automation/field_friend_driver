// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#include "fieldfriend_driver/estop_button_handler.hpp"

namespace fieldfriend_driver
{

EStopButtonHandler::EStopButtonHandler(
  rclcpp::Node * node, Communication & comm, const std::string & module_name)
{
  const std::string topic_param = "modules." + module_name + ".topic_name";
  node->declare_parameter(topic_param, rclcpp::PARAMETER_STRING);
  topic_name_ = node->get_parameter(topic_param).as_string();

  const std::string data_param = "modules." + module_name + ".data_name";
  node->declare_parameter(data_param, rclcpp::PARAMETER_STRING);
  data_name_ = node->get_parameter(data_param).as_string();

  publisher_ = node->create_publisher<std_msgs::msg::Bool>("/emergency_stop/" + topic_name_, 10);

  comm.register_core_observer(this);
}

void EStopButtonHandler::publish_estop(bool emergency_stop)
{
  std_msgs::msg::Bool msg;
  msg.data = emergency_stop;
  publisher_->publish(msg);
}

void EStopButtonHandler::update(const CoreData & data)
{
  const bool emergency_stop = !data.at(data_name_).as_bool();
  publish_estop(emergency_stop);
}

}  // namespace fieldfriend_driver
