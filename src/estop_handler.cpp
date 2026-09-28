// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#include "fieldfriend_driver/estop_handler.hpp"

#include "rclcpp/qos.hpp"

namespace fieldfriend_driver
{

EStop::EStop(rclcpp::Node * node, const std::string & param_name, EStopHandler * handler)
: handler_(handler)
{
  const std::string param_init_triggered = "modules.estop_handler." + param_name + ".init_triggered";
  node->declare_parameter(param_init_triggered, rclcpp::PARAMETER_BOOL);
  estop_triggered_ = node->get_parameter(param_init_triggered).as_bool();

  const std::string param_software_estop = "modules.estop_handler." + param_name + ".software_estop";
  node->declare_parameter(param_software_estop, rclcpp::PARAMETER_BOOL);
  estop_software_ = node->get_parameter(param_software_estop).as_bool();

  const std::string param_name_key = "modules.estop_handler." + param_name + ".name";
  node->declare_parameter(param_name_key, rclcpp::PARAMETER_STRING);
  estop_name_ = node->get_parameter(param_name_key).as_string();

  const std::string param_topic = "modules.estop_handler." + param_name + ".topic";
  node->declare_parameter(param_topic, rclcpp::PARAMETER_STRING);
  const std::string topic_name = node->get_parameter(param_topic).as_string();

  subscription_ = node->create_subscription<std_msgs::msg::Bool>(
    topic_name, 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {callback(msg);});

  const std::string param_timeout = "modules.estop_handler." + param_name + ".timeout";
  node->declare_parameter(param_timeout, rclcpp::PARAMETER_DOUBLE);
  const double timeout = node->get_parameter(param_timeout).as_double();

  if (timeout > 0.0) {
    timeout_timer_ = node->create_wall_timer(
      std::chrono::duration<double>(timeout), [this]() {timeout_callback();});
  }

  if (estop_triggered_) {
    message_ = "Initial emergency stop of " + estop_name_ + " triggered";
  }
}

void EStop::callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (timeout_timer_) {
    timeout_timer_->cancel();
    timeout_timer_->reset();
  }
  if (!estop_triggered_ && msg->data) {
    message_ = "Emergency stop of " + estop_name_ + " triggered";
    estop_triggered_ = msg->data;
    handler_->callback(this);
  } else if (estop_triggered_ && !msg->data) {
    message_ = "";
    estop_triggered_ = msg->data;
    handler_->callback(this);
  }
}

void EStop::timeout_callback()
{
  if (timeout_timer_) {
    timeout_timer_->cancel();
  }
  message_ = "Emergency stop of " + estop_name_ + " timed out";
  estop_triggered_ = true;
  handler_->callback(this);
}

EStopHandler::EStopHandler(rclcpp::Node * node, Communication & comm)
: logger_(node->get_logger()), comm_(comm)
{
  rclcpp::QoS qos_profile(1);
  qos_profile.transient_local();

  pub_estop_global_ = node->create_publisher<std_msgs::msg::Bool>("/emergency_stop/global", qos_profile);
  pub_message_ = node->create_publisher<std_msgs::msg::String>("/emergency_stop/message", qos_profile);

  node->declare_parameter("modules.estop_handler.estop_list", rclcpp::PARAMETER_STRING_ARRAY);
  const auto estop_name_list = node->get_parameter("modules.estop_handler.estop_list").as_string_array();

  for (const auto & estop_name : estop_name_list) {
    estop_list_.push_back(std::make_unique<EStop>(node, estop_name, this));
  }

  // Set a software estop by default
  check_global_estop();
  std_msgs::msg::Bool global_msg;
  global_msg.data = global_estop_triggered_;
  pub_estop_global_->publish(global_msg);
  send_software_estop(software_estop_triggered_);
}

void EStopHandler::send_software_estop(bool value)
{
  const std::string command = std::string("en3.level(") + (value ? "false" : "true") + ")";
  RCLCPP_INFO(logger_, "Send estop command: %s", command.c_str());
  comm_.send(command);
}

bool EStopHandler::check_global_estop()
{
  const bool last_estop_global = global_estop_triggered_;
  global_estop_triggered_ = false;
  std::string message;
  for (const auto & estop : estop_list_) {
    message += estop->message() + ";";
    if (estop->triggered()) {
      global_estop_triggered_ = true;
    }
  }
  std_msgs::msg::String message_msg;
  message_msg.data = message;
  pub_message_->publish(message_msg);
  return last_estop_global != global_estop_triggered_;
}

bool EStopHandler::check_software_estop()
{
  const bool last_estop_software = software_estop_triggered_;
  software_estop_triggered_ = false;
  for (const auto & estop : estop_list_) {
    if (estop->triggered() && estop->software_estop()) {
      software_estop_triggered_ = true;
    }
  }
  return last_estop_software != software_estop_triggered_;
}

void EStopHandler::callback(EStop * estop)
{
  RCLCPP_INFO(logger_, "Estop %s triggered", estop->name().c_str());

  if (check_global_estop()) {
    std_msgs::msg::Bool global_msg;
    global_msg.data = global_estop_triggered_;
    pub_estop_global_->publish(global_msg);
  }

  if (check_software_estop()) {
    send_software_estop(software_estop_triggered_);
  }
}

}  // namespace fieldfriend_driver
