// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#include "fieldfriend_driver/fieldfriend_driver_node.hpp"

#include "fieldfriend_driver/bms_handler.hpp"
#include "fieldfriend_driver/configuration_handler.hpp"
#include "fieldfriend_driver/estop_button_handler.hpp"
#include "fieldfriend_driver/estop_handler.hpp"
#include "fieldfriend_driver/odom_handler.hpp"
#include "fieldfriend_driver/twist_handler.hpp"
#include "fieldfriend_driver/yaxis_handler.hpp"
#include "fieldfriend_driver/zaxis_handler.hpp"

namespace fieldfriend_driver
{

FieldfriendDriver::FieldfriendDriver()
: rclcpp::Node("fieldfriend_driver_node")
{
  declare_parameter("lizard_file", rclcpp::PARAMETER_STRING);
  const std::string configuration_filename = get_parameter("lizard_file").as_string();

  RCLCPP_INFO(get_logger(), "Load lizard file at %s", configuration_filename.c_str());

  serial_communication_ = std::make_unique<SerialCommunication>(this);

  declare_parameter("modules.module_list", rclcpp::PARAMETER_STRING_ARRAY);
  const auto modules = get_parameter("modules.module_list").as_string_array();

  for (const auto & module : modules) {
    const std::string param_name = "modules." + module + ".type";
    declare_parameter(param_name, rclcpp::PARAMETER_STRING);
    const std::string type_name = get_parameter(param_name).as_string();

    if (type_name == "odom_handler") {
      module_handlers_.push_back(std::make_unique<OdomHandler>(this, *serial_communication_));
    } else if (type_name == "bms_handler") {
      module_handlers_.push_back(std::make_unique<BMSHandler>(this, *serial_communication_));
    } else if (type_name == "twist_handler") {
      module_handlers_.push_back(std::make_unique<TwistHandler>(this, *serial_communication_));
    } else if (type_name == "estop_handler") {
      module_handlers_.push_back(std::make_unique<EStopHandler>(this, *serial_communication_));
    } else if (type_name == "yaxis_handler") {
      module_handlers_.push_back(std::make_unique<YAxisHandler>(this, *serial_communication_));
    } else if (type_name == "zaxis_handler") {
      module_handlers_.push_back(std::make_unique<ZAxisHandler>(this, *serial_communication_));
    } else if (type_name == "estop_button_handler") {
      module_handlers_.push_back(
        std::make_unique<EStopButtonHandler>(this, *serial_communication_, module));
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown module type: %s", type_name.c_str());
    }
  }

  configuration_handler_ = std::make_unique<ConfigurationHandler>(
    this, *serial_communication_, configuration_filename);

  read_timer_ = create_wall_timer(
    std::chrono::duration<double>(0.05), [this]() {read_data();});
}

void FieldfriendDriver::read_data()
{
  serial_communication_->read();
}

}  // namespace fieldfriend_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  {
    auto fieldfriend_driver = std::make_shared<fieldfriend_driver::FieldfriendDriver>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(fieldfriend_driver);
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}
