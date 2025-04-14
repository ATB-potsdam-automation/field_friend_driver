#!/usr/bin/env python3

""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from fieldfriend_driver.communication.serial_communication import SerialCommunication
from fieldfriend_driver.modules.bms_handler import BMSHandler
from fieldfriend_driver.modules.configuration_handler import ConfigurationHandler
from fieldfriend_driver.modules.estop_button_handler import EStopButtonHandler
from fieldfriend_driver.modules.estop_handler import EStopHandler
from fieldfriend_driver.modules.odom_handler import OdomHandler
from fieldfriend_driver.modules.twist_handler import TwistHandler
from fieldfriend_driver.modules.yaxis_handler import YAxisHandler
from fieldfriend_driver.modules.zaxis_handler import ZAxisHandler


class FieldfriendDriver(Node):
    """Field friend node handler."""

    def __init__(self):
        super().__init__('fieldfriend_driver_node')

        self.declare_parameter('lizard_file', rclpy.Parameter.Type.STRING)
        # Get the parameter
        configuration_filename = self.get_parameter('lizard_file').value

        self.get_logger().info(f'Load lizard file at {configuration_filename}')

        self._serial_communication = SerialCommunication(self)

        self.declare_parameter('modules.module_list', rclpy.Parameter.Type.STRING_ARRAY)
        modules = self.get_parameter('modules.module_list').value

        self._module_handler = []
        for module in modules:
            param_name = 'modules.' + module + '.type'
            self.declare_parameter(param_name, rclpy.Parameter.Type.STRING)
            type_name = self.get_parameter(param_name).value
            if type_name == "odom_handler":
                self._module_handler.append(OdomHandler(self, self._serial_communication))
            elif type_name == "bms_handler":
                self._module_handler.append(BMSHandler(self, self._serial_communication))
            elif type_name == "twist_handler":
                self._module_handler.append(TwistHandler(self, self._serial_communication))
            elif type_name == "estop_handler":
                self._module_handler.append(EStopHandler(self, self._serial_communication))
            elif type_name == "yaxis_handler":
                self._module_handler.append(YAxisHandler(self, self._serial_communication))
            elif type_name == "zaxis_handler":
                self._module_handler.append(ZAxisHandler(self, self._serial_communication))
            elif type_name == "estop_button_handler":
                self._module_handler.append(EStopButtonHandler(
                    self, self._serial_communication, module))
            else:
                self.get_logger().error(f"Unknown module type: {type_name}")
        self._configuration_handler = ConfigurationHandler(
            self, self._serial_communication, configuration_filename)

        self.read_timer = self.create_timer(0.05, self.read_data)

    def read_data(self):
        """Read data from the serial communication."""
        self._serial_communication.read()


def main(args=None):
    """Implmenets main function call."""
    rclpy.init(args=args)

    try:
        fieldfriend_driver = FieldfriendDriver()

        executor = SingleThreadedExecutor()
        executor.add_node(fieldfriend_driver)

        try:
            executor.spin()
        finally:
            executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
