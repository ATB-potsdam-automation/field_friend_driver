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

        self.declare_parameter('modules', rclpy.Parameter.Type.STRING_ARRAY)
        modules = self.get_parameter('modules').value

        for module in modules:
            if module == "odom_handler":
                self._odom_handler = OdomHandler(self, self._serial_communication)
            elif module == "bms_handler":
                self._bms_handler = BMSHandler(self, self._serial_communication)
            elif module == "twist_handler":
                self._twist_handler = TwistHandler(self, self._serial_communication)
            elif module == "estop_handler":
                self._estop_handler = EStopHandler(self, self._serial_communication)
            elif module == "yaxis_handler":
                self._yaxis_handler = YAxisHandler(self, self._serial_communication)
            elif module == "zaxis_handler":
                self._zaxis_handler = ZAxisHandler(self, self._serial_communication)
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
