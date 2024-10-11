#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from app_field_friend.communication.serial_communication import SerialCommunication
from app_field_friend.modules.bms_handler import BMSHandler
from app_field_friend.modules.estop_handler import EStopHandler
from app_field_friend.modules.odom_handler import OdomHandler
from app_field_friend.modules.twist_handler import TwistHandler
from app_field_friend.modules.yaxis_handler import YAxisHandler
from app_field_friend.modules.zaxis_handler import ZAxisHandler
from app_field_friend.modules.configuration_handler import ConfigurationHandler

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

PACKAGE_NAME = 'app_field_friend'


class FieldFriendDriver(Node):
    """Field friend node handler."""

    def __init__(self):
        super().__init__('field_friend_driver_node')

        configuration_filename = get_package_share_directory(
            PACKAGE_NAME) + '/config/startup.liz'

        self._serial_communication = SerialCommunication(self.get_logger())

        self._odom_handler = OdomHandler(self, self._serial_communication)
        self._bms_handler = BMSHandler(self, self._serial_communication)
        self._twist_handler = TwistHandler(self, self._serial_communication)
        self._estop_handler = EStopHandler(self, self._serial_communication)
        self._yaxis_handler = YAxisHandler(self, self._serial_communication)
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
        field_friend_driver = FieldFriendDriver()

        executor = SingleThreadedExecutor()
        executor.add_node(field_friend_driver)

        try:
            executor.spin()
        finally:
            executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
