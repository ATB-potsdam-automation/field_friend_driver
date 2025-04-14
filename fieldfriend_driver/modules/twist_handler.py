""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

from geometry_msgs.msg import Twist
from rclpy.node import Node, Parameter

from fieldfriend_driver.communication.communication import Communication


class TwistHandler:
    """Handle the odometry."""

    def __init__(
            self,
            node: Node, comm: Communication):

        self._node = node
        self._comm = comm
        self._logger = node.get_logger()

        self._twist = Twist()

        node.declare_parameter('modules.twist_handler.twist_timeout', Parameter.Type.DOUBLE)
        twist_timeout = node.get_parameter('modules.twist_handler.twist_timeout').value

        node.declare_parameter('modules.twist_handler.send_twist_frequency', Parameter.Type.DOUBLE)
        send_twist_frequency = node.get_parameter(
            'modules.twist_handler.send_twist_frequency').value

        self.cmd_subscription = node.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10
        )

        self._send_twist_timer = node.create_timer(1 / send_twist_frequency, self.send_twist)
        self._twist_timeout_timer = node.create_timer(
            twist_timeout, self.twist_timeout, autostart=False)

    def send(self) -> str:
        """Send message to serial port."""
        return f'wheels.speed({self._twist.linear.x:3f}, {self._twist.angular.z:.3f})'

    def update(self, data: Twist) -> None:
        """Read the data from a list of words."""
        # self._logger.info(f'{data}')
        self._twist = data

    def cmd_callback(self, cmd_msg: Twist):
        """Implement callback for cmd_vel message."""
        self._twist_timeout_timer.cancel()
        self._twist_timeout_timer.reset()
        self.update(cmd_msg)

    def send_twist(self):
        """Send twist message to serial device."""
        self._comm.send(self.send())

    def twist_timeout(self):
        """Handle timeout of the twist message."""
        self._logger.warning('Twist timeout. Stopping robot.')
        self._twist_timeout_timer.cancel()
        self._twist = Twist()
