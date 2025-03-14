""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32

from field_friend_driver.communication.communication import Communication


class YAxisHandler:
    """Handle the yaxis of the endeffector."""

    def __init__(
            self,
            node: Node, comm: Communication):
        self._node = node
        self._logger = node.get_logger()
        self._comm = comm

        self._steps_per_m = 667000.67

        self._max_speed = 0.02  # [m/s]

        self._subscription = node.create_subscription(
            Float32, 'yaxis/target_speed', self.speed_callback, 10
        )

    def linear_to_steps(self, linear: float) -> int:
        steps = int((linear * 1000) * self._steps_per_m)
        return steps

    def speed_callback(self, speed_msg: Float32):
        """Implement callback for yaxis speed."""
        speed = speed_msg.data
        if (speed > self._max_speed or speed < -self._max_speed):
            self._logger.warn(
                f'Thresshold speed target of {speed} to {
                    np.sign(speed) * self._max_speed}')
            speed = np.sign(speed) * self._max_speed
        self._comm.send(f'yaxis.speed({speed * self._steps_per_m})')
