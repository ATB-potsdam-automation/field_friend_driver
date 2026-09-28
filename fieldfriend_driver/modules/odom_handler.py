""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

from typing import Dict

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from fieldfriend_driver.communication.communication import Communication
from fieldfriend_driver.data.data_odom import DataOdom


class OdomHandler:
    """Handle the odometry."""

    def __init__(
            self,
            node: Node, comm: Communication):
        self._node = node
        self._logger = node.get_logger()
        self._clock = node.get_clock()
        self._comm = comm
        self.current_pose = PoseStamped()

        # Red parameter
        node.declare_parameter('modules.odom.handler.twist_stddev', np.zeros(36).tolist())
        twist_stddev = node.get_parameter('modules.odom.handler.twist_stddev')
        twist_cov = np.asarray(np.diag(twist_stddev.value)).reshape(-1)
        self._logger.debug(f'Linear twist convariance {twist_cov}')
        node.declare_parameter('modules.odom.handler.pose_stddev', np.zeros(36).tolist())
        pose_stddev = node.get_parameter('modules.odom.handler.pose_stddev')
        pose_cov = np.asarray(np.diag(pose_stddev.value)).reshape(-1)
        self._logger.debug(f'Linear pose convariance {pose_cov}')
        node.declare_parameter('modules.odom.handler.publish_tf', False)
        self._publish_tf = node.get_parameter('modules.odom.handler.publish_tf').value
        # The esp can report speed updates much faster than any consumer needs
        # odometry. Publishing on every incoming message would waste a lot of
        # CPU on message serialization, so we decouple it with its own timer.
        node.declare_parameter('modules.odom.handler.publish_frequency', 20.0)
        publish_frequency = node.get_parameter('modules.odom.handler.publish_frequency').value

        # Publisher
        self._publisher = node.create_publisher(Odometry, 'odom', 10)
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(node)

        self._data: DataOdom = DataOdom(pose_cov, twist_cov, self._logger)

        comm.register_core_observer(self)

        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._publish_timer = node.create_timer(
            1 / publish_frequency, self.publish_odom, clock=self.steady_clock)

    def publish_odom(self):
        """Publish odometry data to ros."""
        self._publisher.publish(self._data.get_odometry())
        if self._publish_tf:
            self._tf_broadcaster.sendTransform(self._data.get_transform_stamped())

    def update(self, data: Dict) -> None:
        """Read the data from a list of words."""
        # self._logger.info(f'{data}')

        self._data.update_data(
            self._clock.now(),
            data['linear_speed'],
            data['angular_speed'])
