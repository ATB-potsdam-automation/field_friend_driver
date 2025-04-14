""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

from typing import Dict

from rclpy.node import Node, Parameter
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from fieldfriend_driver.communication.communication import Communication
from fieldfriend_driver.data.data_odom import DataOdom


class EStopButtonHandler:
    """Handle the odometry."""

    def __init__(
            self,
            node: Node, comm: Communication, module_name: str):
        self._node = node
        self._logger = node.get_logger()
        self._clock = node.get_clock()
        self._comm = comm

        param_topic = 'modules.' + module_name + '.topic_name'
        node.declare_parameter(param_topic, Parameter.Type.STRING)
        self._topic_name = node.get_parameter(param_topic).value

        param_topic = 'modules.' + module_name + '.data_name'
        node.declare_parameter(param_topic, Parameter.Type.STRING)
        self._data_name = node.get_parameter(param_topic).value

        # Publisher
        self._publisher = node.create_publisher(Bool, '/emergency_stop/' + self._topic_name, 10)

        comm.register_core_observer(self)

    def publish_estop(self, emergency_stop: bool) -> None:
        """Publish odometry data to ros."""
        self._publisher.publish(Bool(data=emergency_stop))

    def update(self, data: Dict) -> None:
        """Read the data from a list of words."""
        # self._logger.info(f'{data}')

        emergency_stop = not data[self._data_name]
        self.publish_estop(emergency_stop)
