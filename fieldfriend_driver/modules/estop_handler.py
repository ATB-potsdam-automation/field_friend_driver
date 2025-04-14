""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

from rclpy.node import Node, Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from fieldfriend_driver.communication.communication import Communication


class EStop:
    def __init__(
            self,
            node: Node,
            param_name: str,
            handler_callback=None):

        self._handler_callback = handler_callback

        param_init_triggered = 'modules.estop_handler.' + param_name + '.init_triggered'
        node.declare_parameter(param_init_triggered, Parameter.Type.BOOL)
        self._estop_triggered = node.get_parameter(param_init_triggered).value

        param_software_estop = 'modules.estop_handler.' + param_name + '.software_estop'
        node.declare_parameter(param_software_estop, Parameter.Type.BOOL)
        self._estop_software = node.get_parameter(param_software_estop).value

        param_topic = 'modules.estop_handler.' + param_name + '.name'
        node.declare_parameter(param_topic, Parameter.Type.STRING)
        self._estop_name = node.get_parameter(param_topic).value

        param_topic = 'modules.estop_handler.' + param_name + '.topic'
        node.declare_parameter(param_topic, Parameter.Type.STRING)
        topic_name = node.get_parameter(param_topic).value
        self._sub_estop = node.create_subscription(
            Bool, topic_name, self.callback, 10
        )

        param_topic = 'modules.estop_handler.' + param_name + '.timeout'
        node.declare_parameter(param_topic, Parameter.Type.DOUBLE)
        timeout = node.get_parameter(param_topic).value
        if timeout > 0.0:
            self._timeout_timer = node.create_timer(
                timeout, self.timeout_callback
            )
        else:
            self._timeout_timer = None

        self._logger = node.get_logger()
        if self._estop_triggered:
            self._message = f"Initial emergency stop of {self._estop_name} triggered"
        else:
            self._message = ""

    def callback(self, msg: Bool):
        """Implement a callback for the estop."""
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer.reset()
        if not self._estop_triggered and msg.data:
            self._message = f"Emergency stop of {self._estop_name} triggered"
            self._estop_triggered = msg.data
            self._handler_callback(self)
        elif self._estop_triggered and not msg.data:
            self._message = ""
            self._estop_triggered = msg.data
            self._handler_callback(self)

    def timeout_callback(self):
        """Implement a timeout callback for the estop."""
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
        self._message = f"Emergency stop of {self._estop_name} timed out"
        self._estop_triggered = True
        self._handler_callback(self)


class EStopHandler:
    """Handle the estop."""

    def __init__(self, node: Node, comm: Communication):
        self._node = node
        self._logger = node.get_logger()
        self._comm = comm
        self._software_estop_triggered: bool = True
        self._global_estop_triggered: bool = True

        # Make topic latched
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self._pub_estop_global = node.create_publisher(
            Bool, '/emergency_stop/global', qos_profile
        )
        self._pub_message = node.create_publisher(
            String, '/emergency_stop/message', qos_profile
        )

        node.declare_parameter("modules.estop_handler.estop_list", Parameter.Type.STRING_ARRAY)
        estop_name_list = node.get_parameter("modules.estop_handler.estop_list").value

        self._estop_list = []
        for estop_name in estop_name_list:
            self._estop_list.append(EStop(node, estop_name, self.callback))

        # Set a software estop by default
        self.check_global_estop()
        self._pub_estop_global.publish(Bool(data=self._global_estop_triggered))
        self.send_software_estop(self._software_estop_triggered)

    def send_software_estop(self, value: bool):
        """Send estop command."""
        command = f"en3.level({'false' if value else 'true'})"
        self._logger.info(f"Send estop command: {command}")
        self._comm.send(command)

    def check_global_estop(self):
        """Check if the global estop is triggered. Return True if the state changed."""
        last_estop_global = self._global_estop_triggered
        self._global_estop_triggered = False
        message = ""
        # Check if any estop is triggered
        for estop in self._estop_list:
            message += estop._message + ";"
            if estop._estop_triggered:
                self._global_estop_triggered = True
        self._pub_message.publish(String(data=message))
        return last_estop_global != self._global_estop_triggered

    def check_software_estop(self):
        """Check if the global estop is triggered. Return True if the state changed."""
        last_estop_software = self._software_estop_triggered

        self._software_estop_triggered = False
        # Check if any software estop is triggered
        for estop in self._estop_list:
            if estop._estop_triggered and estop._estop_software:
                self._software_estop_triggered = True
        return last_estop_software != self._software_estop_triggered

    def callback(self, estop: EStop):
        self._logger.info(f"Estop {estop._estop_name} triggered")

        # Check if the global estop is triggered
        # If the state of the global estop changed, publish on the topic
        if self.check_global_estop():
            self._pub_estop_global.publish(Bool(data=self._global_estop_triggered))

        # Check if the software estop is triggered
        # If the state of the software estop changed, send the command
        if self.check_software_estop():
            self.send_software_estop(self._software_estop_triggered)
