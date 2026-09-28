""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

from math import cos, sin

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Time
from tf_transformations import quaternion_from_euler


class DataOdom:
    """Implements odometry data."""

    def __init__(self, covariance_pose, covariance_twist, logger):
        self._logger = logger
        self._odom = Odometry()
        self._odom.header.frame_id = 'odom'
        self._odom.child_frame_id = 'base_link'
        self._odom.pose.covariance = covariance_pose
        self._odom.twist.covariance = covariance_twist
        self._last_time = None
        # Track yaw and position as plain floats to avoid costly quaternion
        # (de)composition on every incoming serial message. The quaternion is
        # only assembled when the odometry message is actually published.
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def update_data(self, current_time: Time, linear_speed: float, angular_speed: float):
        """Integrate new speed data into the current pose estimate."""
        if self._last_time is None:
            self._last_time = current_time
        delta_t: Time = current_time - self._last_time
        dt = delta_t.nanoseconds / 1e9

        self._yaw += angular_speed * dt
        self._x += linear_speed * cos(self._yaw) * dt
        self._y += linear_speed * sin(self._yaw) * dt

        self._odom.header.stamp = current_time.to_msg()
        self._odom.twist.twist.linear.x = linear_speed
        self._odom.twist.twist.angular.z = angular_speed
        self._last_time = current_time

    def get_odometry(self) -> Odometry:
        """Return odometry."""
        self._odom.pose.pose.position.x = self._x
        self._odom.pose.pose.position.y = self._y
        quat_tf = quaternion_from_euler(0, 0, self._yaw)
        self._odom.pose.pose.orientation = Quaternion(
            x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3]
        )
        return self._odom

    def get_transform_stamped(self) -> TransformStamped:
        """Return odometry as transform stamped."""
        t = TransformStamped()
        t.header = self._odom.header
        t.child_frame_id = self._odom.child_frame_id
        t.transform.translation.x = self._odom.pose.pose.position.x
        t.transform.translation.y = self._odom.pose.pose.position.y
        t.transform.translation.z = self._odom.pose.pose.position.z
        t.transform.rotation.x = self._odom.pose.pose.orientation.x
        t.transform.rotation.y = self._odom.pose.pose.orientation.y
        t.transform.rotation.z = self._odom.pose.pose.orientation.z
        t.transform.rotation.w = self._odom.pose.pose.orientation.w
        # Send the transformation
        return t
