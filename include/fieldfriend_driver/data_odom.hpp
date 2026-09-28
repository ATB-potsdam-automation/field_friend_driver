// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include <array>
#include <optional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fieldfriend_driver
{

/// Implements odometry integration and message assembly.
class DataOdom
{
public:
  DataOdom() = default;
  DataOdom(const std::array<double, 36> & covariance_pose, const std::array<double, 36> & covariance_twist);

  /// Integrate new speed data into the current pose estimate.
  void update_data(const rclcpp::Time & current_time, double linear_speed, double angular_speed);

  nav_msgs::msg::Odometry get_odometry();
  geometry_msgs::msg::TransformStamped get_transform_stamped() const;

private:
  nav_msgs::msg::Odometry odom_;
  std::optional<rclcpp::Time> last_time_;

  // Track yaw/position as plain floats to avoid quaternion (de)composition on
  // every incoming serial message. The quaternion is only assembled when the
  // odometry message is actually published.
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
};

}  // namespace fieldfriend_driver
