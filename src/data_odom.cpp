// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#include "fieldfriend_driver/data_odom.hpp"

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"

namespace fieldfriend_driver
{

DataOdom::DataOdom(
  const std::array<double, 36> & covariance_pose,
  const std::array<double, 36> & covariance_twist)
{
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_link";
  odom_.pose.covariance = covariance_pose;
  odom_.twist.covariance = covariance_twist;
}

void DataOdom::update_data(const rclcpp::Time & current_time, double linear_speed, double angular_speed)
{
  if (!last_time_.has_value()) {
    last_time_ = current_time;
  }
  const double dt = (current_time - last_time_.value()).seconds();

  yaw_ += angular_speed * dt;
  x_ += linear_speed * std::cos(yaw_) * dt;
  y_ += linear_speed * std::sin(yaw_) * dt;

  odom_.header.stamp = current_time;
  odom_.twist.twist.linear.x = linear_speed;
  odom_.twist.twist.angular.z = angular_speed;
  last_time_ = current_time;
}

nav_msgs::msg::Odometry DataOdom::get_odometry()
{
  odom_.pose.pose.position.x = x_;
  odom_.pose.pose.position.y = y_;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();

  return odom_;
}

geometry_msgs::msg::TransformStamped DataOdom::get_transform_stamped() const
{
  geometry_msgs::msg::TransformStamped t;
  t.header = odom_.header;
  t.child_frame_id = odom_.child_frame_id;
  t.transform.translation.x = odom_.pose.pose.position.x;
  t.transform.translation.y = odom_.pose.pose.position.y;
  t.transform.translation.z = odom_.pose.pose.position.z;
  t.transform.rotation.x = odom_.pose.pose.orientation.x;
  t.transform.rotation.y = odom_.pose.pose.orientation.y;
  t.transform.rotation.z = odom_.pose.pose.orientation.z;
  t.transform.rotation.w = odom_.pose.pose.orientation.w;
  return t;
}

}  // namespace fieldfriend_driver
