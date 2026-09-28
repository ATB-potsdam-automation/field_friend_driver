// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "fieldfriend_driver/communication.hpp"
#include "fieldfriend_driver/data_odom.hpp"
#include "fieldfriend_driver/module_handler.hpp"

namespace fieldfriend_driver
{

/// Handles the odometry.
class OdomHandler : public ModuleHandler, public CoreObserver
{
public:
  OdomHandler(rclcpp::Node * node, Communication & comm);

  void publish_odom();
  void update(const CoreData & data) override;

private:
  rclcpp::Node * node_;
  rclcpp::Logger logger_;

  bool publish_tf_ = false;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  DataOdom data_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace fieldfriend_driver
