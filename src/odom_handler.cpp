// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#include "fieldfriend_driver/odom_handler.hpp"

#include <vector>

#include "fieldfriend_driver/param_utils.hpp"

namespace fieldfriend_driver
{

OdomHandler::OdomHandler(rclcpp::Node * node, Communication & comm)
: node_(node), logger_(node->get_logger())
{
  node->declare_parameter("modules.odom.handler.twist_stddev", std::vector<double>(36, 0.0));
  const auto twist_stddev = node->get_parameter("modules.odom.handler.twist_stddev").as_double_array();
  const auto twist_cov = diag_covariance_from_vector(twist_stddev);

  node->declare_parameter("modules.odom.handler.pose_stddev", std::vector<double>(36, 0.0));
  const auto pose_stddev = node->get_parameter("modules.odom.handler.pose_stddev").as_double_array();
  const auto pose_cov = diag_covariance_from_vector(pose_stddev);

  node->declare_parameter("modules.odom.handler.publish_tf", false);
  publish_tf_ = node->get_parameter("modules.odom.handler.publish_tf").as_bool();

  // The esp can report speed updates much faster than any consumer needs
  // odometry. Publishing on every incoming message would waste a lot of
  // CPU on message serialization, so we decouple it with its own timer.
  node->declare_parameter("modules.odom.handler.publish_frequency", 20.0);
  const double publish_frequency = node->get_parameter("modules.odom.handler.publish_frequency").as_double();

  publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  }

  data_ = DataOdom(pose_cov, twist_cov);

  comm.register_core_observer(this);

  publish_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_frequency),
    [this]() {publish_odom();});
}

void OdomHandler::publish_odom()
{
  publisher_->publish(data_.get_odometry());
  if (publish_tf_) {
    tf_broadcaster_->sendTransform(data_.get_transform_stamped());
  }
}

void OdomHandler::update(const CoreData & data)
{
  data_.update_data(
    node_->get_clock()->now(),
    data.at("linear_speed").as_double(),
    data.at("angular_speed").as_double());
}

}  // namespace fieldfriend_driver
