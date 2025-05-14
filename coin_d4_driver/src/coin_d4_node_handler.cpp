// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#include <string>
#include <memory>

#include "coin_d4_driver/coin_d4_node_handler.hpp"


namespace robotis
{
namespace coin_d4
{
CoinD4NodeHandler::CoinD4NodeHandler(const std::string parameter_prefix, rclcpp::Node * node)
: CoinD4BaseHandler(
    parameter_prefix,
    node->get_node_logging_interface(),
    node->get_node_parameters_interface()),
  node_(node)
{
  this->make_scan_publisher(lidar_general_info_.topic_name);
}

CoinD4NodeHandler::~CoinD4NodeHandler()
{
  if (laser_scan_pub_) {
    laser_scan_pub_.reset();
  }
}

rclcpp::Time CoinD4NodeHandler::get_node_time()
{
  return node_->now();
}

void CoinD4NodeHandler::make_scan_publisher(const std::string & topic_name)
{
  laser_scan_pub_ =
    node_->create_publisher<sensor_msgs::msg::LaserScan>(
      topic_name,
      rclcpp::SensorDataQoS());
}

void CoinD4NodeHandler::publish_scan(std::unique_ptr<sensor_msgs::msg::LaserScan> && scan_msg)
{
  laser_scan_pub_->publish(std::move(scan_msg));
}

void CoinD4NodeHandler::activate_scan_publisher()
{
}

void CoinD4NodeHandler::deactivate_scan_publisher()
{
}
}  // namespace coin_d4
}  // namespace robotis
