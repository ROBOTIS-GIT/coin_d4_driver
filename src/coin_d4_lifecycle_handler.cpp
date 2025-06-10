// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Hyeongjun Jeon

#include <string>
#include <memory>

#include "coin_d4_driver/coin_d4_lifecycle_handler.hpp"


namespace robotis
{
namespace coin_d4
{
CoinD4LifecycleHandler::CoinD4LifecycleHandler(
  const std::string parameter_prefix,
  rclcpp_lifecycle::LifecycleNode * node)
: CoinD4BaseHandler(
    parameter_prefix,
    node->get_node_logging_interface(),
    node->get_node_parameters_interface()),
  node_(node)
{
  make_scan_publisher(lidar_general_info_.topic_name);
}

CoinD4LifecycleHandler::~CoinD4LifecycleHandler()
{
  deactivate_scan_publisher();
  if (laser_scan_pub_) {
    laser_scan_pub_.reset();
  }
}

rclcpp::Time CoinD4LifecycleHandler::get_node_time()
{
  return node_->now();
}

void CoinD4LifecycleHandler::make_scan_publisher(const std::string & topic_name)
{
  laser_scan_pub_ =
    node_->create_publisher<sensor_msgs::msg::LaserScan>(
    topic_name,
    rclcpp::SensorDataQoS());
}

void CoinD4LifecycleHandler::publish_scan(std::unique_ptr<sensor_msgs::msg::LaserScan> && scan_msg)
{
  laser_scan_pub_->publish(std::move(scan_msg));
}

void CoinD4LifecycleHandler::activate_scan_publisher()
{
  if (laser_scan_pub_ && !laser_scan_pub_->is_activated()) {
    laser_scan_pub_->on_activate();
  }
}

void CoinD4LifecycleHandler::deactivate_scan_publisher()
{
  if (laser_scan_pub_ && laser_scan_pub_->is_activated()) {
    laser_scan_pub_->on_deactivate();
  }
}
}  // namespace coin_d4
}  // namespace robotis
