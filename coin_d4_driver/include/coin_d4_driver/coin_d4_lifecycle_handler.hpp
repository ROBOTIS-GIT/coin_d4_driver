// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#ifndef COIN_D4_LIDAR__COIN_D4_LIFECYCLE_HANDLER_HPP_
#define COIN_D4_LIDAR__COIN_D4_LIFECYCLE_HANDLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "coin_d4_driver/coin_d4_base_handler.hpp"


namespace robotis
{
namespace coin_d4
{
class CoinD4LifecycleHandler : public CoinD4BaseHandler
{
public:
  CoinD4LifecycleHandler(
    const std::string parameter_prefix,
    rclcpp_lifecycle::LifecycleNode * node);
  ~CoinD4LifecycleHandler();

private:
  rclcpp::Time get_node_time() final;
  void make_scan_publisher(const std::string & topic_name) final;
  void publish_scan(std::unique_ptr<sensor_msgs::msg::LaserScan> && scan_msg) final;
  void activate_scan_publisher() final;
  void deactivate_scan_publisher() final;
  rclcpp_lifecycle::LifecycleNode * node_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
};
}  // namespace coin_d4
}  // namespace robotis
#endif  // COIN_D4_LIDAR__COIN_D4_LIFECYCLE_HANDLER_HPP_
