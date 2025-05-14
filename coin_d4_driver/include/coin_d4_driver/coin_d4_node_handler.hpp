// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#ifndef COIN_D4_LIDAR__COIN_D4_NODE_HANDLER_HPP_
#define COIN_D4_LIDAR__COIN_D4_NODE_HANDLER_HPP_

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "coin_d4_driver/coin_d4_base_handler.hpp"


namespace robotis
{
namespace coin_d4
{
class CoinD4NodeHandler : public CoinD4BaseHandler
{
public:
  CoinD4NodeHandler(
    const std::string parameter_prefix,
    rclcpp::Node * node);
  ~CoinD4NodeHandler();

private:
  rclcpp::Time get_node_time() final;
  void make_scan_publisher(const std::string & topic_name) final;
  void publish_scan(std::unique_ptr<sensor_msgs::msg::LaserScan> && scan_msg) final;
  void activate_scan_publisher() final;
  void deactivate_scan_publisher() final;

  rclcpp::Node * node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
};
}  // namespace coin_d4
}  // namespace robotis
#endif  // COIN_D4_LIDAR__COIN_D4_NODE_HANDLER_HPP_
