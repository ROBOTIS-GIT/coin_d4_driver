// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#ifndef COIN_D4_LIDAR__MULTI_COIN_D4_NODE_HPP_
#define COIN_D4_LIDAR__MULTI_COIN_D4_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "coin_d4_driver/coin_d4_node_handler.hpp"


namespace robotis
{
namespace coin_d4
{
class MultiCoinD4Node : public rclcpp::Node
{
public:
  explicit MultiCoinD4Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~MultiCoinD4Node();

private:
  std::unordered_map<std::string, std::shared_ptr<CoinD4NodeHandler>> handlers_;
};
}  // namespace coin_d4
}  // namespace robotis
#endif  // COIN_D4_LIDAR__MULTI_COIN_D4_NODE_HPP_
