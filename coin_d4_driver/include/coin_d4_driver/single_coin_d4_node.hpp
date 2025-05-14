// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#ifndef COIN_D4_LIDAR__SINGLE_COIN_D4_NODE_HPP_
#define COIN_D4_LIDAR__SINGLE_COIN_D4_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "coin_d4_driver/coin_d4_handler.hpp"


namespace robotis
{
namespace coin_d4
{
class SingleCoinD4Node : public rclcpp::Node
{
public:
  explicit SingleCoinD4Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~SingleCoinD4Node();

private:
  std::unique_ptr<CoinD4Handler> handler_;
};
}  // namespace coin_d4
}  // namespace robotis
#endif  // COIN_D4_LIDAR__SINGLE_COIN_D4_NODE_HPP_
