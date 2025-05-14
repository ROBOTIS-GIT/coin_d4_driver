// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#include "coin_d4_driver/single_coin_d4_node.hpp"


namespace robotis
{
namespace coin_d4
{
SingleCoinD4Node::SingleCoinD4Node(const rclcpp::NodeOptions & options)
: Node("lidar_node", options)
{
  handler_ = std::make_unique<CoinD4NodeHandler>("", this);
	handler_->activate_grab_thread();
  handler_->activate_publish_thread();
}

SingleCoinD4Node::~SingleCoinD4Node()
{
  handler_->deactivate_publish_thread();
	handler_->deactivate_grab_thread();
	handler_.reset();
}
}  // namespace coin_d4
}  // namespace robotis

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<robotis::coin_d4::SingleCoinD4Node>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
	return 0;
}
