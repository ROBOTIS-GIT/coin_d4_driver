// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#include "coin_d4_driver/multi_coin_d4_node.hpp"


namespace robotis
{
namespace coin_d4
{
MultiCoinD4Node::MultiCoinD4Node(const rclcpp::NodeOptions & options)
: Node("lidar_node", options)
{
  this->declare_parameter<std::vector<std::string>>(
    "lidar_list",
    std::vector<std::string>({"lidar_0"}));
  std::vector<std::string> lidar_list;
  this->get_parameter("lidar_list", lidar_list);
  for (const auto & lidar : lidar_list) {
    handlers_[lidar] = std::make_shared<CoinD4Handler>(lidar + ".", this);
  }
  for (auto & handler : handlers_) {
    handler.second->activate_grab_thread();
    handler.second->activate_publish_thread();
  }
}

MultiCoinD4Node::~MultiCoinD4Node()
{
  for (auto & handler : handlers_) {
    handler.second->deactivate_publish_thread();
    handler.second->deactivate_grab_thread();
  }
  for (auto & handler : handlers_) {
    handler.second.reset();
  }
}
}  // namespace coin_d4
}  // namespace robotis

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<robotis::coin_d4::MultiCoinD4Node>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
	return 0;
}
