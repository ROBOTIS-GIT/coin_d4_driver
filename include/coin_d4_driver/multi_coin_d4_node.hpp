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

#ifndef COIN_D4_DRIVER__MULTI_COIN_D4_NODE_HPP_
#define COIN_D4_DRIVER__MULTI_COIN_D4_NODE_HPP_

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
#endif  // COIN_D4_DRIVER__MULTI_COIN_D4_NODE_HPP_
