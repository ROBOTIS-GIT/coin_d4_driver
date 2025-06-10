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

#ifndef COIN_D4_DRIVER__COIN_D4_BASE_HANDLER_HPP_
#define COIN_D4_DRIVER__COIN_D4_BASE_HANDLER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "coin_d4_driver/lidar_sdk/lidar_data_processor.hpp"
#include "coin_d4_driver/lidar_sdk/locker.hpp"
#include "coin_d4_driver/lidar_sdk/mtime.hpp"
#include "coin_d4_driver/lidar_sdk/serial_port.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


namespace robotis
{
namespace coin_d4
{
class CoinD4BaseHandler
{
public:
  CoinD4BaseHandler(
    const std::string parameter_prefix,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_interface,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params_interface);
  virtual ~CoinD4BaseHandler();

  void activate_grab_thread();
  void deactivate_grab_thread();
  void activate_publish_thread();
  void deactivate_publish_thread();

private:
  void init_structs();
  bool init_lidar_port();
  void flush_serial();
  bool judge_lidar_state(bool & wait_speed_right, uint64_t & lidar_status_time);
  bool grab_synchronized_data(LaserScan & outscan);
  result_t check_data_synchronization(uint32_t timeout);
  void parse_lidar_serial_data(LaserScan & outscan);

  template<typename ParameterT>
  auto declare_parameter_once(
    const std::string & name,
    const ParameterT & default_value = rclcpp::ParameterValue(),
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (params_interface_->has_parameter(name) == false) {
      return params_interface_->declare_parameter(
        name,
        rclcpp::ParameterValue(default_value),
        parameter_descriptor
      ).get<ParameterT>();
    }
    return params_interface_->get_parameter(name).get_value<ParameterT>();
  }

  const std::string parameter_prefix_;

  // handling variables
  node_info * scan_node_buf_;
  std::shared_ptr<LidarTimeStatus> lidar_time_;
  std::shared_ptr<LidarHardwareStatus> lidar_status_;

  LidarPackage scan_packages_;

  size_t scan_node_count_ = 0;

  std::shared_ptr<LidarDataProcessor> lidar_data_processor_;
  std::shared_ptr<SerialPort> serial_port_;
  Event data_event_;
  Locker lock_;

  std::thread grab_thread_;
  std::atomic_bool skip_grab_ = {false};

  std::thread publish_thread_;
  std::atomic_bool skip_publish_ = {false};

protected:
  virtual rclcpp::Time get_node_time() = 0;
  virtual void make_scan_publisher(const std::string & topic_name) = 0;
  virtual void publish_scan(std::unique_ptr<sensor_msgs::msg::LaserScan> && scan_msg) = 0;
  virtual void activate_scan_publisher() = 0;
  virtual void deactivate_scan_publisher() = 0;

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_interface_;

  LidarGeneralInfo lidar_general_info_;
};
}  // namespace coin_d4
}  // namespace robotis
#endif  // COIN_D4_DRIVER__COIN_D4_BASE_HANDLER_HPP_
