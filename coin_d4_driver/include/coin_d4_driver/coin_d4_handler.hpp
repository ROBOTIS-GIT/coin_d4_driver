// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#ifndef COIN_D4_LIDAR__COIN_D4_HANDLER_HPP_
#define COIN_D4_LIDAR__COIN_D4_HANDLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "coin_d4_driver/lidar_sdk/lidar_data_processing.h"
#include "coin_d4_driver/lidar_sdk/locker.h"
#include "coin_d4_driver/lidar_sdk/mtime.h"
#include "coin_d4_driver/lidar_sdk/serial_port.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


namespace robotis
{
namespace coin_d4
{
class CoinD4Handler
{
public:
  CoinD4Handler(
    const std::string parameter_prefix,
    rclcpp::Node * node);
  ~CoinD4Handler();

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
    if (node_->has_parameter(name) == false) {
      return node_->declare_parameter(
        name,
        rclcpp::ParameterValue(default_value),
        parameter_descriptor
      ).get<ParameterT>();
    }
    return node_->get_parameter(name).get_value<ParameterT>();
  }

  const std::string parameter_prefix_;
  rclcpp::Node * node_;

  // handling variables
  node_info * scan_node_buf_;
  std::shared_ptr<LidarTimeStatus> lidar_time_;
  std::shared_ptr<LidarHardwareStatus> lidar_status_;

  LidarPackage scan_packages_;
  LidarGeneralInfo lidar_general_info_;

  size_t scan_node_count_ = 0;

  std::shared_ptr<Lidar_Data_Processing> lidar_data_processing_;
  std::shared_ptr<Serial_Port> serial_port_;
  Event data_event_;
  Locker lock_;

  std::thread grab_thread_;
  std::atomic_bool skip_grab_ = {false};

  std::thread publish_thread_;
  std::atomic_bool skip_publish_ = {false};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
};
}  // namespace coin_d4
}  // namespace robotis
#endif  // COIN_D4_LIDAR__COIN_D4_HANDLER_HPP_
