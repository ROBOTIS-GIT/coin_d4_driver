// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#ifndef COIN_D4_DRIVER__LIDAR_SDK__HANDLING_INFO_HPP_
#define COIN_D4_DRIVER__LIDAR_SDK__HANDLING_INFO_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "coin_d4_driver/lidar_sdk/lidar_information.h"
#include "coin_d4_driver/lidar_sdk/timer.h"


struct LidarGeneralInfo
{
  int version = 1;  // lidar version
  std::string port = "/dev/sc_mini";  // serial port name
  int serial_baud_rate = 230400;  // baud rate
  bool intensity_data_flag = false;
  uint64_t scan_time_increment = 1e9 / 5000;
  std::string frame_id = "laser_link";
  uint16_t frequency_max = 103;
  uint16_t frequency_min = 97;
  std::string topic_name = "scan";
  bool reverse = false;
  int warmup_time = 0;
};

// LiDAR package info
struct LidarPackage
{
  node_package package;
  node_packages packages;
  node_package_coin package_coin;
};

// LiDAR time info
struct LidarTimeStatus
{
  uint64_t system_start_time = 0;
  uint64_t tim_scan_start = 0;
  uint64_t tim_scan_end = 0;
  uint64_t scan_start_time = 0;
  uint64_t scan_end_time = 0;
  uint64_t lidar_frequency_abnormal_time = 0;
};

// LiDAR hardware status
struct LidarHardwareStatus
{
  bool serial_connected = false;
  bool lidar_ready = false;
  bool lidar_last_status =false;
  bool close_lidar = true;
  bool lidar_trap_restart = false;

  bool lidar_restart_try = false;

  uint8_t lidar_abnormal_state = 0;
};
#endif  // COIN_D4_DRIVER__LIDAR_SDK__HANDLING_INFO_HPP_
