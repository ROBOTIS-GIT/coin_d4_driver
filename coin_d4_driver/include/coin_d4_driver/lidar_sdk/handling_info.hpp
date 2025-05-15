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
  int m_SerialBaudrate = 230400;  // baud rate
  bool m_intensities = false;
  uint64_t m_PointTime = 1e9/5000;
  uint32_t trans_delay = 0;
  std::string frame_id = "laser_link";
  uint16_t frequency_max = 103;
  uint16_t frequency_min = 97;
  std::string topic_name = "scan";
  bool reverse = false;
  int warmup_time = 0;
};

// LiDAR block judgment
struct LidarBlockInfo
{
  uint16_t point_check = 0;
  uint16_t point_check_part = 0;
  uint16_t point_check_all = 0;
  int blocked_size = 0;
  int lidar_zero_count = 0;
  bool blocked_judge = true;
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
  uint32_t scan_time_t = 0;
  uint64_t system_start_time = 0;
  uint64_t tim_scan_start = 0;
  uint64_t tim_scan_end = 0;
  uint64_t scan_start_time = 0;
  uint64_t scan_end_time = 0;
  uint64_t lidar_frequence_abnormal_time = 0;
};

// LiDAR hardware status
struct LidarHardwareStatus
{
  bool FilterEnable = true;           // 是否需要滤波
  bool GyroCompensateEnable = false; // 是否要做旋转角度补偿
  bool isConnected = false;          // 串口连接状体
  bool slam_user = false;
  bool encry_lidar = false;
  bool optimize_enable = true;
  bool lidar_cover = false;
  bool disable_encry = false;
  bool able_exposure = false;
  bool low_exposure = false;
  bool lidar_ready = false;
  bool lidar_last_status =false;
  bool close_lidar = true;
  bool lidar_trap_restart = false;    //雷达卡住重启状态

  bool lidar_restart_try = false;

  uint8_t lidar_abnormal_state = 0;
};
#endif  // COIN_D4_DRIVER__LIDAR_SDK__HANDLING_INFO_HPP_
