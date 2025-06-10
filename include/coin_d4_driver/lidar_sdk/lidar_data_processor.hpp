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

#ifndef COIN_D4_DRIVER__LIDAR_SDK__LIDAR_DATA_PROCESSOR_HPP_
#define COIN_D4_DRIVER__LIDAR_SDK__LIDAR_DATA_PROCESSOR_HPP_

#include <stdint.h>

#include "coin_d4_driver/lidar_sdk/lidar_information.hpp"
#include "coin_d4_driver/lidar_sdk/mtime.hpp"
#include "coin_d4_driver/lidar_sdk/serial_port.hpp"
#include "coin_d4_driver/lidar_sdk/timer.hpp"
#include "coin_d4_driver/lidar_sdk/handling_info.hpp"

class LidarDataProcessor
{
private:
  uint16_t calculated_check_sum_;
  uint16_t target_check_sum_;
  uint16_t sample_numl_and_ct_cal_;
  uint16_t last_sample_angle_calculated_;
  uint16_t check_sum_16_value_holder_;
  uint16_t package_sample_index_;
  uint16_t first_sample_angle_;
  uint16_t last_sample_angle_;
  uint8_t scan_frequency_;
  bool check_sum_result_;
  bool has_package_error_;
  float interval_sample_angle_;
  float interval_sample_angle_last_package_;
  int package_index_;

  float start_t_ = 0;
  float stop_t_ = 0;
  float angle_new_ = 0;
  float angle_bak_ = 0;
  size_t recv_node_count_ = 0;

  uint64_t node_time_ns_;
  uint64_t node_last_time_ns_;
  uint32_t scan_time_increment_;
  size_t buffer_size_ = 0;

  uint8_t * global_recv_buffer_;

  SerialPort * serial_port_;
  LidarTimeStatus * lidar_time_;
  LidarHardwareStatus * lidar_status_;
  LidarGeneralInfo lidar_general_info_;
  LidarPackage scan_packages_;
  uint32_t trans_delay_ = 0;

public:
  LidarDataProcessor(
    LidarTimeStatus * lidar_time,
    LidarHardwareStatus * lidar_status,
    LidarGeneralInfo & lidar_general_info,
    LidarPackage & scan_packages);
  ~LidarDataProcessor();

  void set_serial_port(SerialPort * serial_port);

  int package_sample_bytes_;

  // Send command to lidar
  result_t send_command(uint8_t cmd);
  // Send data to lidar
  result_t send_data(const uint8_t * data, size_t size);
  // Wait for lidar speed adjustment
  result_t wait_speed_right(uint8_t cmd, uint64_t timeout = DEFAULT_TIMEOUT);
  // Receive lidar scan data
  result_t wait_scan_data(
    node_info * nodebuffer, size_t & count,
    uint32_t timeout = DEFAULT_TIMEOUT);
  // Parse received lidar data package
  result_t wait_package(node_info * node, uint32_t timeout = DEFAULT_TIMEOUT);
};

#endif  // COIN_D4_DRIVER__LIDAR_SDK__LIDAR_DATA_PROCESSOR_HPP_
