// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#ifndef COIN_D4_DRIVER__LIDAR_SDK__LIDAR_DATA_PROCESSOR_H_
#define COIN_D4_DRIVER__LIDAR_SDK__LIDAR_DATA_PROCESSOR_H_

#include <stdint.h>

#include "coin_d4_driver/lidar_sdk/lidar_information.h"
#include "coin_d4_driver/lidar_sdk/mtime.h"
#include "coin_d4_driver/lidar_sdk/serial_port.h"
#include "coin_d4_driver/lidar_sdk/timer.h"
#include "coin_d4_driver/lidar_sdk/handling_info.hpp"


class LidarDataProcessor
{
private:
  uint16_t calculated_check_sum_;
  uint16_t target_check_sum_;
  uint16_t sample_numl_and_ct_cal_;
  uint16_t last_sample_angle_calculated_;
  uint16_t Valu8Tou16;
  uint16_t package_Sample_Index; //包采样点索引
  uint16_t FirstSampleAngle;     //起始采样角
  uint16_t last_sample_angle_;      //结束采样角
  uint8_t scan_frequency;	       //协议中雷达转速
  bool CheckSumResult;
  bool has_package_error;
  float interval_sample_angle_;
  float interval_sample_angle_last_package_;
  int package_index;

  float start_t =0;
  float stop_t = 0;
  float angle_new = 0;
  float angle_bak = 0;
  size_t recvNodeCount;

  uint64_t m_node_time_ns;			 //< time stamp
  uint64_t m_node_last_time_ns;  //< time stamp
  uint32_t scan_time_increment;				   //< two laser point time intervals
  size_t buffer_size = 0;

  uint8_t * globalRecvBuffer;

  Serial_Port * serial_port_;
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

  void set_serial_port(Serial_Port * serial_port);

  int PackageSampleBytes;   //一个包包含的激光点数

  /*向激光雷达发布指令*/
  //result_t sendCommand(uint8_t cmd,const void *payload = NULL,size_t payloadsize = 0);
  result_t sendCommand(uint8_t cmd);
  /*向激光雷达写数据*/
  result_t sendData(const uint8_t *data, size_t size);

  /*等待激光雷达调速完成*/
  result_t waitSpeedRight(uint8_t cmd,uint64_t timeout = DEFAULT_TIMEOUT);

  /*接收串口上传的雷达数据*/
  result_t waitScanData(node_info *nodebuffer, size_t &count,uint32_t timeout = DEFAULT_TIMEOUT);

  /*解析接收到的雷达数据点（v1及v2版本）*/
  result_t waitPackage(node_info *node,uint32_t timeout = DEFAULT_TIMEOUT);
};

#endif  // COIN_D4_DRIVER__LIDAR_SDK__LIDAR_DATA_PROCESSOR_H_
