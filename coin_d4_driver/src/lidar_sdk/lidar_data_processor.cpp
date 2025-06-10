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

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <thread>

#include "coin_d4_driver/lidar_sdk/lidar_data_processor.hpp"
#include "coin_d4_driver/lidar_sdk/mtime.hpp"


LidarDataProcessor::LidarDataProcessor(
  LidarTimeStatus * lidar_time,
  LidarHardwareStatus * lidar_status,
  LidarGeneralInfo & lidar_general_info,
  LidarPackage & scan_packages)
: lidar_time_(lidar_time), lidar_status_(lidar_status), lidar_general_info_(lidar_general_info),
  scan_packages_(scan_packages)
{
  calculated_check_sum_ = 0;
  target_check_sum_ = 0;
  sample_numl_and_ct_cal_ = 0;
  last_sample_angle_calculated_ = 0;
  check_sum_16_value_holder_ = 0;
  package_sample_index_ = 0;
  first_sample_angle_ = 0;
  last_sample_angle_ = 0;
  scan_frequency_ = 0;
  check_sum_result_ = false;
  has_package_error_ = false;
  interval_sample_angle_ = 0.0f;
  interval_sample_angle_last_package_ = 0.0;
  package_sample_bytes_ = 2;
  package_index_ = 0;
  recv_node_count_ = 0;
  start_t_ = 0;

  global_recv_buffer_ = new uint8_t[sizeof(node_packages)];
}

LidarDataProcessor::~LidarDataProcessor()
{
  if (global_recv_buffer_) {
    delete[] global_recv_buffer_;
    global_recv_buffer_ = nullptr;
  }
}

void LidarDataProcessor::set_serial_port(SerialPort * serial_port)
{
  serial_port_ = serial_port;
  trans_delay_ = serial_port_->getByteTime();
}

result_t LidarDataProcessor::send_command(uint8_t cmd)
{
  uint8_t pkt_header[10];

  switch (cmd) {
    case 0x60:
      break;
    case 0x65:
      break;
    case 0x63:
      pkt_header[0] = 0xAA;
      pkt_header[1] = 0x55;
      pkt_header[2] = 0xF0;
      pkt_header[3] = 0x0F;
      serial_port_->write_data(pkt_header, 4);
      break;
    default:
      break;
  }

  return 0;
}

result_t LidarDataProcessor::send_data(const uint8_t * data, size_t size)
{
  if (data == nullptr || size == 0) {
    return RESULT_FAIL;
  }

  size_t written_size;

  while (size) {
    written_size = serial_port_->write_data(data, size);

    if (written_size < 1) {
      return RESULT_FAIL;
    }

    size -= written_size;
    data += written_size;
  }
  return RESULT_OK;
}

result_t LidarDataProcessor::wait_speed_right(uint8_t /*cmd*/, uint64_t timeout)
{
  int recv_pos = 0;
  uint32_t start_ts = get_milliseconds();
  uint8_t recv_buffer[100];
  uint32_t wait_time = 0;
  uint16_t check_sum_cal = 0;
  uint16_t data_check_sum = 0;
  bool head_right = false;
  uint16_t data_length = 0;

  while ((wait_time = get_milliseconds() - start_ts) <= timeout) {
    size_t remain_size = 9;
    size_t recv_size = 0;
    result_t ans = serial_port_->waitForData(remain_size, timeout - wait_time, &recv_size);
    if (!IS_OK(ans)) {
      return ans;
    }
    if (recv_size >= remain_size) {
      recv_size = remain_size - recv_pos;
    }
    ans = serial_port_->read_data(recv_buffer, recv_size);
    if (IS_FAIL(ans)) {
      printf("read waitResponseHeader fail\n");
      return RESULT_FAIL;
    }
    for (size_t pos = 0; pos < recv_size; ++pos) {
      uint8_t current_byte = recv_buffer[pos];
      switch (recv_pos) {
        case 0:
          if (current_byte == 0xFA) {
            printf("head_speed_000=%x\n", current_byte);
          } else {
            continue;
          }
          break;
        case 1:
          if (current_byte == 0xFA) {
            printf("head_speed_111=%x\n", current_byte);
          } else {
            continue;
          }
          break;
        case 2:
          if (current_byte == 0xA5) {
            printf("head_speed_222=%x\n", current_byte);
            data_check_sum += current_byte;
          } else {
            continue;
          }
          break;
        case 3:
          if (current_byte == 0x5A) {
            printf("head_speed_333=%x\n", current_byte);
            data_check_sum += current_byte;
          } else {
            continue;
          }
          break;
        case 4:
          data_length = current_byte;
          data_check_sum += current_byte;
          printf("head_speed_444=%x\n", current_byte);
          break;
        case 5:
          data_length += (current_byte * 0x100);
          data_check_sum += current_byte;
          printf("head_speed_555=%x\n", current_byte);
          break;
        case 6:
          check_sum_cal = current_byte;
          printf("head_speed_666=%x\n", current_byte);
          break;
        case 7:
          check_sum_cal += (current_byte * 0x100);
          printf("head_speed_777=%x\n", current_byte);
          break;
        case 8:
          if (current_byte == 0x01) {
            printf("head_speed_888=%x count=%d\n", current_byte, recv_pos);
            data_check_sum += current_byte;
            head_right = true;
          }
        default:
          break;
      }
      recv_pos++;
      if (head_right) {
        printf("111\n");
        break;
      }
    }
    printf("recv_pos---=%d\n", recv_pos);
    if (recv_pos == 9) {
      printf("222\n");
      break;
    }
  }

  if (recv_pos == 9) {
    printf("333\n");
    start_ts = get_milliseconds();
    recv_pos = 0;
    while ((wait_time = get_milliseconds() - start_ts) <= timeout) {
      size_t remain_size = data_length + 1;
      size_t recv_size = 0;
      result_t ans = serial_port_->waitForData(remain_size, timeout - wait_time, &recv_size);

      if (!IS_OK(ans)) {
        return ans;
      }
      if (recv_size > remain_size) {
        recv_size = remain_size;
      }
      serial_port_->read_data(recv_buffer, recv_size);
      for (size_t pos = 0; pos < 20; ++pos) {
        data_check_sum += recv_buffer[pos];
      }
      if (check_sum_cal == data_check_sum) {
        printf("------TRUE\n");
        return RESULT_OK;
      } else {
        printf("------WRONG\n");
      }
    }
  }
  return RESULT_FAIL;
}

result_t LidarDataProcessor::wait_package(node_info * node, uint32_t timeout)
{
  if (!serial_port_) {
    return RESULT_FAIL;
  }
  int recv_pos = 0;
  uint32_t start_ts = get_milliseconds();
  uint32_t wait_time = 0;
  uint8_t * package_buffer = lidar_general_info_.intensity_data_flag ?
    reinterpret_cast<uint8_t *>(&scan_packages_.package.package_Head) :
    reinterpret_cast<uint8_t *>(&scan_packages_.packages.package_Head);
  uint8_t package_sample_num = 0;
  int32_t angle_correct_for_distance = 0;
  int package_recv_pos = 0;
  uint8_t package_type = 0;

  if (package_sample_index_ == 0) {
    recv_pos = 0;
    while ((wait_time = get_milliseconds() - start_ts) <= timeout) {
      size_t remain_size = PackagePaidBytes - recv_pos;
      size_t recv_size = 0;

      result_t ans = serial_port_->waitForData(remain_size, timeout - wait_time, &recv_size);

      if (!IS_OK(ans)) {
        return ans;
      }

      if (recv_size > remain_size) {
        recv_size = remain_size;
      }

      serial_port_->read_data(global_recv_buffer_, recv_size);

      for (size_t pos = 0; pos < recv_size; ++pos) {
        uint8_t current_byte = global_recv_buffer_[pos];
        switch (recv_pos) {
          case 0:
            if (current_byte == (PH & 0xFF)) {
            } else {
              continue;
            }
            break;
          case 1:
            calculated_check_sum_ = PH;
            if (current_byte == (PH >> 8)) {
            } else {
              recv_pos = 0;
              continue;
            }
            break;
          case 2:
            sample_numl_and_ct_cal_ = current_byte;
            package_type = current_byte & 0x01;
            if ((package_type == CT_Normal) || (package_type == CT_RingStart)) {
              if (package_type == CT_RingStart) {
                if (lidar_time_->tim_scan_start == 0) {
                  lidar_time_->tim_scan_start = getTime();
                } else {
                  lidar_time_->tim_scan_end = getTime();
                }
                scan_frequency_ = (current_byte & 0xFE) >> 1;
              }
            } else {
              has_package_error_ = true;
              recv_pos = 0;
              continue;
            }
            break;
          case 3:
            sample_numl_and_ct_cal_ += (current_byte * 0x100);
            package_sample_num = current_byte;
            break;
          case 4:
            if (current_byte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
              first_sample_angle_ = current_byte;
            } else {
              has_package_error_ = true;
              recv_pos = 0;
              continue;
            }
            break;
          case 5:
            first_sample_angle_ += current_byte * 0x100;
            calculated_check_sum_ ^= first_sample_angle_;
            first_sample_angle_ = first_sample_angle_ >> 1;
            break;
          case 6:
            if (current_byte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
              last_sample_angle_ = current_byte;
            } else {
              has_package_error_ = true;
              recv_pos = 0;
              continue;
            }
            break;
          case 7:
            last_sample_angle_ = current_byte * 0x100 + last_sample_angle_;
            last_sample_angle_calculated_ = last_sample_angle_;
            last_sample_angle_ = last_sample_angle_ >> 1;

            if (package_sample_num == 1) {
              interval_sample_angle_ = 0.0f;
            } else {
              if (last_sample_angle_ < first_sample_angle_) {
                if ((first_sample_angle_ > 270 * 64) && (last_sample_angle_ < 90 * 64)) {
                  interval_sample_angle_ =
                    static_cast<float>(360 * 64 + last_sample_angle_ - first_sample_angle_) /
                    static_cast<float>(package_sample_num - 1);
                  interval_sample_angle_last_package_ = interval_sample_angle_;
                } else {
                  interval_sample_angle_ = interval_sample_angle_last_package_;
                }
              } else {
                interval_sample_angle_ =
                  static_cast<float>(last_sample_angle_ - first_sample_angle_) /
                  static_cast<float>(package_sample_num - 1);
                interval_sample_angle_last_package_ = interval_sample_angle_;
              }
            }
            break;
          case 8:
            target_check_sum_ = current_byte;
            break;
          case 9:
            target_check_sum_ += (current_byte * 0x100);
            break;
        }
        package_buffer[recv_pos++] = current_byte;
      }

      if (recv_pos == PackagePaidBytes) {
        package_recv_pos = recv_pos;
        break;
      }
    }

    if (PackagePaidBytes == recv_pos) {
      start_ts = get_milliseconds();
      recv_pos = 0;

      while ((wait_time = get_milliseconds() - start_ts) <= timeout) {
        size_t remain_size = package_sample_num * package_sample_bytes_ - recv_pos;
        size_t recv_size = 0;
        result_t ans = serial_port_->waitForData(remain_size, timeout - wait_time, &recv_size);

        if (!IS_OK(ans)) {
          return ans;
        }

        if (recv_size > remain_size) {
          recv_size = remain_size;
        }

        serial_port_->read_data(global_recv_buffer_, recv_size);

        for (size_t pos = 0; pos < recv_size; ++pos) {
          if (lidar_general_info_.intensity_data_flag) {
            if (recv_pos % 3 == 2) {
              check_sum_16_value_holder_ += global_recv_buffer_[pos] * 0x100;
              calculated_check_sum_ ^= check_sum_16_value_holder_;
            } else if (recv_pos % 3 == 1) {
              check_sum_16_value_holder_ = global_recv_buffer_[pos];
            } else {
              check_sum_16_value_holder_ = global_recv_buffer_[pos];
              check_sum_16_value_holder_ += 0x00 * 0x100;
              calculated_check_sum_ ^= global_recv_buffer_[pos];
            }
          } else {
            if (recv_pos % 2 == 1) {
              check_sum_16_value_holder_ += global_recv_buffer_[pos] * 0x100;
              calculated_check_sum_ ^= check_sum_16_value_holder_;
            } else {
              check_sum_16_value_holder_ = global_recv_buffer_[pos];
            }
          }
          package_buffer[package_recv_pos + recv_pos] = global_recv_buffer_[pos];
          recv_pos++;
        }

        if (package_sample_num * package_sample_bytes_ == recv_pos) {
          package_recv_pos += recv_pos;
          break;
        }
      }

      if (package_sample_num * package_sample_bytes_ != recv_pos) {
        return RESULT_FAIL;
      }
    } else {
      return RESULT_FAIL;
    }

    calculated_check_sum_ ^= sample_numl_and_ct_cal_;
    calculated_check_sum_ ^= last_sample_angle_calculated_;
    if (calculated_check_sum_ != target_check_sum_) {
      printf(
        "data check, %x, %x, %d, %d\n",
        calculated_check_sum_,
        target_check_sum_,
        package_sample_num,
        recv_pos);
      check_sum_result_ = false;
      has_package_error_ = true;
    } else {
      check_sum_result_ = true;
    }
  }

  uint8_t package_ct;
  if (lidar_general_info_.intensity_data_flag) {
    package_ct = scan_packages_.package.package_CT;
  } else {
    package_ct = scan_packages_.packages.package_CT;
  }

  node->scan_frequency = 0;

  if ((package_ct & 0x01) == CT_Normal) {
    node->sync_flag = Node_NotSync;
    memset(node->debug_info, 0xff, sizeof(node->debug_info));

    if (!has_package_error_) {
      if (package_index_ < 10) {
        node->debug_info[package_index_] = (package_ct >> 1);
        node->index = package_index_;
      } else {
        node->index = 0xff;
      }
      if (package_sample_index_ == 0) {
        package_index_++;
      }
    } else {
      node->index = 255;
      package_index_ = 0;
    }
  } else {
    node->sync_flag = Node_Sync;
    node->index = 255;
    package_index_ = 0;

    if (check_sum_result_) {
      has_package_error_ = false;
      node->scan_frequency = scan_frequency_;
    }
  }

  node->sync_quality = Node_Default_Quality;
  node->stamp = 0;

  if (check_sum_result_) {
    if (lidar_general_info_.intensity_data_flag) {
      node->distance_q2 =
        (scan_packages_.package.packageSampleDistance[package_sample_index_ * 3 + 2] * 64) +
        (scan_packages_.package.packageSampleDistance[package_sample_index_ * 3 + 1] >> 2);
      node->sync_quality =
        (scan_packages_.package.packageSampleDistance[package_sample_index_ * 3 + 1] & 0x03) * 64 +
        (scan_packages_.package.packageSampleDistance[package_sample_index_ * 3] >> 2);
      node->exp_m = scan_packages_.package.packageSampleDistance[package_sample_index_ * 3] & 0x01;
    } else {
      node->distance_q2 = scan_packages_.packages.packageSampleDistance[package_sample_index_] >> 2;
      node->sync_quality =
        static_cast<uint16_t>(
        scan_packages_.packages.packageSampleDistance[package_sample_index_] & 0x03);
    }

    if (node->distance_q2 != 0) {
      angle_correct_for_distance =
        static_cast<int32_t>(
        atan(19.16 * (node->distance_q2 - 90.15) / (90.15 * node->distance_q2)) * 64);
    } else {
      angle_correct_for_distance = 0;
    }

    float sample_angle = interval_sample_angle_ * package_sample_index_;

    if ((first_sample_angle_ + sample_angle + angle_correct_for_distance) < 0) {
      node->angle_q6_checkbit = (((uint16_t)(first_sample_angle_ + sample_angle +
        angle_correct_for_distance + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
        LIDAR_RESP_MEASUREMENT_CHECKBIT;
    } else {
      if ((first_sample_angle_ + sample_angle + angle_correct_for_distance) > 23040) {
        node->angle_q6_checkbit = (((uint16_t)(first_sample_angle_ + sample_angle +
          angle_correct_for_distance - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
          LIDAR_RESP_MEASUREMENT_CHECKBIT;
      } else {
        node->angle_q6_checkbit = (((uint16_t)(first_sample_angle_ + sample_angle +
          angle_correct_for_distance)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
          LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
    }
  } else {
    node->sync_flag = Node_NotSync;
    node->sync_quality = Node_Default_Quality;
    node->angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    node->distance_q2 = 0;
    node->scan_frequency = 0;
  }

  uint8_t now_package_num;
  if (lidar_general_info_.intensity_data_flag) {
    now_package_num = scan_packages_.package.nowPackageNum;
  } else {
    now_package_num = scan_packages_.packages.nowPackageNum;
  }

  package_sample_index_++;

  if (package_sample_index_ >= now_package_num) {
    package_sample_index_ = 0;
    check_sum_result_ = false;
  }
  return RESULT_OK;
}

result_t LidarDataProcessor::wait_scan_data(
  node_info * nodebuffer, size_t & count,
  uint32_t timeout)
{
  if (!lidar_status_->serial_connected) {
    count = 0;
    return RESULT_FAIL;
  }

  recv_node_count_ = 0;
  uint32_t start_ts = get_milliseconds();
  uint32_t wait_time = 0;
  result_t ans = RESULT_FAIL;
  /*超时处理及点数判断*/
  while ((wait_time = get_milliseconds() - start_ts) <= timeout && recv_node_count_ < count) {
    node_info node;
    ans = wait_package(&node, timeout - wait_time);

    if (!IS_OK(ans)) {
      count = recv_node_count_;
      return ans;
    }

    nodebuffer[recv_node_count_++] = node;

    if (node.sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
      size_t size = serial_port_->available();
      uint64_t delay_time = 0;
      size_t package_size =
        lidar_general_info_.intensity_data_flag ?
        INTENSITY_NORMAL_PACKAGE_SIZE :
        NORMAL_PACKAGE_SIZE;

      if (size > PackagePaidBytes && size < PackagePaidBytes * package_size) {
        size_t package_num = size / package_size;
        size_t number = size % package_size;
        delay_time = package_num * lidar_general_info_.scan_time_increment * package_size / 2;

        if (number > PackagePaidBytes) {
          delay_time += lidar_general_info_.scan_time_increment * ((number - PackagePaidBytes) / 2);
        }

        size = number;

        if (package_num > 0 && number == 0) {
          size = package_size;
        }
      }
      nodebuffer[recv_node_count_ - 1].stamp = size * trans_delay_ + delay_time;
      nodebuffer[recv_node_count_ - 1].scan_frequency = node.scan_frequency;
      count = recv_node_count_;
      return RESULT_OK;
    }
    if (recv_node_count_ == count) {
      return RESULT_OK;
    }
  }
  count = recv_node_count_;
  return RESULT_FAIL;
}
