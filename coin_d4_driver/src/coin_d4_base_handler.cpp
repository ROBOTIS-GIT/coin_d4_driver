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

#include <chrono>
#include <memory>
#include <string>

#include "coin_d4_driver/coin_d4_base_handler.hpp"


namespace robotis
{
namespace coin_d4
{
CoinD4BaseHandler::CoinD4BaseHandler(
  const std::string parameter_prefix,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_interface,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params_interface)
: parameter_prefix_(parameter_prefix), logging_interface_(logging_interface),
  params_interface_(params_interface)
{
  this->init_structs();
}

CoinD4BaseHandler::~CoinD4BaseHandler()
{
  skip_grab_ = true;
  if (grab_thread_.joinable()) {
    grab_thread_.join();
  }
  skip_publish_ = true;
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  serial_port_->close();
  lidar_status_->lidar_ready = false;
  lidar_status_->close_lidar = true;
  flush_serial();
  RCLCPP_INFO(
    logging_interface_->get_logger(),
    "Closed lidar for port %s",
    lidar_general_info_.port.c_str());
}

void CoinD4BaseHandler::init_structs()
{
  lidar_status_ = std::make_shared<LidarHardwareStatus>();
  lidar_time_ = std::make_shared<LidarTimeStatus>();
  lidar_general_info_.port = declare_parameter_once(parameter_prefix_ + "port", "/dev/ttyUSB0");
  lidar_general_info_.frame_id =
    declare_parameter_once(parameter_prefix_ + "frame_id", "base_scan");
  lidar_general_info_.serial_baud_rate =
    declare_parameter_once(parameter_prefix_ + "baudrate", 230400);
  lidar_general_info_.version = declare_parameter_once(parameter_prefix_ + "version", 4);
  lidar_general_info_.topic_name =
    declare_parameter_once(parameter_prefix_ + "topic_name", "scan");
  lidar_general_info_.reverse =
    declare_parameter_once(parameter_prefix_ + "reverse", false);
  lidar_general_info_.warmup_time =
    declare_parameter_once(parameter_prefix_ + "warmup_time", 0);
  switch (lidar_general_info_.version) {
    case M1C1_Mini_v1:
      RCLCPP_INFO(logging_interface_->get_logger(), "version M1C1_Mini_v1");
      lidar_general_info_.serial_baud_rate = 115200;
      break;

    case M1C1_Mini_v2:
      RCLCPP_INFO(logging_interface_->get_logger(), "version M1C1_Mini_v2");
      lidar_general_info_.serial_baud_rate = 150000;
      lidar_general_info_.intensity_data_flag = true;
      break;

    case M1CT_Coin_Plus:
      RCLCPP_INFO(logging_interface_->get_logger(), "version M1CT_Coin_Plus");
      lidar_general_info_.serial_baud_rate = 115200;
      break;

    case M1CT_TOF:
      RCLCPP_INFO(logging_interface_->get_logger(), "version M1CT_TOF");
      lidar_general_info_.serial_baud_rate = 230400;
      lidar_general_info_.intensity_data_flag = true;
      break;

    default:
      break;
  }

  lidar_data_processor_ =
    std::make_shared<LidarDataProcessor>(
    lidar_time_.get(),
    lidar_status_.get(),
    lidar_general_info_,
    scan_packages_);

  if (lidar_general_info_.version == M1C1_Mini_v1) {
    lidar_data_processor_->package_sample_bytes_ = 2;
  } else {
    lidar_data_processor_->package_sample_bytes_ = 3;
  }

  if (!init_lidar_port()) {
    RCLCPP_WARN(logging_interface_->get_logger(), "Lidar port is wrong");
    return;
  }
  lidar_data_processor_->set_serial_port(serial_port_.get());
  scan_node_buf_ = new node_info[1000];
}

bool CoinD4BaseHandler::init_lidar_port()
{
  if (lidar_status_->serial_connected) {
    return true;
  }
  if (parameter_prefix_.empty()) {
    RCLCPP_INFO(
      logging_interface_->get_logger(),
      "Lidar port: %s",
      lidar_general_info_.port.c_str());
  } else {
    RCLCPP_INFO(
      logging_interface_->get_logger(),
      "%s lidar port: %s",
      parameter_prefix_.c_str(),
      lidar_general_info_.port.c_str());
  }

  serial_port_ =
    std::make_shared<SerialPort>(
    lidar_general_info_.port,
    lidar_general_info_.serial_baud_rate,
    Timeout::simpleTimeout(DEFAULT_TIMEOUT));

  if (!serial_port_->open()) {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Failed to open lidar port");
    return false;
  }

  lidar_status_->serial_connected = true;
  sleep_ms(100);
  serial_port_->setDTR(0);
  return true;
}

void CoinD4BaseHandler::flush_serial()
{
  if (!lidar_status_->serial_connected) {
    return;
  }

  size_t len = serial_port_->available();

  if (len) {
    uint8_t * buffer = static_cast<uint8_t *>(alloca(len * sizeof(uint8_t)));
    serial_port_->read_data(buffer, len);
  }

  sleep_ms(20);
}

void CoinD4BaseHandler::activate_grab_thread()
{
  if (!lidar_status_->serial_connected) {
    return;
  }

  skip_grab_ = false;
  grab_thread_ = std::thread(
    [this]() {
      node_info local_buf[128];
      size_t count = 128;
      node_info local_scan[1000];
      size_t scan_count = 0;
      result_t ans = RESULT_FAIL;

      bool wait_speed_right = false;  // Check if LiDAR is normal rotation speed
      uint64_t lidar_status_time = 0;  // Time for start operation or restart operation

      memset(local_scan, 0, sizeof(local_scan));

      while (!skip_grab_) {
        bool state_judge = judge_lidar_state(wait_speed_right, lidar_status_time);
        if (state_judge) {
          count = 128;
          ans = lidar_data_processor_->wait_scan_data(local_buf, count);
          if (!IS_OK(ans)) {
            if (current_times() - lidar_time_->system_start_time > 3000) {
              if (!lidar_status_->lidar_restart_try) {
                RCLCPP_WARN(logging_interface_->get_logger(), "Tried to restart lidar");
                lidar_status_->lidar_restart_try = true;
                lidar_status_->lidar_trap_restart = true;
              } else {
                RCLCPP_WARN(logging_interface_->get_logger(), "Detected lidar stuck");
                lidar_status_->lidar_abnormal_state |= 0x01;
                usleep(100);
              }
            }
          } else {
            lidar_status_->lidar_restart_try = false;
            lidar_time_->system_start_time = current_times();
          }

          for (size_t pos = 0; pos < count; ++pos) {
            if (local_buf[pos].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
              if ((local_scan[0].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                local_scan[0].stamp = local_buf[pos].stamp;
                local_scan[0].scan_frequency = local_buf[pos].scan_frequency;

                // If frequency is abnormal for over 30 seconds, trigger abnormal state
                if (local_scan[0].scan_frequency > 200 || local_scan[0].scan_frequency < 10) {
                  if (current_times() - lidar_time_->lidar_frequency_abnormal_time > 30000) {
                    lidar_status_->lidar_abnormal_state |= 0x02;
                  }
                } else {
                  lidar_time_->lidar_frequency_abnormal_time = current_times();
                }

                lock_.lock();
                lidar_time_->scan_start_time = lidar_time_->tim_scan_start;
                lidar_time_->scan_end_time = lidar_time_->tim_scan_end;
                if (lidar_time_->tim_scan_start != lidar_time_->tim_scan_end) {
                  lidar_time_->tim_scan_start = lidar_time_->tim_scan_end;
                }

                memcpy(scan_node_buf_, local_scan, scan_count * sizeof(node_info));
                scan_node_count_ = scan_count;
                data_event_.set();
                lock_.unlock();
              }
              scan_count = 0;
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == _countof(local_scan)) {
              scan_count -= 1;
            }
          }
        } else {
          flush_serial();
          delay(100);
        }
      }
      return;
    });

  lidar_status_->lidar_ready = true;
  RCLCPP_INFO(
    logging_interface_->get_logger(),
    "Activated lidar grab thread for port %s",
    lidar_general_info_.port.c_str());
}

void CoinD4BaseHandler::deactivate_grab_thread()
{
  if (!lidar_status_->serial_connected) {
    return;
  }
  skip_grab_ = true;
  if (grab_thread_.joinable()) {
    grab_thread_.join();
  }

  lidar_status_->close_lidar = true;
  lidar_status_->lidar_ready = false;
  lidar_status_->lidar_last_status = lidar_status_->lidar_ready;
  lidar_status_->lidar_trap_restart = false;
  serial_port_->write_data(end_lidar);
  RCLCPP_INFO(
    logging_interface_->get_logger(),
    "Deactivated lidar grab thread for port %s",
    lidar_general_info_.port.c_str());
}

void CoinD4BaseHandler::activate_publish_thread()
{
  if (!lidar_status_->serial_connected) {
    return;
  }
  skip_publish_ = false;
  activate_scan_publisher();
  publish_thread_ = std::thread(
    [this]() {
      std::this_thread::sleep_for(std::chrono::seconds(lidar_general_info_.warmup_time));
      while (!skip_publish_) {
        LaserScan scan;
        if (grab_synchronized_data(scan)) {
          auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();

          scan_msg->ranges.resize(scan.points.size());
          scan_msg->intensities.resize(scan.points.size());
          scan_msg->header.stamp = get_node_time();
          scan_msg->header.frame_id = lidar_general_info_.frame_id;

          scan_msg->angle_min = scan.config.min_angle;
          scan_msg->angle_max = scan.config.max_angle;
          scan_msg->angle_increment = scan.config.angle_increment;
          scan_msg->scan_time = scan.config.scan_time;
          scan_msg->time_increment = scan.config.time_increment;
          scan_msg->range_min = scan.config.min_range;
          scan_msg->range_max = scan.config.max_range;
          for (uint16_t i = 0; i < scan.points.size(); i++) {
            if (lidar_general_info_.reverse) {
              scan_msg->ranges[i] = scan.points[scan.points.size() - 1 - i].range;
              scan_msg->intensities[i] = scan.points[scan.points.size() - 1 - i].intensity;
            } else {
              scan_msg->ranges[i] = scan.points[i].range;
              scan_msg->intensities[i] = scan.points[i].intensity;
            }
          }

          publish_scan(std::move(scan_msg));
        }
      }
      return;
    });

  RCLCPP_INFO(
    logging_interface_->get_logger(),
    "Activated lidar publish thread for port %s",
    lidar_general_info_.port.c_str());
}

void CoinD4BaseHandler::deactivate_publish_thread()
{
  skip_publish_ = true;
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  deactivate_scan_publisher();
  RCLCPP_INFO(
    logging_interface_->get_logger(),
    "Deactivated lidar publish thread for port %s",
    lidar_general_info_.port.c_str());
}

bool CoinD4BaseHandler::judge_lidar_state(bool & wait_speed_right, uint64_t & lidar_status_time)
{
  if (
    lidar_status_->lidar_ready != lidar_status_->lidar_last_status ||
    lidar_status_->close_lidar)
  {
    RCLCPP_INFO(
      logging_interface_->get_logger(),
      "Lidar status changed for %s : %d -> %d",
      lidar_general_info_.port.c_str(),
      lidar_status_->lidar_last_status,
      lidar_status_->lidar_ready);

    lidar_status_->close_lidar = false;
    lidar_status_->lidar_last_status = lidar_status_->lidar_ready;

    wait_speed_right = false;

    lidar_status_time = current_times();
    flush_serial();
  }
  if (lidar_status_->lidar_trap_restart) {
    RCLCPP_WARN(
      logging_interface_->get_logger(),
      "Abnormal lidar status for %s",
      lidar_general_info_.port.c_str());

    lidar_status_->lidar_trap_restart = false;

    wait_speed_right = false;

    lidar_status_time = current_times();
    serial_port_->write_data(end_lidar);
  }

  if (lidar_status_->lidar_ready && !wait_speed_right) {
    if (current_times() - lidar_status_time > 1000) {
      switch (lidar_general_info_.version) {
        case M1C1_Mini_v1:
          RCLCPP_INFO(
            logging_interface_->get_logger(),
            "V1 version lidar start for %s",
            lidar_general_info_.port.c_str());
          serial_port_->write_data(start_lidar);
          wait_speed_right = true;
          break;

        case M1C1_Mini_v2:
          RCLCPP_INFO(
            logging_interface_->get_logger(),
            "V2 X2 version lidar start for %s",
            lidar_general_info_.port.c_str());
          serial_port_->write_data(start_lidar);
          wait_speed_right = true;
          break;

        case M1CT_TOF:
          RCLCPP_INFO(
            logging_interface_->get_logger(),
            "TOF version lidar start for %s",
            lidar_general_info_.port.c_str());
          serial_port_->write_data(start_lidar);
          wait_speed_right = true;
          break;

        default:
          break;
      }
    }
    lidar_time_->lidar_frequency_abnormal_time = current_times();
    lidar_time_->system_start_time = current_times();
  }

  return wait_speed_right;
}

bool CoinD4BaseHandler::grab_synchronized_data(LaserScan & outscan)
{
  if (check_data_synchronization(2000) == RESULT_OK) {
    parse_lidar_serial_data(outscan);
    return true;
  } else {
    return false;
  }
}

result_t CoinD4BaseHandler::check_data_synchronization(uint32_t timeout)
{
  switch (data_event_.wait(timeout)) {
    case Event::EVENT_TIMEOUT:
      return RESULT_TIMEOUT;

    case Event::EVENT_OK:
      {
        lock_.lock();
        if (scan_node_count_ == 0) {
          return RESULT_FAIL;
        }
        lock_.unlock();
      }
      return RESULT_OK;

    default:
      return RESULT_FAIL;
  }
}

void CoinD4BaseHandler::parse_lidar_serial_data(LaserScan & outscan)
{
  lock_.lock();

  size_t count = scan_node_count_;

  if (count < MAX_SCAN_NODES && count > 0) {
    uint64_t scan_time = (lidar_time_->scan_end_time - lidar_time_->scan_start_time);

    outscan.config.angle_increment = 2.0 * M_PI / count;
    outscan.config.min_angle = 0;
    outscan.config.max_angle = 2 * M_PI;
    outscan.config.min_range = 0.10;
    outscan.config.max_range = 100.0;
    outscan.config.scan_time = static_cast<float>(scan_time * 1.0 / 1e9);
    outscan.config.time_increment = outscan.config.scan_time / static_cast<double>(count - 1);
    outscan.stamp = lidar_time_->scan_start_time;

    if (lidar_status_->serial_connected) {
      float range = std::numeric_limits<float>::quiet_NaN();
      float angle = std::numeric_limits<float>::quiet_NaN();
      uint16_t intensity = 0;
      for (int i = count - 1; i > 0; i--) {
        LaserPoint point;
        LaserPoint point_check;
        angle =
          static_cast<float>(
          (scan_node_buf_[count - i].angle_q6_checkbit >>
          LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);
        range = (scan_node_buf_[i].distance_q2 / 1000.0f);
        intensity = scan_node_buf_[i].sync_quality;

        if (scan_node_buf_[i].exp_m == 1) {
          intensity = 255;
        } else {
          if (intensity >= 255) {
            intensity = 254;
          }
        }

        point_check.angle = angle;
        point_check.range = range;
        point_check.intensity = intensity;

        if (0 <= angle && angle <= 360) {
          point.range = range;
          point.angle = angle;
          point.intensity = intensity;
        } else {
          point.range = std::numeric_limits<float>::quiet_NaN();
          point.intensity = 0;
          point.angle = std::numeric_limits<float>::quiet_NaN();
        }

        if (range <= 0.15 && intensity <= 65) {
          point.range = std::numeric_limits<float>::quiet_NaN();
          point.intensity = 0;
        }
        outscan.points.push_back(point);
      }
    }
  }
  lock_.unlock();
}
}  // namespace coin_d4
}  // namespace robotis
