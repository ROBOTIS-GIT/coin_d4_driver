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

#ifndef COIN_D4_DRIVER__LIDAR_SDK__SERIAL_PORT_HPP_
#define COIN_D4_DRIVER__LIDAR_SDK__SERIAL_PORT_HPP_

#include <stdint.h>
#include <termios.h>

#include <atomic>
#include <limits>
#include <string>
#include <vector>

#include "coin_d4_driver/lidar_sdk/lidar_information.hpp"


class MillisecondTimer
{
public:
  explicit MillisecondTimer(const uint64_t millis);
  int64_t remaining();

private:
  static timespec timespec_now();
  timespec expiry;
};

struct Timeout
{
  static uint32_t max()
  {
    return std::numeric_limits<uint32_t>::max();
  }

  static Timeout simpleTimeout(uint32_t timeout_t)
  {
    return Timeout(max(), timeout_t, 0, timeout_t, 0);
  }

  uint32_t inter_byte_timeout;
  uint32_t read_timeout_constant;

  uint32_t read_timeout_multiplier;
  uint32_t write_timeout_constant;

  uint32_t write_timeout_multiplier;

  explicit Timeout(
    uint32_t inter_byte_timeout_ = 0,
    uint32_t read_timeout_constant_ = 0,
    uint32_t read_timeout_multiplier_ = 0,
    uint32_t write_timeout_constant_ = 0,
    uint32_t write_timeout_multiplier_ = 0)
  : inter_byte_timeout(inter_byte_timeout_),
    read_timeout_constant(read_timeout_constant_),
    read_timeout_multiplier(read_timeout_multiplier_),
    write_timeout_constant(write_timeout_constant_),
    write_timeout_multiplier(write_timeout_multiplier_)
  {
  }
};


typedef enum
{
  fivebits = 5,
  sixbits = 6,
  sevenbits = 7,
  eightbits = 8
} bytesize_t;

typedef enum
{
  stopbits_one = 1,
  stopbits_two = 2,
  stopbits_one_point_five
} stopbits_t;

typedef enum
{
  flowcontrol_none = 0,
  flowcontrol_software,
  flowcontrol_hardware
} flowcontrol_t;

typedef enum
{
  parity_none = 0,
  parity_odd = 1,
  parity_even = 2,
  parity_mark = 3,
  parity_space = 4
} parity_t;

class SerialPort
{
private:
  std::string port_;

  uint64_t baudrate_;  // Baudrate
  int fd_;
  pid_t pid;
  bool is_open_ = false;
  uint32_t byte_time_ns_;  // Nanoseconds to transmit/receive a single byte

  Timeout timeout_;  // Timeout for read operations

  bytesize_t bytesize_;  // Size of the bytes
  parity_t parity_;  // Parity
  stopbits_t stopbits_;  // Stop Bits
  flowcontrol_t flowcontrol_;  // Flow Control

public:
  SerialPort(
    const std::string & port = "",
    uint32_t baudrate = 115200,
    Timeout timeout = Timeout(),
    bytesize_t bytesize = eightbits,
    parity_t parity = parity_none,
    stopbits_t stopbits = stopbits_one,
    flowcontrol_t flowcontrol = flowcontrol_none);

  ~SerialPort();
  bool open();
  void close();
  bool getTermios(termios * tio);

  void set_databits(termios * tio, bytesize_t databits);

  void set_parity(termios * tio, parity_t parity);

  void set_stopbits(termios * tio, stopbits_t stopbits);

  void set_flowcontrol(termios * tio, flowcontrol_t flowcontrol);

  void set_common_props(termios * tio);


  result_t read_data(uint8_t * buf, size_t size);
  size_t write_data(const uint8_t * data, size_t length = 4);
  result_t waitForData(size_t data_count, uint64_t timeout, size_t * returned_size);
  size_t available();
  bool setDTR(bool level);

  uint32_t getByteTime();

  bool setBaudrate(uint64_t baudrate);
  bool setTermios(const termios * tio);
  bool setCustomBaudRate(uint64_t baudrate);
  bool waitReadable(uint32_t timeout_t);
};

#endif  // COIN_D4_DRIVER__LIDAR_SDK__SERIAL_PORT_HPP_
