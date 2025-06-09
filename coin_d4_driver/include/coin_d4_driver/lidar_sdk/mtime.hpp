// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#ifndef COIN_D4_DRIVER__LIDAR_SDK__MTIME_HPP_
#define COIN_D4_DRIVER__LIDAR_SDK__MTIME_HPP_

#include <time.h>
#include <unistd.h>

#include <chrono>


enum TIME_PRECISION
{
  TIME_NANOSECOND = 0,
  TIME_MICROSECOND,
  TIME_MILLISECOND,
  TIME_SECOND,
  TIME_MINUTE,
  TIME_HOUR
};

void sleep_ms(int ms);

int64_t current_times(int precision = TIME_MILLISECOND);

#endif  // COIN_D4_DRIVER__LIDAR_SDK__MTIME_HPP_
