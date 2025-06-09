// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#include "coin_d4_driver/lidar_sdk/mtime.hpp"


int64_t current_times(int precision)
{
  auto durations = std::chrono::steady_clock::now().time_since_epoch();
  switch(precision)
  {
    case TIME_NANOSECOND:
      return durations.count();
    case TIME_MICROSECOND:
      return std::chrono::duration_cast<std::chrono::microseconds>(durations).count();
    case TIME_MILLISECOND:
      return std::chrono::duration_cast<std::chrono::milliseconds>(durations).count();
    case TIME_SECOND:
      return std::chrono::duration_cast<std::chrono::seconds>(durations).count();
    case TIME_MINUTE:
      return std::chrono::duration_cast<std::chrono::minutes>(durations).count();
    case TIME_HOUR:
      return std::chrono::duration_cast<std::chrono::hours>(durations).count();
  }
  return 0;
}

void sleep_ms(int ms)
{
  int ts = ms / 1000;
  int ns = ms % 1000;
  struct timespec r {ts, ns * 1000 * 1000};
  nanosleep(&r, NULL);
}
