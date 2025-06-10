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
