// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#include "coin_d4_driver/lidar_sdk/mtime.h"

using namespace std;
using namespace chrono;

time_point<steady_clock> time_start;

int64_t current_times(int precision)
{
	auto durations = std::chrono::steady_clock::now().time_since_epoch();
	switch(precision)
	{
        case TIME_NANOSECOND:
            return durations.count();
        case TIME_MICROSECOND:
            return duration_cast<microseconds>(durations).count();
        case TIME_MILLISECOND:
            return duration_cast<milliseconds>(durations).count();
        case TIME_SECOND:
            return duration_cast<seconds>(durations).count();
        case TIME_MINUTE:
            return duration_cast<minutes>(durations).count();
        case TIME_HOUR:
            return duration_cast<hours>(durations).count();
	}
    return 0;
}

//休眠多少毫秒
void sleep_ms(int ms)
{
    int ts = ms / 1000;
    int ns = ms % 1000;
    struct timespec r {ts, ns * 1000 * 1000};
    nanosleep(&r, NULL);
}
