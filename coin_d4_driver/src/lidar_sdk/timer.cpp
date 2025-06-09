// Copyright 2025 ROBOTIS CO., LTD.
// Authors: Hyeongjun Jeon

#include <assert.h>
#include <inttypes.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>


#include "coin_d4_driver/lidar_sdk/timer.h"


uint32_t get_milliseconds()
{
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000L + t.tv_nsec / 1000000L;
}

int gettimeofday_t(struct timeval *tv, void * /*tzv*/)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    tv->tv_sec = ts.tv_sec;
    tv->tv_usec = ts.tv_nsec / 1000;
    return 0;
}

uint64_t getTime()
{
#if HAS_CLOCK_GETTIME
    struct timespec tim;
    clock_gettime(CLOCK_REALTIME, &tim);
    return static_cast<uint64_t>(tim.tv_sec) * 1000000000LL + tim.tv_nsec;
#else
    struct timeval timeofday;
    gettimeofday_t(&timeofday, NULL);
    return static_cast<uint64_t>(timeofday.tv_sec) * 1000000000LL +
           static_cast<uint64_t>(timeofday.tv_usec) * 1000LL;
#endif
}

void delay(uint32_t ms) {
  while (ms >= 1000) {
    usleep(1000 * 1000);
    ms -= 1000;
  }

  if (ms != 0) {
    usleep(ms * 1000);
  }
}
