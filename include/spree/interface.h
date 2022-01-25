#ifndef INTERACE_H
#define INTERACE_H

#include <stdint.h>

typedef uint32_t Timestamp;
typedef uint64_t Timestamp_Long;

enum SignalType
{
    SignalType_IMU,
    SignalType_SYNC,
    SignalType_SWEEP
};

struct Vec3
{
    double x, y, z;
};

struct IMU
{
    Timestamp time;
    Vec3 acc;
    Vec3 gyro;
};

struct SYNC
{
    Timestamp time;
    uint8_t channel;
    bool ootx;
    bool gen;
};

struct SWEEP
{
    Timestamp time;
    uint8_t channel;
    uint8_t sensor;
    bool half_clock_flag;
};

#endif