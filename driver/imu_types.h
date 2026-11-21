#pragma once

#include <cstdint>

namespace quad {

struct Vector3 {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct ImuSample {
    Vector3 accel_g;
    Vector3 gyro_dps;
    float temperature_c = 0;
    int64_t timestamp_us = 0;
};

struct ImuBias {
    Vector3 accel_g;
    Vector3 gyro_dps;
};

}  // namespace quad
