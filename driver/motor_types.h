#pragma once

#include <array>
#include <cstdint>

namespace quad {

constexpr uint16_t kMotorMaxDuty = 1023;
// M1 front-left, M2 front-right, M3 rear-left, M4 rear-right; 0..1023.
using MotorDuty = std::array<uint16_t, 4>;

}  // namespace quad
