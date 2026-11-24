#pragma once

#include <cstdint>
#include "esp_err.h"

#ifdef quad
#undef quad
#endif

namespace quad {

// Pilot setpoints consumed by task_pid, not normalized mixer corrections.
struct PilotCommand {
    float throttle = 0;       // 0..1
    float roll_deg = 0;       // -30..30
    float pitch_deg = 0;      // -30..30
    float yaw_rate_dps = 0;   // -180..180
    uint32_t sequence = 0;
    int64_t received_us = 0;
    int64_t expires_us = 0;   // Limited by the echoed server clock as well as receipt.
};

// Copies the latest command only while its connection and 100 ms lease are valid.
// Does not refresh its timestamp or forward anything to the motor/mixer.
esp_err_t get_network_pilot_command(PilotCommand& command);
void task_network_init(void* context);

}  // namespace quad
