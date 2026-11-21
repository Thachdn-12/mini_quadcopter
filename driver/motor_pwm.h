#pragma once

#include <cstddef>

#include "gpio.h"
#include "esp_err.h"
#include "motor_types.h"

namespace quad {

struct MotorPwmConfig {
    std::array<Gpio::gpio_num_t, 4> pins{3, 4, 5, 10};
};

// Owns low-speed LEDC timer 0 and channels 0..3. Use from task_motor only.
class MotorPwm {
public:
    MotorPwm() = default;
    ~MotorPwm();
    MotorPwm(const MotorPwm&) = delete;
    MotorPwm& operator=(const MotorPwm&) = delete;

    esp_err_t init(const MotorPwmConfig& config = {});
    esp_err_t write(const MotorDuty& duty);
    esp_err_t stop();
    esp_err_t deinit();

private:
    size_t configured_channels_ = 0;
    bool timer_configured_ = false;
    bool initialized_ = false;
};

}  // namespace quad
