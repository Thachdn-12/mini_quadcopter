#include "motor_pwm.h"

#include "driver/ledc.h"

namespace quad {

MotorPwm::~MotorPwm() { (void)deinit(); }

esp_err_t MotorPwm::init(const MotorPwmConfig& config) {
    if (timer_configured_) return ESP_ERR_INVALID_STATE;
    for (size_t i = 0; i < config.pins.size(); ++i) {
        if (!Gpio::valid_output(config.pins[i])) return ESP_ERR_INVALID_ARG;
        for (size_t j = 0; j < i; ++j) {
            if (config.pins[i] == config.pins[j]) return ESP_ERR_INVALID_ARG;
        }
    }
    ledc_timer_config_t timer{};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = 20000;
    timer.clk_cfg = LEDC_AUTO_CLK;
    esp_err_t error = ledc_timer_config(&timer);
    if (error != ESP_OK) return error;
    timer_configured_ = true;

    for (size_t i = 0; i < config.pins.size(); ++i) {
        error = Gpio::configure_output(config.pins[i], false);
        if (error != ESP_OK) {
            (void)deinit();
            return error;
        }
        ledc_channel_config_t channel{};
        channel.gpio_num = config.pins[i];
        channel.speed_mode = LEDC_LOW_SPEED_MODE;
        channel.channel = static_cast<ledc_channel_t>(i);
        channel.intr_type = LEDC_INTR_DISABLE;
        channel.timer_sel = LEDC_TIMER_0;
        channel.duty = 0;
        channel.hpoint = 0;
        error = ledc_channel_config(&channel);
        if (error != ESP_OK) {
            (void)deinit();
            return error;
        }
        ++configured_channels_;
    }
    initialized_ = true;
    return ESP_OK;
}

esp_err_t MotorPwm::write(const MotorDuty& duty) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    for (const auto value : duty) {
        if (value > kMotorMaxDuty) {
            (void)stop();
            return ESP_ERR_INVALID_ARG;
        }
    }
    // Channel updates are sequential, not simultaneous across all four motors.
    for (size_t i = 0; i < duty.size(); ++i) {
        const esp_err_t error = ledc_set_duty_and_update(
            LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(i), duty[i], 0);
        if (error != ESP_OK) {
            (void)stop();
            return error;
        }
    }
    return ESP_OK;
}

esp_err_t MotorPwm::stop() {
    esp_err_t result = ESP_OK;
    for (size_t i = 0; i < configured_channels_; ++i) {
        const esp_err_t error = ledc_stop(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(i), 0);
        if (result == ESP_OK && error != ESP_OK) result = error;
    }
    return result;
}

esp_err_t MotorPwm::deinit() {
    const esp_err_t error = stop();
    if (error != ESP_OK) return error;
    if (timer_configured_) {
        const esp_err_t timer_error = ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
        if (timer_error != ESP_OK) return timer_error;
    }
    configured_channels_ = 0;
    timer_configured_ = initialized_ = false;
    return ESP_OK;
}

}  // namespace quad
