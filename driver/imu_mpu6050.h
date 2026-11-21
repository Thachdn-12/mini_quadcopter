#pragma once

#include <cstddef>
#include <cstdint>

#include "i2c.h"
#include "esp_err.h"
#include "imu_types.h"

namespace quad {

struct ImuConfig {
    gpio_num_t sda = GPIO_NUM_6;
    gpio_num_t scl = GPIO_NUM_7;
    uint8_t address = 0x68;  // AD0 low; use 0x69 for AD0 high.
};

// Owns its I2C bus. Call from one task; do not copy or share concurrently.
class Mpu6050 {
public:
    static constexpr unsigned kSampleRateHz = 200;
    Mpu6050() = default;
    ~Mpu6050();
    Mpu6050(const Mpu6050&) = delete;
    Mpu6050& operator=(const Mpu6050&) = delete;

    esp_err_t init(const ImuConfig& config = {});
    esp_err_t deinit();
    // Waits for fresh data, bounded by 50 ms plus one I2C transaction.
    // Output is unchanged on error. Units: g, degrees/s, Celsius, microseconds.
    esp_err_t read(ImuSample& sample);
    // Keep sensor level, motionless, with +Z upward. Biases commit only on success.
    // Single-position calibration preserves +1 g on Z; no scale calibration.
    esp_err_t calibrate(size_t sample_count = 400);
    const ImuBias& bias() const { return bias_; }

private:
    esp_err_t write_register(uint8_t reg, uint8_t value);
    esp_err_t read_registers(uint8_t reg, uint8_t* data, size_t size);
    esp_err_t read_uncalibrated(ImuSample& sample);

    I2cBus bus_;
    I2cDevice device_;  // Destroyed before bus_.
    bool initialized_ = false;
    ImuBias bias_{};
};

}  // namespace quad
