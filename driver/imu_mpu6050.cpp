#include "imu_mpu6050.h"

#include <cmath>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace quad {
namespace {
constexpr uint8_t kWhoAmI = 0x75;
constexpr uint8_t kPowerManagement = 0x6B;
constexpr uint8_t kInterruptStatus = 0x3A;
constexpr int kI2cTimeoutMs = 20;

void delay_ms(unsigned ms) {
    vTaskDelay(pdMS_TO_TICKS(ms) > 0 ? pdMS_TO_TICKS(ms) : 1);
}

int16_t signed_word(const uint8_t* data) {
    const int value = (static_cast<int>(data[0]) << 8) | data[1];
    return static_cast<int16_t>(value >= 32768 ? value - 65536 : value);
}

struct Statistics {
    double mean = 0;
    double m2 = 0;
    void add(float value, size_t n) {
        const double delta = value - mean;
        mean += delta / static_cast<double>(n);
        m2 += delta * (value - mean);
    }
    double variance(size_t n) const { return m2 / static_cast<double>(n - 1); }
};
}  // namespace

Mpu6050::~Mpu6050() { (void)deinit(); }

esp_err_t Mpu6050::deinit() {
    initialized_ = false;
    esp_err_t error = device_.deinit();
    if (error != ESP_OK) return error;
    error = bus_.deinit();
    if (error != ESP_OK) return error;
    bias_ = {};
    return ESP_OK;
}

esp_err_t Mpu6050::write_register(uint8_t reg, uint8_t value) {
    const uint8_t bytes[] = {reg, value};
    return device_.write(bytes, sizeof(bytes));
}

esp_err_t Mpu6050::read_registers(uint8_t reg, uint8_t* data, size_t size) {
    return device_.write_read(&reg, 1, data, size);
}

esp_err_t Mpu6050::init(const ImuConfig& config) {
    if (bus_.initialized() || device_.initialized()) return ESP_ERR_INVALID_STATE;
    if (config.address != 0x68 && config.address != 0x69) return ESP_ERR_INVALID_ARG;
    esp_err_t err = bus_.init({config.sda, config.scl});
    if (err != ESP_OK) return err;

    // Clean up every partial initialization, preserving the original error.
    const auto fail = [this](esp_err_t error) {
        (void)deinit();
        return error;
    };
    err = bus_.probe(config.address, kI2cTimeoutMs);
    if (err != ESP_OK) return fail(err);
    err = device_.init(bus_, config.address, 400000, kI2cTimeoutMs);
    if (err != ESP_OK) return fail(err);

    uint8_t identity = 0;
    err = read_registers(kWhoAmI, &identity, 1);
    if (err != ESP_OK) return fail(err);
    // WHO_AM_I is 0x68 even when AD0 selects address 0x69.
    if (identity != 0x68) return fail(ESP_ERR_NOT_SUPPORTED);
    err = write_register(kPowerManagement, 0x80);  // Device reset.
    if (err != ESP_OK) return fail(err);
    delay_ms(100);

    const uint8_t settings[][2] = {
        {0x6B, 0x01},  // Wake, PLL with X gyro clock.
        {0x6C, 0x00},  // Enable all axes.
        {0x1A, 0x03},  // DLPF: accel 44 Hz, gyro 42 Hz; internal rate 1 kHz.
        {0x19, 0x04},  // 1000 / (1 + 4) = 200 Hz.
        {0x1B, 0x00},  // Gyro +/-250 degrees/s, 131 LSB/(degrees/s).
        {0x1C, 0x00},  // Accel +/-2 g, 16384 LSB/g.
        {0x23, 0x00},  // FIFO disabled; read latest coherent register sample.
        {0x6A, 0x00},  // Disable FIFO and auxiliary I2C master.
        {0x37, 0x00},  // INT_STATUS read clears data-ready; no GPIO IRQ needed.
        {0x38, 0x01},  // Enable data-ready status.
    };
    for (const auto& setting : settings) {
        err = write_register(setting[0], setting[1]);
        if (err != ESP_OK) return fail(err);
        uint8_t actual = 0;
        err = read_registers(setting[0], &actual, 1);
        if (err != ESP_OK) return fail(err);
        if (actual != setting[1]) return fail(ESP_ERR_INVALID_RESPONSE);
    }
    delay_ms(100);  // Allow gyro/DLPF to settle.
    initialized_ = true;
    return ESP_OK;
}

esp_err_t Mpu6050::read_uncalibrated(ImuSample& sample) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    const int64_t deadline = esp_timer_get_time() + 50000;
    uint8_t data[15]{};
    do {
        // INT_STATUS followed by accel XYZ, temperature and gyro XYZ.
        const esp_err_t err = read_registers(kInterruptStatus, data, sizeof(data));
        if (err != ESP_OK) return err;
        if (data[0] & 0x01) {
            ImuSample result{};
            result.accel_g = {signed_word(data + 1) / 16384.0f,
                              signed_word(data + 3) / 16384.0f,
                              signed_word(data + 5) / 16384.0f};
            result.temperature_c = signed_word(data + 7) / 340.0f + 36.53f;
            result.gyro_dps = {signed_word(data + 9) / 131.0f,
                               signed_word(data + 11) / 131.0f,
                               signed_word(data + 13) / 131.0f};
            result.timestamp_us = esp_timer_get_time();
            sample = result;
            return ESP_OK;
        }
        delay_ms(1);
    } while (esp_timer_get_time() < deadline);
    return ESP_ERR_TIMEOUT;
}

esp_err_t Mpu6050::read(ImuSample& sample) {
    ImuSample result{};
    const esp_err_t err = read_uncalibrated(result);
    if (err != ESP_OK) return err;
    result.accel_g.x -= bias_.accel_g.x;
    result.accel_g.y -= bias_.accel_g.y;
    result.accel_g.z -= bias_.accel_g.z;
    result.gyro_dps.x -= bias_.gyro_dps.x;
    result.gyro_dps.y -= bias_.gyro_dps.y;
    result.gyro_dps.z -= bias_.gyro_dps.z;
    sample = result;
    return ESP_OK;
}

esp_err_t Mpu6050::calibrate(size_t sample_count) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (sample_count < 100 || sample_count > 4000) return ESP_ERR_INVALID_ARG;
    Statistics stats[6];
    // Discard any pending sample before collecting fresh calibration data.
    ImuSample sample{};
    esp_err_t err = read_uncalibrated(sample);
    if (err != ESP_OK) return err;
    for (size_t n = 1; n <= sample_count; ++n) {
        err = read_uncalibrated(sample);
        if (err != ESP_OK) return err;
        const float values[] = {sample.accel_g.x, sample.accel_g.y, sample.accel_g.z,
                                sample.gyro_dps.x, sample.gyro_dps.y, sample.gyro_dps.z};
        for (size_t axis = 0; axis < 6; ++axis) stats[axis].add(values[axis], n);
    }
    // Reject wrong pose, strong rotation and vibration rather than learning them as bias.
    if (std::fabs(stats[0].mean) > 0.15 || std::fabs(stats[1].mean) > 0.15 ||
        std::fabs(stats[2].mean - 1.0) > 0.15) return ESP_ERR_INVALID_STATE;
    for (size_t axis = 0; axis < 6; ++axis) {
        const double max_variance = axis < 3 ? 0.0004 : 0.25;
        if (stats[axis].variance(sample_count) > max_variance ||
            (axis >= 3 && std::fabs(stats[axis].mean) > 5.0)) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    bias_.accel_g = {static_cast<float>(stats[0].mean), static_cast<float>(stats[1].mean),
                     static_cast<float>(stats[2].mean - 1.0)};
    bias_.gyro_dps = {static_cast<float>(stats[3].mean), static_cast<float>(stats[4].mean),
                      static_cast<float>(stats[5].mean)};
    return ESP_OK;
}

}  // namespace quad
