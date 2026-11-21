#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "gpio.h"

namespace quad {

struct I2cBusConfig {
    gpio_num_t sda = GPIO_NUM_6;
    gpio_num_t scl = GPIO_NUM_7;
};

// Owns I2C port 0. Multiple I2cDevice objects may borrow this bus.
// The bus must outlive all its devices; configure/tear down from one task.
class I2cBus {
public:
    I2cBus() = default;
    ~I2cBus();
    I2cBus(const I2cBus&) = delete;
    I2cBus& operator=(const I2cBus&) = delete;

    esp_err_t init(const I2cBusConfig& config = {});
    esp_err_t deinit();
    esp_err_t probe(uint8_t address, int timeout_ms = 20);
    bool initialized() const { return handle_ != nullptr; }

private:
    friend class I2cDevice;
    i2c_master_bus_handle_t handle_ = nullptr;
};

class I2cDevice {
public:
    I2cDevice() = default;
    ~I2cDevice();
    I2cDevice(const I2cDevice&) = delete;
    I2cDevice& operator=(const I2cDevice&) = delete;

    esp_err_t init(I2cBus& bus, uint8_t address, uint32_t speed_hz = 400000,
                   int timeout_ms = 20);
    esp_err_t deinit();
    esp_err_t write(const uint8_t* data, size_t size);
    // Repeated-start transaction, with no STOP between write and read phases.
    esp_err_t write_read(const uint8_t* write_data, size_t write_size,
                         uint8_t* read_data, size_t read_size);
    bool initialized() const { return handle_ != nullptr; }

private:
    i2c_master_dev_handle_t handle_ = nullptr;
    int timeout_ms_ = 20;
};

}  // namespace quad
