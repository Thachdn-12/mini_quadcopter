#include "i2c.h"

namespace quad {
namespace {
bool valid_address(uint8_t address) { return address >= 0x08 && address <= 0x77; }
}

I2cBus::~I2cBus() { (void)deinit(); }
I2cDevice::~I2cDevice() { (void)deinit(); }

esp_err_t I2cBus::init(const I2cBusConfig& config) {
    if (initialized()) return ESP_ERR_INVALID_STATE;
    if (!Gpio::valid_output(config.sda) || !Gpio::valid_output(config.scl) ||
        config.sda == config.scl) return ESP_ERR_INVALID_ARG;
    i2c_master_bus_config_t native{};
    native.i2c_port = I2C_NUM_0;
    native.sda_io_num = config.sda;
    native.scl_io_num = config.scl;
    native.clk_source = I2C_CLK_SRC_DEFAULT;
    native.glitch_ignore_cnt = 7;
    native.flags.enable_internal_pullup = true;
    return i2c_new_master_bus(&native, &handle_);
}

esp_err_t I2cBus::deinit() {
    if (!handle_) return ESP_OK;
    const esp_err_t error = i2c_del_master_bus(handle_);
    if (error == ESP_OK) handle_ = nullptr;
    return error;
}

esp_err_t I2cBus::probe(uint8_t address, int timeout_ms) {
    if (!initialized()) return ESP_ERR_INVALID_STATE;
    if (!valid_address(address) || timeout_ms <= 0) return ESP_ERR_INVALID_ARG;
    return i2c_master_probe(handle_, address, timeout_ms);
}

esp_err_t I2cDevice::init(I2cBus& bus, uint8_t address, uint32_t speed_hz, int timeout_ms) {
    if (initialized() || !bus.initialized()) return ESP_ERR_INVALID_STATE;
    if (!valid_address(address) || speed_hz == 0 || timeout_ms <= 0) return ESP_ERR_INVALID_ARG;
    i2c_device_config_t config{};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = address;
    config.scl_speed_hz = speed_hz;
    const esp_err_t error = i2c_master_bus_add_device(bus.handle_, &config, &handle_);
    if (error == ESP_OK) timeout_ms_ = timeout_ms;
    return error;
}

esp_err_t I2cDevice::deinit() {
    if (!handle_) return ESP_OK;
    const esp_err_t error = i2c_master_bus_rm_device(handle_);
    if (error == ESP_OK) handle_ = nullptr;
    return error;
}

esp_err_t I2cDevice::write(const uint8_t* data, size_t size) {
    if (!initialized()) return ESP_ERR_INVALID_STATE;
    if (!data || size == 0) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit(handle_, data, size, timeout_ms_);
}

esp_err_t I2cDevice::write_read(const uint8_t* write_data, size_t write_size,
                                 uint8_t* read_data, size_t read_size) {
    if (!initialized()) return ESP_ERR_INVALID_STATE;
    if (!write_data || write_size == 0 || !read_data || read_size == 0)
        return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit_receive(handle_, write_data, write_size, read_data, read_size,
                                     timeout_ms_);
}

}  // namespace quad
