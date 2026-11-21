#include "gpio.h"

namespace quad {

bool Gpio::valid_input(gpio_num_t pin) {
    return pin >= 0 && pin < 64 && GPIO_IS_VALID_GPIO(pin);
}

bool Gpio::valid_output(gpio_num_t pin) {
    return pin >= 0 && pin < 64 && GPIO_IS_VALID_OUTPUT_GPIO(pin);
}

esp_err_t Gpio::configure_input(gpio_num_t pin, GpioPull pull) {
    if (!valid_input(pin)) return ESP_ERR_INVALID_ARG;
    if (pull != GpioPull::None && pull != GpioPull::Up && pull != GpioPull::Down)
        return ESP_ERR_INVALID_ARG;
    gpio_config_t config{};
    config.pin_bit_mask = uint64_t{1} << pin;
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = pull == GpioPull::Up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    config.pull_down_en = pull == GpioPull::Down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&config);
}

esp_err_t Gpio::configure_output(gpio_num_t pin, bool initial_level) {
    if (!valid_output(pin)) return ESP_ERR_INVALID_ARG;
    // Set the latch before enabling output to avoid a contrary initial level.
    const esp_err_t error = gpio_set_level(static_cast<::gpio_num_t>(pin), initial_level ? 1 : 0);
    if (error != ESP_OK) return error;
    gpio_config_t config{};
    config.pin_bit_mask = uint64_t{1} << pin;
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&config);
}

esp_err_t Gpio::write(gpio_num_t pin, bool level) {
    if (!valid_output(pin)) return ESP_ERR_INVALID_ARG;
    return gpio_set_level(static_cast<::gpio_num_t>(pin), level ? 1 : 0);
}

esp_err_t Gpio::read(gpio_num_t pin, bool& level) {
    if (!valid_input(pin)) return ESP_ERR_INVALID_ARG;
    level = gpio_get_level(static_cast<::gpio_num_t>(pin)) != 0;
    return ESP_OK;
}

esp_err_t Gpio::reset(gpio_num_t pin) {
    if (!valid_input(pin)) return ESP_ERR_INVALID_ARG;
    return gpio_reset_pin(static_cast<::gpio_num_t>(pin));
}

}  // namespace quad
