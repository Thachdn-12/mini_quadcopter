#pragma once

#include <cstdint>
#include "esp_err.h"

#include <driver/gpio.h>

namespace quad {

enum class GpioPull { None, Up, Down };

// Stateless pin operations. The device driver owns the pin's lifetime/function.
class Gpio {
public:
    typedef int gpio_num_t;
    static bool valid_input(gpio_num_t pin);
    static bool valid_output(gpio_num_t pin);
    static esp_err_t configure_input(gpio_num_t pin, GpioPull pull = GpioPull::None);
    static esp_err_t configure_output(gpio_num_t pin, bool initial_level = false);
    static esp_err_t write(gpio_num_t pin, bool level);
    // Pin must have its input path enabled (for example configure_input()).
    static esp_err_t read(gpio_num_t pin, bool& level);
    static esp_err_t reset(gpio_num_t pin);
};

}  // namespace quad
