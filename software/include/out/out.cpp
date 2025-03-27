#include "out.h"
#include <hardware/gpio.h>
#include <hardware/clocks.h>

// Global module class instance
Out out;

char Out::getType(uint8_t i) {
    if(i >= OUT_SIZE) return 0;
    return type[i];
}

float Out::get(uint8_t i) {
    // Get last set value (might not be output because of armed == false)
    if(i >= OUT_SIZE) return 0;
    return command[i];
}

void Out::setup() {
    // Optional initialization method
    // Can be used for any global setup needed
}

bool Out::_setupOutput(char typ, uint8_t i, int pin, int freq_hz, int pwm_min_us, int pwm_max_us) {
    if(i >= OUT_SIZE) return false;

    // Set the type of output
    type[i] = typ;

    // Get slice number
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Calculate PWM parameters
    uint16_t max_duty = (1 << 16) - 1;
    float divider = clock_get_hz(clk_sys) / (max_duty + 1) / freq_hz;
    float inv_duty_resolution_us = 1.0e-6 * freq_hz * (max_duty + 1);

    // Configure GPIO for PWM
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Configure PWM slice
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, divider);
    pwm_config_set_wrap(&config, max_duty);
    pwm_init(slice_num, &config, true);

    // Store configuration
    outputs[i] = {
        pin,
        slice_num,
        freq_hz,
        pwm_min_us,
        pwm_max_us,
        inv_duty_resolution_us
    };

    // Initialize command to zero
    command[i] = 0;

    // Set initial PWM level to zero
    pwm_set_gpio_level(pin, 0);

    return true;
}

bool Out::setupMotor(uint8_t i, int pin, int freq_hz, int pwm_min_us, int pwm_max_us) {
    return _setupOutput('M', i, pin, freq_hz, pwm_min_us, pwm_max_us);
}

bool Out::setupServo(uint8_t i, int pin, int freq_hz, int pwm_min_us, int pwm_max_us) {
    return _setupOutput('S', i, pin, freq_hz, pwm_min_us, pwm_max_us);
}

void Out::set(uint8_t i, float value) {
    if(i >= OUT_SIZE) return;

    // Bounds check and clamp value
    if(value < 0) value = 0;
    if(value > 1) value = 1;

    // Store the commanded value
    command[i] = value;

    // Only write to PWM if armed
    if(armed) {
        // Calculate PWM level based on min/max microseconds
        OutputConfig& cfg = outputs[i];
        float us = cfg.min_us + value * (cfg.max_us - cfg.min_us);
        
        // Convert microseconds to PWM level
        uint16_t level = us * cfg.inv_duty_resolution_us;
        
        // Set PWM level
        pwm_set_gpio_level(cfg.pin, level);
    } else {
        // If not armed and it's a motor, force to zero
        if(type[i] == 'M') {
            pwm_set_gpio_level(outputs[i].pin, 0);
        }
    }
}