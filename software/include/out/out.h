#pragma once

#include <cstdint>
#include <hardware/pwm.h>

#define OUT_SIZE 16  // Maximum number of outputs

class Out {
public:
    Out() : armed(false) {}  // Constructor initializes armed to false

    // Setup methods
    void setup();
    bool setupMotor(uint8_t i, int pin, int freq_hz, int pwm_min_us, int pwm_max_us);
    bool setupServo(uint8_t i, int pin, int freq_hz, int pwm_min_us, int pwm_max_us);

    // Getter methods
    char getType(uint8_t i);
    float get(uint8_t i);

    // Control methods
    void set(uint8_t i, float value);
    
    // Arming method to enable/disable output
    void setArmed(bool arm) { armed = arm; }
    bool isArmed() const { return armed; }

private:
    // Internal setup method
    bool _setupOutput(char typ, uint8_t i, int pin, int freq_hz, int pwm_min_us, int pwm_max_us);

    // Member variables
    struct OutputConfig {
        int pin;
        uint slice_num;
        int freq_hz;
        int min_us;
        int max_us;
        float inv_duty_resolution_us;
    };

    OutputConfig outputs[OUT_SIZE] = {0};
    char type[OUT_SIZE] = {0};      // Type of each output ('M' for motor, 'S' for servo)
    float command[OUT_SIZE] = {0};  // Last commanded value for each output
    bool armed = false;             // Global arming flag
};

// Global instance declaration
extern Out out;