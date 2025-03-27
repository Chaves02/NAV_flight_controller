#include "led.h"

// Global instance definition
DroneLedsController droneLeds;

DroneLedsController::DroneLedsController() : enabled(true) {}

void DroneLedsController::setup() {
    // Initialize all pins
    gpio_init(FRONT_LEFT_PIN);
    gpio_init(FRONT_RIGHT_PIN);
    gpio_init(BACK_LEFT_PIN);
    gpio_init(BACK_RIGHT_PIN);
    
    // Set all pins as outputs
    gpio_set_dir(FRONT_LEFT_PIN, GPIO_OUT);
    gpio_set_dir(FRONT_RIGHT_PIN, GPIO_OUT);
    gpio_set_dir(BACK_LEFT_PIN, GPIO_OUT);
    gpio_set_dir(BACK_RIGHT_PIN, GPIO_OUT);
    
    // Start with all LEDs off
    allOff();
}

// Control individual LEDs
void DroneLedsController::setFrontLeft(bool on) { 
    if (!enabled) return;
    gpio_put(FRONT_LEFT_PIN, on == ON_VALUE); 
}

void DroneLedsController::setFrontRight(bool on) { 
    if (!enabled) return;
    gpio_put(FRONT_RIGHT_PIN, on == ON_VALUE); 
}

void DroneLedsController::setBackLeft(bool on) { 
    if (!enabled) return;
    gpio_put(BACK_LEFT_PIN, on == ON_VALUE); 
}

void DroneLedsController::setBackRight(bool on) { 
    if (!enabled) return;
    gpio_put(BACK_RIGHT_PIN, on == ON_VALUE); 
}

// Control groups of LEDs
void DroneLedsController::setFront(bool on) {
    setFrontLeft(on);
    setFrontRight(on);
}

void DroneLedsController::setBack(bool on) {
    setBackLeft(on);
    setBackRight(on);
}

void DroneLedsController::setLeft(bool on) {
    setFrontLeft(on);
    setBackLeft(on);
}

void DroneLedsController::setRight(bool on) {
    setFrontRight(on);
    setBackRight(on);
}

void DroneLedsController::allOn() {
    setFrontLeft(true);
    setFrontRight(true);
    setBackLeft(true);
    setBackRight(true);
}

void DroneLedsController::allOff() {
    setFrontLeft(false);
    setFrontRight(false);
    setBackLeft(false);
    setBackRight(false);
}

void DroneLedsController::toggleAll() {
    if (!enabled) return;
    gpio_put(FRONT_LEFT_PIN, !gpio_get(FRONT_LEFT_PIN));
    gpio_put(FRONT_RIGHT_PIN, !gpio_get(FRONT_RIGHT_PIN));
    gpio_put(BACK_LEFT_PIN, !gpio_get(BACK_LEFT_PIN));
    gpio_put(BACK_RIGHT_PIN, !gpio_get(BACK_RIGHT_PIN));
}

//// Patterns useful for drone status indication
//void DroneLedsController::blinkAll(int time_on, int time_off) {
//        allOff();
//        sleep_ms(time_off);
//        allOn();
//        sleep_ms(time_on);
//}
//
//void DroneLedsController::rotatePattern(int times) {
//    for (int i = 0; i < times; i++) {
//        allOff();
//        setFrontRight(true);
//        sleep_ms(200);
//        allOff();
//        setBackRight(true);
//        sleep_ms(200);
//        allOff();
//        setBackLeft(true);
//        sleep_ms(200);
//        allOff();
//        setFrontLeft(true);
//        sleep_ms(200);
//    }
//    allOff();
//}

// Timer-based blink method
bool DroneLedsController::updateBlinkPattern(uint32_t time_on, uint32_t time_off) {
    uint32_t currentTime = to_ms_since_boot(get_absolute_time());
    
    // Check if timing has changed
    static uint32_t lastTimeOn = 0;
    static uint32_t lastTimeOff = 0;
    
    bool timingChanged = (time_on != lastTimeOn) || (time_off != lastTimeOff);
    
    // Reset if timing changed
    if (timingChanged) {
        blinkStartTime = 0;
        blinkState = false;
        lastTimeOn = time_on;
        lastTimeOff = time_off;
    }
    
    // First time called or reset
    if (blinkStartTime == 0) {
        blinkStartTime = currentTime;
        blinkOnTime = time_on;
        blinkOffTime = time_off;
        blinkState = false;
        allOff();
        return true;
    }

    uint32_t elapsedTime = currentTime - blinkStartTime;

    if (!blinkState && elapsedTime >= blinkOffTime) {
        // Switch to on state
        allOn();
        blinkState = true;
        blinkStartTime = currentTime;
    }
    else if (blinkState && elapsedTime >= blinkOnTime) {
        // Switch to off state
        allOff();
        blinkState = false;
        blinkStartTime = currentTime;
    }

    return true;
}

// Continuous rotate pattern for calibration
bool DroneLedsController::updateRotatePattern() {
    uint32_t currentTime = to_ms_since_boot(get_absolute_time());
    
    // 200ms per step
    if (rotateStartTime == 0 || currentTime - rotateStartTime >= 200) {
        // Reset LED state
        allOff();

        // Rotate through LEDs
        switch(rotateStep) {
            case 0: setFrontRight(true); break;
            case 1: setBackRight(true); break;
            case 2: setBackLeft(true); break;
            case 3: setFrontLeft(true); break;
        }

        // Update rotation state
        rotateStep = (rotateStep + 1) % 4;
        rotateStartTime = currentTime;
    }

    return true;
}

// Stop rotate pattern and turn off LEDs
void DroneLedsController::stopRotatePattern() {
    rotateStartTime = 0;
    rotateStep = 0;
    allOff();
}