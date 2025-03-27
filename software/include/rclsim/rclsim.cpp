#include "rclsim.h"
#include <random>
#include <algorithm>

RCLSimulation rcl;  // Global instance

RCLSimulation::RCLSimulation() {
    reset();
}

void RCLSimulation::setMode(SimulationMode mode) {
    currentMode = mode;
    simulationTime = 0.0;
    reset();
}

void RCLSimulation::update(float deltaTime) {
    simulationTime += deltaTime;

    switch(currentMode) {
        case SINE_WAVE:
            generateSineWaveInputs(deltaTime);
            break;
        case SQUARE_WAVE:
            generateSquareWaveInputs(deltaTime);
            break;
        case STEP_RESPONSE:
            generateStepResponseInputs(deltaTime);
            break;
        case MANUAL:
        default:
            setManualInputs(0.30, 0.0, 0.0, 0.0, simulationTime > 3.0);
            break;
    }
}

void RCLSimulation::generateSineWaveInputs(float deltaTime) {
    // Sine wave inputs with different frequencies
    throttle = 0.5 + 0.4 * sin(simulationTime);
    roll = 0.3 * sin(simulationTime * 1.5);
    pitch = 0.3 * cos(simulationTime * 1.5);
    yaw = 0.2 * sin(simulationTime * 2);
    
    // Auto-arm after a delay
    arm = (simulationTime > 2.0);
}

void RCLSimulation::generateSquareWaveInputs(float deltaTime) {
    // Square wave inputs
    throttle = (sin(simulationTime) > 0) ? 0.7 : 0.3;
    roll = (sin(simulationTime * 2) > 0) ? 0.5 : -0.5;
    pitch = (cos(simulationTime * 2) > 0) ? 0.5 : -0.5;
    yaw = (sin(simulationTime * 3) > 0) ? 0.4 : -0.4;
    
    arm = (simulationTime > 1.0);
}

void RCLSimulation::generateStepResponseInputs(float deltaTime) {
    // Simulate step response with gradual changes
    throttle = std::min(1.0f, simulationTime / 5.0f);
    roll = std::sin(simulationTime) * 0.5f;
    pitch = (simulationTime > 2.0) ? 0.3f : 0.0f;
    yaw = (simulationTime > 3.0) ? 0.2f : 0.0f;
    
    arm = (simulationTime > 1.0);
}

void RCLSimulation::setManualInputs(float _throttle, float _roll, float _pitch, float _yaw, bool _arm) {
    throttle = std::clamp(_throttle, 0.0f, 1.0f);
    roll = std::clamp(_roll, -1.0f, 1.0f);
    pitch = std::clamp(_pitch, -1.0f, 1.0f);
    yaw = std::clamp(_yaw, -1.0f, 1.0f);
    arm = _arm;
}

bool RCLSimulation::connected() const {
    // In simulation, always connected
    return true;
}

void RCLSimulation::reset() {
    throttle = 0.0f;
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    arm = false;
    simulationTime = 0.0f;
}