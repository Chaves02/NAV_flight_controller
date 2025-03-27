#ifndef RCL_SIMULATION_H
#define RCL_SIMULATION_H

#include <cmath>
#include <algorithm>

class RCLSimulation {
public:
    // Simulation modes
    enum SimulationMode {
        MANUAL,         // Manual input control
        SINE_WAVE,      // Generates sine wave inputs
        SQUARE_WAVE,    // Generates square wave inputs
        STEP_RESPONSE,  // Generates step response-like inputs
        RANDOM_WALK     // Generates random walk inputs
    };

    // Constructor
    RCLSimulation();

    // Input components
    float throttle = 0.0;
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
    bool arm = false;

    // Methods to interact with simulation
    void setMode(SimulationMode mode);
    void update(float deltaTime);
    
    // Manual input methods
    void setManualInputs(float _throttle, float _roll, float _pitch, float _yaw, bool _arm);
    
    // Connection simulation
    bool connected() const;
    
    // Reset to initial state
    void reset();

private:
    // Simulation state
    SimulationMode currentMode = MANUAL;
    float simulationTime = 0.0;
    
    // Sine wave generation
    void generateSineWaveInputs(float deltaTime);
    void generateSquareWaveInputs(float deltaTime);
    void generateStepResponseInputs(float deltaTime);
};

// Global instance for compatibility with existing code
extern RCLSimulation rcl;

#endif // RCL_SIMULATION_H