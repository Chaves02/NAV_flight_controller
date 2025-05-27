#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "rp_agrolib_motors.h"
#include "rp_agrolib_bmi088.h"
#include "rp_agrolib_vl53l8cx_api.h"
#include "rp_agrolib_i2c.h"

#define I2C_SDA1 18     // I2C1 SDA on GPIO18
#define I2C_SCL1 19     // I2C1 SCL on GPIO19
#define I2C_BAUD 400000 // 400kHz I2C speed

#define BMI088_ACCL_ADDR 0x18
#define BMI088_GYRO_ADDR 0x68
#define GYRO_INT_PIN 20
#define ACCEL_INT_PIN 21
#define LIDAR_ADDR 0x29 // VL53L8CX I2C address

#define PWM_FREQ 100 // 400Hz for ESCs
#define NUM_MOTORS 4

//     Front
//
//    1     2
//     \   /
//      \ /
//      / \
//     /   \
//    3     4
//
//     Back

// Motor output pins
const uint8_t MOTOR_PINS[NUM_MOTORS] = {2, 24, 10, 14}; // Adjust these to your setup

// PID constants
struct PID_Constants
{
    float Kp;
    float Ki;
    float Kd;
    float outputLimit;
};

// PID controllers for each axis
PID_Constants ratePID[3] = {
    {0.5, 0.0, 0.01, 400.0}, // Roll
    {0.5, 0.0, 0.01, 400.0}, // Pitch
    {1.0, 0.0, 0.05, 400.0}  // Yaw
};

// PID controller for altitude
PID_Constants altitudePID = {0.3, 0.05, 0.1, 0.8}; // Tune these values

float altitude = 0;   // Current height in meters
float dt = 0.01;      // 10ms loop time

// Initialize I2C
i2c_inst_t *i2c = init_i2c1(I2C_SDA1, I2C_SCL1, I2C_BAUD, true);

// Initialize sensors
Bmi088 bmi088(i2c, BMI088_ACCL_ADDR, BMI088_GYRO_ADDR);
VL53L8CX_Configuration Dev;           // ToF sensor configuration
VL53L8CX_ResultsData Results;         // ToF results data
uint8_t status, isReady, isAlive;              // For ranging status

// Altitude control variables
float targetAltitude = 0.20; // Target altitude in meters
float minAltitude = 0.05;    // Minimum altitude for landing
float maxAltitude = 1.5;     // Maximum allowed altitude
float takeoffThrottle = 0.4; // Initial throttle for takeoff
float hoverThrottle = 0.3;   // Base throttle when hovering
float landingRate = 0.05;    // How fast to decrease altitude when landing (m/s)

// Control variables
float throttle = 0;     // Current throttle (0 to 1)
float rollCommand = 0;  // Roll rate command
float pitchCommand = 0; // Pitch rate command
float yawCommand = 0;   // Yaw rate command

// Motor outputs (range 0 to 1000)
uint16_t motorOutput[NUM_MOTORS] = {0};

// Flight state
enum FlightMode
{
    IDLE,
    TAKEOFF,
    HOVER,
    LANDING,
    EMERGENCY
};

FlightMode currentMode = IDLE;
bool armed = false;
float loopFrequency = 0;
absolute_time_t lastLoopTime;

// PID variables
float lastError[3] = {0};    // For rate PIDs
float errorSum[3] = {0};     // For rate PIDs
float lastAltitudeError = 0; // For altitude PID
float altitudeErrorSum = 0;  // For altitude PID

// Function prototypes
void setup();
void loop();
void calculatePID();
void mixMotors();
void writeMotors();
void processCommand();
void printStatus();
void updateAltitudeControl();

void readBMI088()
{
    bmi088.readSensor();
    bmi088.updateRPY();
}

// Read data from VL53L8CX
void readVL53L8CX() {
    // Check if new measurement is ready
    status = vl53l8cx_check_data_ready(&Dev, &isReady);
    
    if (isReady) {
        // Get ranging data
        vl53l8cx_get_ranging_data(&Dev, &Results);
        
        // Initialize minimum distance to a large value
        uint16_t min_distance_mm = UINT16_MAX;
        bool valid_measurement_found = false;
        
        // Iterate through all zones (4x4 grid = 16 zones)
        for (int zone = 0; zone < 16; zone++) {
            // Get distance and status for current zone
            uint16_t distance_mm = Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * zone];
            uint8_t status = Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * zone];
            
            // Check if measurement is valid (status = 5 or 9 typically means valid)
            // Check VL53L8CX documentation for full list of valid status codes
            if ((status == 5 || status == 9) && distance_mm > 0) {
                // Update minimum distance if this zone has a valid shorter distance
                if (distance_mm < min_distance_mm) {
                    min_distance_mm = distance_mm;
                    valid_measurement_found = true;
                }
            }
        }
        
        // Only update altitude if at least one valid measurement was found
        if (valid_measurement_found) {
            // Convert mm to meters
            altitude = min_distance_mm / 1000.0f;
            
            // Simple filtering to reduce noise (adjust alpha as needed)
            static float filtered_altitude = 0.0f;
            const float alpha = 0.1f; // Smoothing factor
            filtered_altitude = alpha * altitude + (1.0f - alpha) * filtered_altitude;
            altitude = filtered_altitude;
            
            // Optional debugging - comment out in final version
            //static uint32_t lidarTimer = 0;
            //if (time_us_32() - lidarTimer > 500000) { // Print every 0.5 seconds
            //    printf("Min Distance: %.3f m\n", altitude);
            //    lidarTimer = time_us_32();
            //}
        }
    }
}

// PID controller implementation for rate control
float updateRatePID(int axis, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    // Calculate P term
    float pTerm = ratePID[axis].Kp * error;

    // Calculate I term
    errorSum[axis] += error * dt;
    float iTerm = ratePID[axis].Ki * errorSum[axis];

    // Calculate D term
    float dTerm = ratePID[axis].Kd * (error - lastError[axis]) / dt;
    lastError[axis] = error;

    // Calculate total
    float output = pTerm + iTerm + dTerm;

    // Limit output
    if (output > ratePID[axis].outputLimit)
        output = ratePID[axis].outputLimit;
    if (output < -ratePID[axis].outputLimit)
        output = -ratePID[axis].outputLimit;

    return output;
}

// PID controller implementation for altitude control
float updateAltitudePID(float setpoint, float measurement)
{
    float error = setpoint - measurement;

    // Calculate P term
    float pTerm = altitudePID.Kp * error;

    // Calculate I term
    altitudeErrorSum += error * dt;
    float iTerm = altitudePID.Ki * altitudeErrorSum;

    // Calculate D term
    float dTerm = altitudePID.Kd * (error - lastAltitudeError) / dt;
    lastAltitudeError = error;

    // Calculate total
    float output = pTerm + iTerm + dTerm;

    // Limit output
    if (output > altitudePID.outputLimit)
        output = altitudePID.outputLimit;
    if (output < -altitudePID.outputLimit)
        output = -altitudePID.outputLimit;

    return output;
}

// Update altitude control based on flight mode
void updateAltitudeControl()
{
    float altitudeThrottle = 0;

    switch (currentMode)
    {
    case TAKEOFF:
        // During takeoff, gradually increase throttle until reaching target altitude
        if (altitude < targetAltitude)
        {
            throttle = takeoffThrottle;
        }
        else
        {
            // Reached target altitude, switch to hover mode
            currentMode = HOVER;
            printf("Target altitude reached. Switching to HOVER mode\n");
        }
        break;

    case HOVER:
        // Use PID to maintain target altitude
        altitudeThrottle = updateAltitudePID(targetAltitude, altitude);
        throttle = hoverThrottle + altitudeThrottle;

        // Safety check for maximum altitude
        if (altitude > maxAltitude)
        {
            printf("WARNING: Maximum altitude exceeded. Starting landing sequence.\n");
            currentMode = LANDING;
        }
        break;

    case LANDING:
        // Gradually decrease target altitude
        targetAltitude -= landingRate * dt;
        if (targetAltitude < minAltitude)
        {
            targetAltitude = minAltitude;
        }

        // Use PID to control descent
        altitudeThrottle = updateAltitudePID(targetAltitude, altitude);
        throttle = hoverThrottle + altitudeThrottle;

        // Check if we've landed
        if (altitude <= minAltitude)
        {
            throttle = 0;
            armed = false;
            currentMode = IDLE;
            printf("Landing complete. Motors disarmed.\n");
        }
        break;

    case EMERGENCY:
        // Cut power immediately
        throttle = 0;
        armed = false;
        currentMode = IDLE;
        printf("Emergency stop executed. Motors stopped.\n");
        break;

    case IDLE:
    default:
        // Motors not running
        throttle = 0;
        break;
    }

    // Constrain throttle
    if (throttle > 1.0f)
        throttle = 1.0f;
    if (throttle < 0.0f)
        throttle = 0.0f;
}

// Calculate motor outputs based on PID results
void calculateMotorOutputs(float rollPID, float pitchPID, float yawPID)
{
    // Convert throttle from range 0 to 1 to range 0 to 1000
    float throttleOutput = throttle * 1000.0f;

    // Motor mixing for quadcopter (X configuration)
    // Motor order: Front Right, Front Left, Rear Left, Rear Right
    motorOutput[0] = throttleOutput - rollPID + pitchPID - yawPID; // Front Right
    motorOutput[1] = throttleOutput + rollPID + pitchPID + yawPID; // Front Left
    motorOutput[2] = throttleOutput + rollPID - pitchPID - yawPID; // Rear Left
    motorOutput[3] = throttleOutput - rollPID - pitchPID + yawPID; // Rear Right

    // Constrain motor outputs
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (motorOutput[i] > 1000)
            motorOutput[i] = 1000;
        if (motorOutput[i] < 0)
            motorOutput[i] = 0;

        // If not armed, set motors to zero
        if (!armed)
            motorOutput[i] = 0;
    }
}

// Write motor outputs to ESCs
void writeMotors()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        set_motor_speed(MOTOR_PINS[i], motorOutput[i]);
    }
}

// Process commands from serial
void processCommand()
{
    if (stdio_usb_connected() && getchar_timeout_us(0) != PICO_ERROR_TIMEOUT)
    {
        char cmd[32];
        int result = scanf("%31s", cmd);

        if (result > 0)
        {
            if (strcmp(cmd, "takeoff") == 0)
            {
                float height;
                if (scanf("%f", &height) == 1)
                {
                    if (height > 0 && height <= maxAltitude)
                    {
                        targetAltitude = height;
                    }
                    else
                    {
                        printf("Invalid altitude. Must be between 0 and %f meters\n", maxAltitude);
                        return;
                    }
                }
                else
                {
                    // Use default target altitude if no value provided
                    targetAltitude = 0.2f;
                }

                if (!armed)
                {
                    armed = true;
                }
                currentMode = TAKEOFF;
                printf("Starting takeoff to %f meters\n", targetAltitude);
            }
            else if (strcmp(cmd, "land") == 0)
            {
                if (currentMode != IDLE)
                {
                    currentMode = LANDING;
                    printf("Starting landing sequence\n");
                }
                else
                {
                    printf("Not currently flying\n");
                }
            }
            else if (strcmp(cmd, "hover") == 0)
            {
                if (armed && (currentMode == TAKEOFF || currentMode == HOVER || currentMode == LANDING))
                {
                    float height;
                    if (scanf("%f", &height) == 1)
                    {
                        if (height > 0 && height <= maxAltitude)
                        {
                            targetAltitude = height;
                            currentMode = HOVER;
                            printf("Adjusting hover altitude to %f meters\n", targetAltitude);
                        }
                        else
                        {
                            printf("Invalid altitude. Must be between 0 and %f meters\n", maxAltitude);
                        }
                    }
                    else
                    {
                        // Stay at current target altitude
                        currentMode = HOVER;
                        printf("Maintaining current altitude of %f meters\n", targetAltitude);
                    }
                }
                else
                {
                    printf("Not currently in flight\n");
                }
            }
            else if (strcmp(cmd, "emergency") == 0)
            {
                currentMode = EMERGENCY;
                printf("EMERGENCY STOP\n");
            }
            else if (strcmp(cmd, "status") == 0)
            {
                printStatus();
            }
            else if (strcmp(cmd, "setpid") == 0)
            {
                char type[10];
                if (scanf("%9s", type) == 1)
                {
                    if (strcmp(type, "roll") == 0)
                    {
                        float kp, ki, kd;
                        if (scanf("%f %f %f", &kp, &ki, &kd) == 3)
                        {
                            ratePID[0].Kp = kp;
                            ratePID[0].Ki = ki;
                            ratePID[0].Kd = kd;
                            printf("Roll PID set to P=%f, I=%f, D=%f\n", kp, ki, kd);
                        }
                    }
                    else if (strcmp(type, "pitch") == 0)
                    {
                        float kp, ki, kd;
                        if (scanf("%f %f %f", &kp, &ki, &kd) == 3)
                        {
                            ratePID[1].Kp = kp;
                            ratePID[1].Ki = ki;
                            ratePID[1].Kd = kd;
                            printf("Pitch PID set to P=%f, I=%f, D=%f\n", kp, ki, kd);
                        }
                    }
                    else if (strcmp(type, "yaw") == 0)
                    {
                        float kp, ki, kd;
                        if (scanf("%f %f %f", &kp, &ki, &kd) == 3)
                        {
                            ratePID[2].Kp = kp;
                            ratePID[2].Ki = ki;
                            ratePID[2].Kd = kd;
                            printf("Yaw PID set to P=%f, I=%f, D=%f\n", kp, ki, kd);
                        }
                    }
                    else if (strcmp(type, "alt") == 0)
                    {
                        float kp, ki, kd;
                        if (scanf("%f %f %f", &kp, &ki, &kd) == 3)
                        {
                            altitudePID.Kp = kp;
                            altitudePID.Ki = ki;
                            altitudePID.Kd = kd;
                            printf("Altitude PID set to P=%f, I=%f, D=%f\n", kp, ki, kd);
                        }
                    }
                    else
                    {
                        printf("Unknown PID type. Use roll, pitch, yaw, or alt\n");
                    }
                }
            }
            else if (strcmp(cmd, "trim") == 0)
            {
                float roll, pitch;
                if (scanf("%f %f", &roll, &pitch) == 2)
                {
                    rollCommand = roll;
                    pitchCommand = pitch;
                    printf("Trim set to roll=%f, pitch=%f\n", roll, pitch);
                }
            }
            else if (strcmp(cmd, "help") == 0)
            {
                printf("Available commands:\n");
                printf("  takeoff [height] - Take off to specified height (default 0.2m)\n");
                printf("  land - Start landing sequence\n");
                printf("  hover [height] - Hover at current or specified height\n");
                printf("  emergency - Emergency stop (cut motors)\n");
                printf("  status - Print current status\n");
                printf("  setpid [type] [kp] [ki] [kd] - Set PID values (type=roll/pitch/yaw/alt)\n");
                printf("  trim [roll] [pitch] - Set roll and pitch trim values\n");
                printf("  help - Show this help\n");
            }
            else
            {
                printf("Unknown command: %s\n", cmd);
                printf("Type 'help' for available commands\n");
            }
        }
    }
}

// Print current status
void printStatus()
{
    // Determine current mode string
    const char *modeString;
    switch (currentMode)
    {
    case IDLE:
        modeString = "IDLE";
        break;
    case TAKEOFF:
        modeString = "TAKEOFF";
        break;
    case HOVER:
        modeString = "HOVER";
        break;
    case LANDING:
        modeString = "LANDING";
        break;
    case EMERGENCY:
        modeString = "EMERGENCY";
        break;
    default:
        modeString = "UNKNOWN";
    }

    printf("=== Flight Controller Status ===\n");
    printf("Mode: %s\n", modeString);
    printf("Armed: %s\n", armed ? "YES" : "NO");
    printf("Loop Frequency: %.2f Hz\n", loopFrequency);
    printf("Altitude: Current=%.3f, Target=%.3f m\n", altitude, targetAltitude);
    printf("Gyro: Roll=%.2f, Pitch=%.2f, Yaw=%.2f deg/s\n", bmi088.getRoll(), bmi088.getPitch(), bmi088.getYaw());
    printf("Accel: X=%.2f, Y=%.2f, Z=%.2f m/s^2\n", bmi088.getAccelX_mss(), bmi088.getAccelY_mss(), bmi088.getAccelZ_mss());
    printf("Throttle: %.3f\n", throttle);
    printf("Motor outputs: %d, %d, %d, %d\n",
           motorOutput[0], motorOutput[1], motorOutput[2], motorOutput[3]);
    printf("PID values:\n");
    printf("  Roll: P=%.2f, I=%.2f, D=%.2f\n",
           ratePID[0].Kp, ratePID[0].Ki, ratePID[0].Kd);
    printf("  Pitch: P=%.2f, I=%.2f, D=%.2f\n",
           ratePID[1].Kp, ratePID[1].Ki, ratePID[1].Kd);
    printf("  Yaw: P=%.2f, I=%.2f, D=%.2f\n",
           ratePID[2].Kp, ratePID[2].Ki, ratePID[2].Kd);
    printf("  Altitude: P=%.2f, I=%.2f, D=%.2f\n",
           altitudePID.Kp, altitudePID.Ki, altitudePID.Kd);
}

// Setup function
void setup()
{
    // Initialize serial
    stdio_init_all();
    sleep_ms(3000); // Allow time for USB enumeration
    printf("\nAltitude-Based Flight Controller starting...\n");

    // Initialize sensors
    int stat;
    stat = bmi088.begin();
    if (stat < 0)
    {
        printf("BMI_Error: %d\n", stat);
        while (1){}        
    }

    // Initialize VL53L8CX sensor
    Dev.platform.address = LIDAR_ADDR; /* Default address */
    Dev.platform.i2c_instance = i2c;   /* I2C instance */

    // Init VL53L8CX sensor
    status = vl53l8cx_init(&Dev);
    if (status)
    {
        printf("VL53L8CX ULD Loading failed\n");
        while (1){}
    }

    // Initialize motor PWM
    setup_motors();

    // Test motors
    motor_test();

    sleep_ms(1000);

    // Initialize other variables
    lastLoopTime = get_absolute_time();

    printf("Flight controller initialized and ready!\n");
    printf("Type 'help' for available commands\n");
    sleep_ms(1000);
}

// Main loop
void loop()
{
    // Calculate loop time
    absolute_time_t currentTime = get_absolute_time();
    dt = absolute_time_diff_us(lastLoopTime, currentTime) / 1000000.0f;
    lastLoopTime = currentTime;
    loopFrequency = 1.0f / dt;

    // Read sensors
    readBMI088();
    readVL53L8CX();

    // Handle altitude control based on current mode
    updateAltitudeControl();

    // Calculate PID for each rate axis
    float rollPID = updateRatePID(0, rollCommand * 200.0f, bmi088.getRoll());
    float pitchPID = updateRatePID(1, pitchCommand * 200.0f, bmi088.getPitch());
    float yawPID = updateRatePID(2, yawCommand * 200.0f, bmi088.getYaw());

    // Calculate motor outputs
    calculateMotorOutputs(rollPID, pitchPID, yawPID);

    // Write motor outputs
    writeMotors();

    // Process commands
    processCommand();

    // Print status occasionally
    static uint32_t statusTimer = 0;
    if (time_us_32() - statusTimer > 1000000)
    { // Every 1 second
        printStatus();
        statusTimer = time_us_32();
    }

    // Ensure consistent loop time
    sleep_ms(5); // 100Hz loop rate
}

// Main function
int main()
{
    setup();

    status = vl53l8cx_start_ranging(&Dev);
    //printf("Start Ranging status: %d\n", status);

    while (true)
    {
        loop();
    }

    return 0;
}