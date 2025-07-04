#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/multicore.h"
#include "pico/critical_section.h"
#include <time.h>
#include <cmath>
#include <cstring>
#include <cstdlib> // For strtof

#include "rp_agrolib_motors.h"
// M1 = 24, M2 = 14, M3 = 10, M4 = 2
bool armed = false; // Flag to indicate if motors are armed

#include "rp_agrolib_uart.h"
#include "rp_agrolib_bc832.h"
#define BLE_MODE_PIN 9

// Global BLE instance
RP_AGROLIB_BC832_Simple *ble_instance = nullptr;

// UART configuration for Bluetooth module
#define UART_ID uart0
#define UART_RX 17
#define UART_TX 16
#define UART_BAUDRATE 115200

// Buffer settings
#define MAX_BUFFER_LEN 128
#define TIMEOUT_US 50000 // 50ms timeout

// Expected message format from smartphone/docking
// Format: "RC,roll,pitch,throttle,yaw\n"
// Example: "RC,1500,1500,1000,1500\n"

/*   receiver/controller   */
float ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber = 0;

//************************Critical Section for Thread Safety**********************************//
critical_section_t rc_data_cs;
critical_section_t telemetry_cs;
// Core 1 status flag
static volatile bool core1_running = false;

//********************************Receiver Data Structure**++********************************//

typedef struct
{
    float roll;
    float pitch;
    float throttle;
    float yaw;
    bool data_valid;
    uint64_t last_update;
} rc_data_t;

static volatile rc_data_t rc_data = {1500.0f, 1500.0f, 1500.0f, 1500.0f, false, 0};

//********************************Telemetry Data Structure**********************************//
typedef struct
{
    // Angles and rates
    float kalman_angle_roll;
    float kalman_angle_pitch;
    float rate_roll;
    float rate_pitch;
    float rate_yaw;

    // PID errors
    float error_angle_roll;
    float error_angle_pitch;
    float error_rate_roll;
    float error_rate_pitch;
    float error_rate_yaw;

    // Motor outputs
    float motor_input1;
    float motor_input2;
    float motor_input3;
    float motor_input4;

    // Altitude data
    float altitude_kalman;
    float velocity_vertical_kalman;
    float altitude_lidar;

    // System status
    bool armed;

    // Timestamp
    uint64_t timestamp;
} telemetry_data_t;

static volatile telemetry_data_t telemetry_data = {0};

// Improved parsing function with better error handling
bool parse_rc_command(const char *cmd)
{
    // Remove any leading whitespace
    while (*cmd == ' ' || *cmd == '\t')
        cmd++;

    // Handle different prefixes
    if (strncmp(cmd, "+B", 2) == 0)
    {
        cmd += 2;
        // Skip any special character after +B
        if (*cmd != 'R')
            cmd++;
    }

    // Check if the command starts with "RC,"
    if (strncmp(cmd, "RC,", 3) != 0)
    {
        return false;
    }

    // printf("Parsing RC command: %s\n", cmd);
    const char *ptr = cmd + 3; // Skip "RC,"

    // Temporary variables to validate all values before updating
    float temp_values[4];

    // Parse all 4 values
    for (int i = 0; i < 4; i++)
    {
        char *endptr;
        temp_values[i] = strtof(ptr, &endptr);

        // Check if parsing was successful
        if (ptr == endptr)
        {
            // printf("Parse error at value %d\n", i);
            return false;
        }

        // Validate range (typical RC values are 1000-2000)
        if (temp_values[i] < 0 || temp_values[i] > 2500)
        {
            printf("Value %d out of range: %.1f\n", i, temp_values[i]);
            return false;
        }

        // Move to next value (skip comma)
        if (i < 3)
        {
            ptr = strchr(endptr, ',');
            if (!ptr)
            {
                // printf("Missing comma after value %d\n", i);
                return false;
            }
            ptr++; // Skip comma
        }
    }

    // All values are valid, update atomically using critical section
    critical_section_enter_blocking(&rc_data_cs);
    rc_data.roll = temp_values[0];
    rc_data.pitch = temp_values[1];
    rc_data.throttle = temp_values[2];
    rc_data.yaw = temp_values[3];
    rc_data.data_valid = true;
    rc_data.last_update = time_us_64();
    critical_section_exit(&rc_data_cs);

    return true;
}

// Function to update telemetry data (called from core 0)
void update_telemetry_data(float kalman_angle_roll, float kalman_angle_pitch,
                           float rate_roll, float rate_pitch, float rate_yaw,
                           float error_angle_roll, float error_angle_pitch,
                           float error_rate_roll, float error_rate_pitch, float error_rate_yaw,
                           float motor_input1, float motor_input2, float motor_input3, float motor_input4,
                           float altitude_kalman, float velocity_vertical_kalman, float altitude_lidar,
                           bool armed_status)
{

    critical_section_enter_blocking(&telemetry_cs);
    telemetry_data.kalman_angle_roll = kalman_angle_roll;
    telemetry_data.kalman_angle_pitch = kalman_angle_pitch;
    telemetry_data.rate_roll = rate_roll;
    telemetry_data.rate_pitch = rate_pitch;
    telemetry_data.rate_yaw = rate_yaw;

    telemetry_data.error_angle_roll = error_angle_roll;
    telemetry_data.error_angle_pitch = error_angle_pitch;
    telemetry_data.error_rate_roll = error_rate_roll;
    telemetry_data.error_rate_pitch = error_rate_pitch;
    telemetry_data.error_rate_yaw = error_rate_yaw;

    telemetry_data.motor_input1 = motor_input1;
    telemetry_data.motor_input2 = motor_input2;
    telemetry_data.motor_input3 = motor_input3;
    telemetry_data.motor_input4 = motor_input4;

    telemetry_data.altitude_kalman = altitude_kalman;
    telemetry_data.velocity_vertical_kalman = velocity_vertical_kalman;
    telemetry_data.altitude_lidar = altitude_lidar;

    telemetry_data.armed = armed_status;
    telemetry_data.timestamp = time_us_64();
    critical_section_exit(&telemetry_cs);
}

// Function to send telemetry data (called from core 1)
void send_telemetry()
{
    // Create local copy of telemetry data
    telemetry_data_t local_telemetry;

    critical_section_enter_blocking(&telemetry_cs);
    local_telemetry.kalman_angle_roll = telemetry_data.kalman_angle_roll;
    local_telemetry.kalman_angle_pitch = telemetry_data.kalman_angle_pitch;
    local_telemetry.rate_roll = telemetry_data.rate_roll;
    local_telemetry.rate_pitch = telemetry_data.rate_pitch;
    local_telemetry.rate_yaw = telemetry_data.rate_yaw;

    local_telemetry.error_angle_roll = telemetry_data.error_angle_roll;
    local_telemetry.error_angle_pitch = telemetry_data.error_angle_pitch;
    local_telemetry.error_rate_roll = telemetry_data.error_rate_roll;
    local_telemetry.error_rate_pitch = telemetry_data.error_rate_pitch;
    local_telemetry.error_rate_yaw = telemetry_data.error_rate_yaw;

    local_telemetry.motor_input1 = telemetry_data.motor_input1;
    local_telemetry.motor_input2 = telemetry_data.motor_input2;
    local_telemetry.motor_input3 = telemetry_data.motor_input3;
    local_telemetry.motor_input4 = telemetry_data.motor_input4;

    local_telemetry.altitude_kalman = telemetry_data.altitude_kalman;
    local_telemetry.velocity_vertical_kalman = telemetry_data.velocity_vertical_kalman;
    local_telemetry.altitude_lidar = telemetry_data.altitude_lidar;

    local_telemetry.armed = telemetry_data.armed;
    local_telemetry.timestamp = telemetry_data.timestamp;
    critical_section_exit(&telemetry_cs);

    // Create telemetry string in CSV format for easy parsing
    char telemetry_buffer[256];

    snprintf(telemetry_buffer, sizeof(telemetry_buffer),
             "TELEM,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f,%.1f,%.1f,%.1f,%dt",
             // Angles and rates
             local_telemetry.kalman_angle_roll, local_telemetry.kalman_angle_pitch,
             local_telemetry.rate_roll, local_telemetry.rate_pitch, local_telemetry.rate_yaw,
             // PID errors
             local_telemetry.error_angle_roll, local_telemetry.error_angle_pitch,
             local_telemetry.error_rate_roll, local_telemetry.error_rate_pitch, local_telemetry.error_rate_yaw,
             // Motor outputs
             local_telemetry.motor_input1, local_telemetry.motor_input2, local_telemetry.motor_input3, local_telemetry.motor_input4,
             // Altitude data
             local_telemetry.altitude_kalman, local_telemetry.velocity_vertical_kalman,
             local_telemetry.altitude_lidar,
             // System status
             local_telemetry.armed ? 1 : 0);

    // Send via BLE (if connected)
    if (ble_instance->isConnected())
    {
        ble_instance->sendMessage(std::string(telemetry_buffer));
    }
}

// Improved UART reading with better buffer management
void uart_reader_core1()
{
    static char buffer[MAX_BUFFER_LEN];
    static int buffer_index = 0;
    static uint64_t last_char_time = 0;
    static uint64_t last_telemetry_time = 0;
    const uint64_t telemetry_interval = 200000; // 200ms = 5Hz

    printf("Core 1: UART reader started\n");
    core1_running = true;

    while (true)
    {

        uint64_t current_time = time_us_64();

        // Check for timeout on incomplete message
        if (buffer_index > 0 && (current_time - last_char_time) > TIMEOUT_US)
        {
            // printf("UART timeout, clearing buffer: %.*s\n", buffer_index, buffer);
            buffer_index = 0;
        }

        // Read all available characters
        while (uart_is_readable(UART_ID))
        {
            char c = uart_getc(UART_ID);
            last_char_time = current_time;

            // Skip null characters and other control chars except CR/LF
            if (c == 0 || (c < 32 && c != '\r' && c != '\n'))
            {
                continue;
            }

            // Check for end of message
            if (c == '\n' || c == '\r')
            {
                if (buffer_index > 0)
                {
                    buffer[buffer_index] = '\0'; // Null-terminate

                    // Process different message types
                    if (strncmp(buffer, "+C", 2) == 0)
                    {
                        ble_instance->processIncomingData(buffer, buffer_index);
                    }
                    else if (strncmp(buffer, "+D", 2) == 0)
                    {
                        ble_instance->processIncomingData(buffer, buffer_index);
                    }
                    else if (strstr(buffer, "RC,") != NULL)
                    {
                        // Handle RC command (with or without +B prefix)
                        parse_rc_command(buffer);
                    }
                    else
                    {
                        // printf("Unknown command: %s\n", buffer);
                    }

                    // Reset buffer for next message
                    buffer_index = 0;
                }
            }
            // Add character to buffer if there's room
            else if (buffer_index < MAX_BUFFER_LEN - 1)
            {
                buffer[buffer_index++] = c;
            }
            else
            {
                // Buffer overflow - reset and log
                // printf("UART buffer overflow, resetting\n");
                buffer_index = 0;
            }
        }

        // Send telemetry at 5Hz
        if (current_time - last_telemetry_time >= telemetry_interval)
        {
            if (armed)
            {
                send_telemetry();
                last_telemetry_time = current_time;
            }
        }

        sleep_us(100); // Yield to avoid busy-waiting
    }
}

// Thread-safe function to get RC values
void get_rc_values(float *roll, float *pitch, float *throttle, float *yaw, bool *valid)
{

    critical_section_enter_blocking(&rc_data_cs);
    *roll = rc_data.roll;
    *pitch = rc_data.pitch;
    *throttle = rc_data.throttle;
    *yaw = rc_data.yaw;
    *valid = rc_data.data_valid;

    // Check if data is stale (older than 500ms)
    uint64_t current_time = time_us_64();
    if (rc_data.data_valid && (current_time - rc_data.last_update) > 500000)
    {
        *valid = false;
        printf("RC data stale, disabling\n");
    }
    critical_section_exit(&rc_data_cs);
}

// Function to read receiver values
void read_receiver(void)
{

    bool valid;
    float temp_roll, temp_pitch, temp_throttle, temp_yaw;
    get_rc_values(&temp_roll, &temp_pitch, &temp_throttle, &temp_yaw, &valid);

    if (valid)
    {
        // Apply deadband to reduce oscillation around center values
        const float DEADBAND = 10.0f;      // ±10 units around center
        const float DEADBAND_YAW = 200.0f; // ±10 units for yaw

        // Center values (adjust based on your controller)
        const float CENTER_ROLL = 1500.0f;
        const float CENTER_PITCH = 1500.0f;
        const float CENTER_YAW = 1500.0f;

        // Apply deadband
        if (fabs(temp_roll - CENTER_ROLL) < DEADBAND)
        {
            temp_roll = CENTER_ROLL;
        }
        if (fabs(temp_pitch - CENTER_PITCH) < DEADBAND)
        {
            temp_pitch = CENTER_PITCH;
        }
        if (fabs(temp_yaw - CENTER_YAW) < DEADBAND_YAW)
        {
            temp_yaw = CENTER_YAW;
        }

        // Update receiver values
        ReceiverValue[0] = temp_roll;
        ReceiverValue[1] = temp_pitch;
        ReceiverValue[2] = temp_throttle;
        ReceiverValue[3] = temp_yaw;
    }
    else
    {
        // Use safe default values if no valid data
        ReceiverValue[0] = 1500.0f; // Roll center
        ReceiverValue[1] = 1500.0f; // Pitch center
        ReceiverValue[2] = 1000.0f; // Throttle off
        ReceiverValue[3] = 1500.0f; // Yaw center
    }
}

#define I2C_SDA1 18     // I2C1 SDA on GPIO18
#define I2C_SCL1 19     // I2C1 SCL on GPIO19
#define I2C_BAUD 400000 // 400kHz I2C speed

#define BMI088_ACCL_ADDR 0x18
#define BMI088_GYRO_ADDR 0x68
#define GYRO_INT_PIN 20
#define ACCEL_INT_PIN 21
#define LIDAR_ADDR 0x29 // VL53L8CX I2C address

#define LED_GREEN_L 3
#define LED_GREEN_R 25
#define LED_RED_L 11
#define LED_RED_R 15

/*   BMI088   */
#include "rp_agrolib_bmi088.h"
float RatePitch, RateRoll, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float RateCalibrationPitch = 0, RateCalibrationRoll = 0, RateCalibrationYaw = 0;
float AcclCalibrationX = 0, AcclCalibrationY = 0, AcclCalibrationZ = 0;
float AccZBias = 0; // Bias for Z acceleration, used in altitude hold
int RateCalibrationNumber = 0;
float RadtoDeg = 180 / M_PI;

uint32_t LoopTimer; // lengh of each control loop
float dt_kalman;    // Time step for Kalman filter, used to predict velocity

// Define predicted angles and uncertainties
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;

// Initialize output of filter
float Kalman1DOutput[] = {0 /*angle prediction*/, 0 /*uncertainty of the prediction*/};

// Define the desired roll and pitch angles and corresponding errors for the outer loop PID controller
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;

// Define the values necessary for the outer loop PID controller, including the P, I and D parameters
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll = 2;
float PAnglePitch = PAngleRoll; // 0.6PRoll
float IAngleRoll = 0;
float IAnglePitch = IAngleRoll;
float DAngleRoll = 0;
float DAnglePitch = 0;

// Create the function that calculates the predicted angle and uncertainty using Kalman equations
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
    KalmanState = KalmanState + 0.004 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;              // std desviation of accl 4º
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3); // std desviation of gyro 3º
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

    Kalman1DOutput[0] = KalmanState; // Kalman filter output
    Kalman1DOutput[1] = KalmanUncertainty;
}
// KalmanInput is the rotation rate from the gyro
// KalmanMeasurement is the angle from the accelerometer
// KalmanState is the angle calculated by the Kalman filter

/*   PID   */
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};
float PRateRoll = 1;
float PRatePitch = PRateRoll;
float PRateYaw = 45; // 0.6PRoll
float IRateRoll = 3;
float IRatePitch = IRateRoll;
float IRateYaw = 10;
float DRateRoll = 0.05;
float DRatePitch = DRateRoll;
float DRateYaw = 0.01; // 0Dyaw
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

/*   PID Equation   */
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm, float dt)
{
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * dt / 2; // dt is the time step -> 250Hz
    if (Iterm > 200)
        Iterm = 200;
    else if (Iterm < -200)
        Iterm = -200;
    float Dterm = D * (Error - PrevError) / dt;
    float PIDOutput = Pterm + Iterm + Dterm;
    if (PIDOutput > 200)
        PIDOutput = 200;
    else if (PIDOutput < -200)
        PIDOutput = -200;

    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}

#include "rp_agrolib_vl53l8cx_api.h"

// LIDAR variables
VL53L8CX_Configuration Dev;   // LIDAR configuration
VL53L8CX_ResultsData Results; // LIDAR results
uint8_t status, isAlive, isReady;

// Initialize LIDAR sensor
void init_lidar()
{
    printf("Initializing VL53L8CX LIDAR...\n");

    Dev.platform.address = LIDAR_ADDR; // Set I2C address
    Dev.platform.i2c_instance = init_i2c1(I2C_SDA1, I2C_SCL1, I2C_BAUD, true);

    // Check if sensor is alive
    status = vl53l8cx_is_alive(&Dev, &isAlive);
    if (!isAlive || status)
    {
        printf("VL53L8CX not detected: %d\n", status);
        return;
    }

    // Reset and initialize the sensor
    status = vl53l8cx_init(&Dev);
    if (status)
    {
        printf("VL53L8CX init failed: %d\n", status);
        return;
    }

    status = vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    if (status)
    {
        printf("VL53L8CX set resolution failed: %d\n", status);
        return;
    }

    status = vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    if (status)
    {
        printf("VL53L8CX set ranging mode failed: %d\n", status);
        return;
    }

    status = vl53l8cx_set_ranging_frequency_hz(&Dev, 60); // Set frequency to 50Hz
    if (status)
    {
        printf("VL53L8CX set ranging frequency failed: %d\n", status);
        return;
    }

    status = vl53l8cx_start_ranging(&Dev);
    if (status)
    {
        printf("VL53L8CX start ranging failed: %d\n", status);
        return;
    }

    printf("VL53L8CX initialized successfully\n");
}

float AltitudeLidar, AltitudeLidarStartUp = 0;
float AccZInertial;
float AltitudeKalman, VelocityVerticalKalman;

// Kalman filter matrices represented as arrays
// State vector S [altitude, velocity] - 2x1
float S[2] = {0.0f, 0.0f};

// State transition matrix F - 2x2
// F = [1, dt_kalman]
//     [0,     1]
float F[2][2] = {{1.0f, dt_kalman},
                 {0.0f, 1.0f}};

// Control input matrix G - 2x1
// G = [0.5*dt_kalman*dt_kalman] = [0.000008]
//     [dt_kalman         ]   [dt_kalman   ]
float G[2] = {0.5f * dt_kalman * dt_kalman, dt_kalman};

// Process covariance matrix P - 2x2
float P[2][2] = {{0.0f, 0.0f},
                 {0.0f, 0.0f}};

// Process noise covariance Q - 2x2
// Q = G * G^T * 10 * 10
float Q[2][2];

// Observation matrix H - 1x2
// H = [1, 0] (we only observe altitude)
float H[2] = {1.0f, 0.0f};

// Identity matrix I - 2x2
float I[2][2] = {{1.0f, 0.0f},
                 {0.0f, 1.0f}};

// Measurement noise covariance R - 1x1 (scalar)
// R = 1 * 1 = 1.0
float R = 1.0f * 1.0f; // Measurement noise covariance (altitude)

// Temporary matrices for calculations
float temp_2x2[2][2];
float temp_2x1[2];
float K[2];       // Kalman gain - 2x1
float innovation; // Innovation (scalar)
float S_pred;     // Innovation covariance (scalar)

// PID velocity controller variables
float DesiredVelocityVertical, ErrorVelocityVertical;
float PVelocityVertical = 6.0f;
float IVelocityVertical = 3.0f;
float DVelocityVertical = 0.01f;
float PrevErrorVelocityVertical, PrevItermVelocityVertical;

// Matrix operations
void matrix_multiply_2x2(float A[2][2], float B[2][2], float result[2][2])
{
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    result[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
    result[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1];
}

void matrix_multiply_2x2_transpose(float A[2][2], float B[2][2], float result[2][2])
{
    // A * B^T
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[0][1];
    result[0][1] = A[0][0] * B[1][0] + A[0][1] * B[1][1];
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[0][1];
    result[1][1] = A[1][0] * B[1][0] + A[1][1] * B[1][1];
}

void matrix_multiply_2x1(float A[2][2], float b[2], float result[2])
{
    result[0] = A[0][0] * b[0] + A[0][1] * b[1];
    result[1] = A[1][0] * b[0] + A[1][1] * b[1];
}

void matrix_add_2x2(float A[2][2], float B[2][2], float result[2][2])
{
    result[0][0] = A[0][0] + B[0][0];
    result[0][1] = A[0][1] + B[0][1];
    result[1][0] = A[1][0] + B[1][0];
    result[1][1] = A[1][1] + B[1][1];
}

void matrix_subtract_2x2(float A[2][2], float B[2][2], float result[2][2])
{
    result[0][0] = A[0][0] - B[0][0];
    result[0][1] = A[0][1] - B[0][1];
    result[1][0] = A[1][0] - B[1][0];
    result[1][1] = A[1][1] - B[1][1];
}

void kalman_2d(void)
{

    // Update time-dependent matrices
    F[0][1] = dt_kalman; // Update state transition matrix

    // Update control input matrix
    G[0] = 0.5f * dt_kalman * dt_kalman;
    G[1] = dt_kalman;

    // Update process noise covariance matrix
    float process_noise_std = 1000.0f; // Standard deviation of accl noise
    float process_noise_var = process_noise_std * process_noise_std;
    Q[0][0] = G[0] * G[0] * process_noise_var;
    Q[0][1] = G[0] * G[1] * process_noise_var;
    Q[1][0] = G[1] * G[0] * process_noise_var;
    Q[1][1] = G[1] * G[1] * process_noise_var;

    // Prediction step
    // S = F * S + G * AccZInertial
    matrix_multiply_2x1(F, S, temp_2x1);
    S[0] = temp_2x1[0] + G[0] * AccZInertial;
    S[1] = temp_2x1[1] + G[1] * AccZInertial;

    // P = F * P * F^T + Q
    matrix_multiply_2x2(F, P, temp_2x2);
    matrix_multiply_2x2_transpose(temp_2x2, F, P);
    matrix_add_2x2(P, Q, P);

    // Update step
    // Innovation covariance: S = H * P * H^T + R
    S_pred = H[0] * P[0][0] * H[0] + H[1] * P[1][0] * H[0] +
             H[0] * P[0][1] * H[1] + H[1] * P[1][1] * H[1] + R;

    // Kalman gain: K = P * H^T / S
    K[0] = (P[0][0] * H[0] + P[0][1] * H[1]) / S_pred;
    K[1] = (P[1][0] * H[0] + P[1][1] * H[1]) / S_pred;

    // Innovation: y = z - H * S
    innovation = AltitudeLidar - (H[0] * S[0] + H[1] * S[1]);

    // State update: S = S + K * innovation
    S[0] += K[0] * innovation;
    S[1] += K[1] * innovation;

    // Covariance update: P = (I - K * H) * P
    temp_2x2[0][0] = I[0][0] - K[0] * H[0];
    temp_2x2[0][1] = I[0][1] - K[0] * H[1];
    temp_2x2[1][0] = I[1][0] - K[1] * H[0];
    temp_2x2[1][1] = I[1][1] - K[1] * H[1];

    matrix_multiply_2x2(temp_2x2, P, P);

    // Extract results
    AltitudeKalman = S[0];
    VelocityVerticalKalman = S[1];
}

void reset_pid(void)
{
    PrevErrorRateRoll = 0;
    PrevErrorRatePitch = 0;
    PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0;
    PrevItermRatePitch = 0;
    PrevItermRateYaw = 0;

    // reset PID error and integral values for the outer PID loop as well
    PrevErrorAngleRoll = 0;
    PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = 0;
    PrevItermAnglePitch = 0;

    PrevErrorVelocityVertical = 0;
    PrevItermVelocityVertical = 0;
}

// Initialize I2C
i2c_inst_t *i2c = init_i2c1(I2C_SDA1, I2C_SCL1, I2C_BAUD, true);

// Init BMI088
Bmi088 bmi088(i2c, BMI088_ACCL_ADDR, BMI088_GYRO_ADDR);

void bmi_signals()
{

    bmi088.readSensor();

    // Read accelerometer data    // 1G = 9.807 m/s^2
    AccX = -bmi088.getAccelX_G(); // X forward seeing the drone from the back
    AccY = bmi088.getAccelY_G();  // Y left
    AccZ = bmi088.getAccelZ_G();  // Z up

    // Read gyroscope data
    RateRoll = -bmi088.getGyroX_degs() - RateCalibrationRoll;  // X forward
    RatePitch = bmi088.getGyroY_degs() - RateCalibrationPitch; // Y left
    RateYaw = bmi088.getGyroZ_degs() - RateCalibrationYaw;     // Z up

    AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * RadtoDeg;   // to degrees
    AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RadtoDeg; // to degrees
}

bool newAlt = false; // Flag to indicate new altitude data

void lidar_signals(void)
{
    // Read the LIDAR sensor (VL53L8CX)
    uint8_t NewDataReady = 0;
    status = vl53l8cx_check_data_ready(&Dev, &NewDataReady);
    if (NewDataReady)
    {
        status = vl53l8cx_get_ranging_data(&Dev, &Results);
        if (status == 0)
        {
            if (Results.target_status[5] == 5 || Results.target_status[5] == 9)
            {
                // Use the 6th measurement (index 5) as the center altitude
                AltitudeLidar = (Results.distance_mm[5] - AltitudeLidarStartUp) / 10; // Convert mm to cm
                newAlt = true;
            }
        }
    }
}

// Initialize Kalman filter matrices properly
void init_altitude_kalman()
{
    // Initialize state vector S [altitude, velocity]
    S[0] = 0.0f; // Initial altitude
    S[1] = 0.0f; // Initial velocity

    // Initialize state covariance matrix P
    P[0][0] = 100.0f; // Initial altitude uncertainty (cm)
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 100.0f; // Initial velocity uncertainty (cm/s²)

    // Calculate process noise covariance Q = G * G^T * σ²
    // Assuming process noise standard deviation of 30 cm/s² for acceleration
    float process_noise_std = 1000.0f; // cm/s²
    float process_noise_var = process_noise_std * process_noise_std;

    Q[0][0] = G[0] * G[0] * process_noise_var; // (0.000008)² * 30²
    Q[0][1] = G[0] * G[1] * process_noise_var; // 0.000008 * dt_kalman * 30²
    Q[1][0] = G[1] * G[0] * process_noise_var; // dt_kalman * 0.000008 * 30²
    Q[1][1] = G[1] * G[1] * process_noise_var; // (dt_kalman)² * 30²

    printf("Kalman filter initialized\n");
    printf("Q matrix: [%.6f, %.6f; %.6f, %.6f]\n", Q[0][0], Q[0][1], Q[1][0], Q[1][1]);
}

void setup()
{

    stdio_init_all();
    sleep_ms(2000); // Allow time for USB enumeration
    printf("\nDual-Core Flight Controller starting...\n");

    // Initialize critical section for thread-safe RC data access
    critical_section_init(&rc_data_cs);

    // Initialize GPIO for LEDs
    gpio_init(LED_GREEN_L);
    gpio_set_dir(LED_GREEN_L, GPIO_OUT);
    gpio_init(LED_GREEN_R);
    gpio_set_dir(LED_GREEN_R, GPIO_OUT);
    gpio_init(LED_RED_L);
    gpio_set_dir(LED_RED_L, GPIO_OUT);
    gpio_init(LED_RED_R);
    gpio_set_dir(LED_RED_R, GPIO_OUT);
    gpio_put(LED_GREEN_L, 1);
    gpio_put(LED_GREEN_R, 1);
    gpio_put(LED_RED_L, 0);
    gpio_put(LED_RED_R, 0);
    sleep_ms(1000);

    // BMI initialization
    int status = bmi088.begin();
    if (status < 0)
    {
        printf("Error: %d\n", status);
    }
    bmi088.setOdr(Bmi088::ODR_400HZ); // Set ODR to 400Hz
    bmi088.setRange(Bmi088::ACCEL_RANGE_24G, Bmi088::GYRO_RANGE_2000DPS);

    init_lidar(); // Initialize LIDAR sensor

    // Initialize motors
    setup_motors();
    motor_test();

    printf("Starting calibration...\n");
    float accz_sum = 0.0f;
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
    {
        bmi088.readSensor();
        RateCalibrationRoll += -bmi088.getGyroX_degs();
        RateCalibrationPitch += +bmi088.getGyroY_degs();
        RateCalibrationYaw += +bmi088.getGyroZ_degs();
        AccX += -bmi088.getAccelX_G();
        AccY += +bmi088.getAccelY_G();
        AccZ += +bmi088.getAccelZ_G();
        // Calculate inertial Z acceleration
        float accz_inertial = -sin(AnglePitch * M_PI / 180.0f) * AccX +
                              cos(AnglePitch * M_PI / 180.0f) * sin(AngleRoll * M_PI / 180.0f) * AccY +
                              cos(AngleRoll * M_PI / 180.0f) * cos(AnglePitch * M_PI / 180.0f) * AccZ;

        accz_sum += accz_inertial;
        sleep_ms(1);
    }
    RateCalibrationRoll = RateCalibrationRoll / RateCalibrationNumber;
    RateCalibrationPitch = RateCalibrationPitch / RateCalibrationNumber;
    RateCalibrationYaw = RateCalibrationYaw / RateCalibrationNumber;
    AccZBias = accz_sum / RateCalibrationNumber + 1; // Average inertial Z acceleration in G(remove 1G)

    bool got_first_lidar = false;
    while (!got_first_lidar)
    {
        lidar_signals();
        if (newAlt)
        {
            AltitudeLidarStartUp = Results.distance_mm[5]; // Store raw reading as offset
            got_first_lidar = true;
            printf("LIDAR offset set to: %.1f mm\n", AltitudeLidarStartUp);
        }
        newAlt = false; // Reset flag
        sleep_ms(5);
    }

    printf("Calibration complete\n");

    gpio_put(LED_GREEN_L, 0);
    gpio_put(LED_GREEN_R, 0);
    gpio_put(LED_RED_L, 1);
    gpio_put(LED_RED_R, 1);
    sleep_ms(1000);

    init_altitude_kalman(); // Initialize Kalman filter for altitude

    // RP_AGROLIB_BC832_Simple ble(UART_ID, UART_TX, UART_RX, BLE_MODE_PIN, 115200);

    printf("Initializing BC832 module...\n");
    if (!ble_instance->begin())
    {
        printf("Failed to initialize BC832 module!\n");
        while (1)
        {
            tight_loop_contents();
        }
    }
    printf("BC832 module initialized successfully\n");

    // Reset module
    printf("Resetting module...\n");
    ble_instance->reset();
    sleep_ms(1000);

    // Setup peripheral device
    printf("Setting up peripheral device...\n");

    std::string macAddress = ble_instance->getAddress();
    printf("Peripheral device MAC address: %s\n", macAddress.c_str());

    ble_instance->setRFPower(0);

    printf("Setting up advertising...\n");
    if (ble_instance->setupPeripheral("BLE_PERIPHERAL"))
    {
        printf("Peripheral setup successful\n");
    }
    else
    {
        printf("Peripheral setup failed\n");
    }

    ble_instance->saveConfig();
    printf("Peripheral device setup complete\n");

    // Initialize UART for Core 1
    // int irq = uart_setup(UART_ID, UART_RX, UART_TX, 115200, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    // Start Core 1 with UART reader
    printf("Starting Core 1 for UART processing...\n");
    multicore_launch_core1(uart_reader_core1);
    // Wait for Core 1 to start
    while (!core1_running)
    {
        sleep_ms(10);
    }
    printf("Core 1 started successfully\n");

    sleep_ms(1000); // Allow time for setup

    // Wait for valid throttle range before arming
    while (ReceiverValue[2] > 1200 || ReceiverValue[2] <= 1000)
    {
        if (ble_instance->isConnected())
        {
            read_receiver();
            printf("Throttle %f\n", ReceiverValue[2]);
            sleep_ms(200);
        }
        else
        {
            printf("Waiting for RC connection...\n");
            sleep_ms(500);
        }
    }
    printf("Armed\n");
    armed = true;

    // Last line of setup - time variable for control loop
    LoopTimer = time_us_32();
}

void loop()
{

    bmi_signals();

    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    AccZInertial = -sin(AnglePitch * M_PI / 180.0f) * AccX +
                   cos(AnglePitch * M_PI / 180.0f) * sin(AngleRoll * M_PI / 180.0f) * AccY +
                   cos(AngleRoll * M_PI / 180.0f) * cos(AnglePitch * M_PI / 180.0f) * AccZ;

    AccZInertial = (AccZInertial + 1) * 9.81 * 100; // Convert to cm/s^2

    lidar_signals();

    if (newAlt)
    {
        static uint32_t last_kalman_time = 0;
        uint32_t now = time_us_32();
        dt_kalman = (now - last_kalman_time) / 1e6f; // Convert to seconds
        last_kalman_time = now;
        kalman_2d(); // Run the 2D Kalman filter for altitude and velocity
    }

    // Read receiver values
    read_receiver();

    // Calculate desired angles from receiver inputs
    DesiredAngleRoll = 0.04 * (ReceiverValue[0] - 1500); // limit to 20 degrees
    DesiredAnglePitch = 0.04 * (ReceiverValue[1] - 1500);
    DesiredRateYaw = 0.1 * (ReceiverValue[3] - 1500); // limit to 50 degrees/s

    DesiredVelocityVertical = 0.1 * (ReceiverValue[2] - 1500); // limit to -60/60 cm/s worked descent

    if (newAlt)
    {
        // Calculate the error in vertical velocity
        ErrorVelocityVertical = DesiredVelocityVertical - VelocityVerticalKalman;
        pid_equation(ErrorVelocityVertical, PVelocityVertical, IVelocityVertical, DVelocityVertical, PrevErrorVelocityVertical, PrevItermVelocityVertical, dt_kalman);
        InputThrottle = 500 + PIDReturn[0]; // Base throttle at 500us, adjust with PID output
        PrevErrorVelocityVertical = PIDReturn[1];
        PrevItermVelocityVertical = PIDReturn[2];
        newAlt = false; // Reset flag after processing
    }

    // Calculate difference between desired and actual angles
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll, 0.004);
    DesiredRateRoll = PIDReturn[0];
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];

    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch, 0.004);
    DesiredRatePitch = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];

    // Calculate the difference between the desired and the actual roll, pitch and yaw rotation rates. Use these for the PID controller of the inner loop
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;

    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll, 0.004);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];
    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch, 0.004);
    InputPitch = PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];
    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw, 0.004);
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[1];
    PrevItermRateYaw = PIDReturn[2];

    if (InputThrottle > 800)
        InputThrottle = 800; // limit throttle to 80% to permit roll, pitch and yaw estabilization

    // Calculate motor inputs
    MotorInput1 = InputThrottle - InputRoll - InputPitch + InputYaw;
    MotorInput2 = InputThrottle - InputRoll + InputPitch - InputYaw;
    MotorInput3 = InputThrottle + InputRoll + InputPitch + InputYaw;
    MotorInput4 = InputThrottle + InputRoll - InputPitch - InputYaw;

    // Limit motor inputs to 0-1000 microseconds
    if (MotorInput1 > 1000)
        MotorInput1 = 999;
    if (MotorInput2 > 1000)
        MotorInput2 = 999;
    if (MotorInput3 > 1000)
        MotorInput3 = 999;
    if (MotorInput4 > 1000)
        MotorInput4 = 999;

    // Keep motors at 10% if throttle is low
    int ThrottleIdle = 100;
    if (MotorInput1 < ThrottleIdle)
        MotorInput1 = ThrottleIdle;
    if (MotorInput2 < ThrottleIdle)
        MotorInput2 = ThrottleIdle;
    if (MotorInput3 < ThrottleIdle)
        MotorInput3 = ThrottleIdle;
    if (MotorInput4 < ThrottleIdle)
        MotorInput4 = ThrottleIdle;

    // Make sure able to disarm motors
    int ThrottleCutOff = 0;
    static int counter_cutof = 0; // Counter for disarm condition
    if (ReceiverValue[2] < 1050)
    { // If throttle is below 10%
        counter_cutof++;
    }
    else
    {
        counter_cutof = 0; // Reset counter if throttle is above 10%
    }

    // arm/disarm logic
    if (ReceiverValue[2] > 1500 && !armed)
    {                 // If throttle is above 100% and not armed
        armed = true; // Set armed flag to true
        reset_pid();
        KalmanAngleRoll = AngleRoll;
        KalmanAnglePitch = AnglePitch;
        KalmanUncertaintyAngleRoll = 2 * 2;
        KalmanUncertaintyAnglePitch = 2 * 2;
        MotorInput1 = 50; // Set motors to idle speed
        MotorInput2 = 50;
        MotorInput3 = 50;
        MotorInput4 = 50;
        printf("Motors armed\n");
        gpio_put(LED_RED_L, 1);
        gpio_put(LED_RED_R, 1);
        gpio_put(LED_GREEN_L, 0);
        gpio_put(LED_GREEN_R, 0);
    }

    if (ReceiverValue[2] < 1050 && counter_cutof > 250 & AltitudeKalman < 5)
    { // If throttle is below 10% and pitch is below 10% (disarm condition)

        armed = false; // Set armed flag to false
        printf("Motors disarmed\n");
        gpio_put(LED_RED_L, 0);
        gpio_put(LED_RED_R, 0);
        gpio_put(LED_GREEN_L, 1);
        gpio_put(LED_GREEN_R, 1);
        counter_cutof = 0; // Reset disarm counter
    }

    // send commands to motors
    if (armed)
    {
        set_motor_speed(MOTOR1_PIN, MotorInput1);
        set_motor_speed(MOTOR2_PIN, MotorInput2);
        set_motor_speed(MOTOR3_PIN, MotorInput3);
        set_motor_speed(MOTOR4_PIN, MotorInput4);
    }
    else
    {
        set_motor_speed(MOTOR1_PIN, ThrottleCutOff);
        set_motor_speed(MOTOR2_PIN, ThrottleCutOff);
        set_motor_speed(MOTOR3_PIN, ThrottleCutOff);
        set_motor_speed(MOTOR4_PIN, ThrottleCutOff);
    }

    // Debug output (reduce frequency to avoid spam)
    // static int debug_counter = 0;
    // if(debug_counter++ > 100) {
    //    printf("                               Angles:   %.1f   %.1f\n", AngleRoll, AnglePitch);
    //    printf("ACCL: X=%.3f, Y=%.3f, Z=%.3f | Kalman: R=%.1f P=%.1f | Rates: R=%.1f P=%.1f Y=%.1f \n",
    //           AccX, AccY, AccZ, KalmanAngleRoll, KalmanAnglePitch, RateRoll, RatePitch, RateYaw);
    //    printf("Motors: %.1f  %.1f  %.1f  %.1f \n", MotorInput1, MotorInput2, MotorInput3, MotorInput4);
    //    printf("Receiver: R=%.1f P=%.1f T=%.1f Y=%.1f\n",
    //           ReceiverValue[0], ReceiverValue[1], ReceiverValue[2], ReceiverValue[3]);
    //    printf("AccZInertial: %.1f | AltitudeLidar: %.1f | AltitudeKalman: %.1f | VelocityVerticalKalman: %.1f\n",
    //           AccZInertial, AltitudeLidar, AltitudeKalman, VelocityVerticalKalman);
    //    printf("Kalman State: Alt=%.1f, Vel=%.1f | Innovation=%.1f\n",
    //           S[0], S[1], innovation);
    //    printf("dt_kalman: %.6f \n", dt_kalman);
    //    debug_counter = 0;
    //}

    update_telemetry_data(KalmanAngleRoll, KalmanAnglePitch, RateRoll, RatePitch, RateYaw, ErrorAngleRoll, ErrorAnglePitch, ErrorRateRoll, ErrorRatePitch, ErrorRateYaw, MotorInput1, MotorInput2, MotorInput3, MotorInput4, AltitudeKalman, VelocityVerticalKalman, AltitudeLidar, armed);

    // printf("Roll_angle: %f, Pitch_angle: %f\n", KalmanAngleRoll, KalmanAnglePitch);
    uint32_t current_time = time_us_32();
    while (current_time - LoopTimer < 4000) // 250Hz
    {
        // printf("Waiting for next loop...\n");
        current_time = time_us_32();
        tight_loop_contents();
    }
    LoopTimer = time_us_32();
}

int main()
{

    // Create BLE instance
    RP_AGROLIB_BC832_Simple ble(UART_ID, UART_TX, UART_RX, BLE_MODE_PIN, 115200);
    ble_instance = &ble;

    setup();

    printf("Core 0: Starting main flight control loop\n");
    while (1)
    {
        loop();
    }

    return 0;
}