#include "pico/stdlib.h"
#include <time.h>
#include <cmath>
#include <cstring>
#include <cstdlib>   // For strtof

#include "rp_agrolib_motors.h"
//M1 = 24, M2 = 14, M3 = 10, M4 = 2

#include "rp_agrolib_uart.h"
// UART configuration for Bluetooth module
#define UART_ID uart0
#define UART_RX 17
#define UART_TX 16
#define UART_BAUDRATE 115200

// Buffer settings
#define MAX_BUFFER_LEN 128
#define TIMEOUT_US 50000  // 50ms timeout

// Expected message format from smartphone
// Format: "RC,roll,pitch,throttle,yaw\n"
// Example: "RC,1500,1500,1000,1500\n"

/*   receiver/controller   */
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber=0;

typedef struct {
    float roll;
    float pitch; 
    float throttle;
    float yaw;
    bool data_valid;
    uint64_t last_update;
} rc_data_t;

static volatile rc_data_t rc_data = {1500.0f, 1500.0f, 0.0f, 1500.0f, false, 0};

// Improved parsing function with better error handling
bool parse_rc_command(const char* cmd) {
    // Remove any leading whitespace
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    
    // Handle different prefixes
    if (strncmp(cmd, "+B", 2) == 0) {
        cmd += 2;
        // Skip any special character after +B
        if (*cmd != 'R') cmd++;
    }
    
    // Check if the command starts with "RC,"
    if (strncmp(cmd, "RC,", 3) != 0) {
        return false;
    }
    
    //printf("Parsing RC command: %s\n", cmd);
    const char* ptr = cmd + 3; // Skip "RC,"
    
    // Temporary variables to validate all values before updating
    float temp_values[4];
    
    // Parse all 4 values
    for (int i = 0; i < 4; i++) {
        char* endptr;
        temp_values[i] = strtof(ptr, &endptr);
        
        // Check if parsing was successful
        if (ptr == endptr) {
            //printf("Parse error at value %d\n", i);
            return false;
        }
        
        // Validate range (typical RC values are 1000-2000)
        if (temp_values[i] < 0 || temp_values[i] > 2500) {
            printf("Value %d out of range: %.1f\n", i, temp_values[i]);
            return false;
        }
        
        // Move to next value (skip comma)
        if (i < 3) {
            ptr = strchr(endptr, ',');
            if (!ptr) {
                //printf("Missing comma after value %d\n", i);
                return false;
            }
            ptr++; // Skip comma
        }
    }
    
    // All values are valid, update atomically    
    rc_data.roll = temp_values[0];
    rc_data.pitch = temp_values[1];
    rc_data.throttle = temp_values[2];
    rc_data.yaw = temp_values[3];
    rc_data.data_valid = true;
    rc_data.last_update = time_us_64();

    return true;
}

// Improved UART reading with better buffer management
void read_uart_data() {
    static char buffer[MAX_BUFFER_LEN];
    static int buffer_index = 0;
    static uint64_t last_char_time = 0;
    
    uint64_t current_time = time_us_64();
    
    // Check for timeout on incomplete message
    if (buffer_index > 0 && (current_time - last_char_time) > TIMEOUT_US) {
        //printf("UART timeout, clearing buffer: %.*s\n", buffer_index, buffer);
        buffer_index = 0;
    }
    
    // Read all available characters
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        last_char_time = current_time;
        
        // Skip null characters and other control chars except CR/LF
        if (c == 0 || (c < 32 && c != '\r' && c != '\n')) {
            continue;
        }
        
        // Check for end of message
        if (c == '\n' || c == '\r') {
            if (buffer_index > 0) {
                buffer[buffer_index] = '\0';  // Null-terminate
                
                // Process different message types
                if (strncmp(buffer, "+C", 2) == 0) {
                    printf("Connected to RC\n");
                } else if (strncmp(buffer, "+D", 2) == 0) {
                    printf("Disconnected from RC\n");
                } else if (strstr(buffer, "RC,") != NULL) {
                    // Handle RC command (with or without +B prefix)
                    parse_rc_command(buffer);
                } else {
                    //printf("Unknown command: %s\n", buffer);
                }
                
                // Reset buffer for next message
                buffer_index = 0;
            }
        } 
        // Add character to buffer if there's room
        else if (buffer_index < MAX_BUFFER_LEN - 1) {
            buffer[buffer_index++] = c;
        } else {
            // Buffer overflow - reset and log
            //printf("UART buffer overflow, resetting\n");
            buffer_index = 0;
        }
    }
}

// Thread-safe function to get RC values
void get_rc_values(float* roll, float* pitch, float* throttle, float* yaw, bool* valid) {
    
    *roll = rc_data.roll;
    *pitch = rc_data.pitch;
    *throttle = rc_data.throttle;
    *yaw = rc_data.yaw;
    *valid = rc_data.data_valid;
    
    // Check if data is stale (older than 500ms)
    uint64_t current_time = time_us_64();
    if (rc_data.data_valid && (current_time - rc_data.last_update) > 500000) {
        *valid = false;
        printf("RC data stale, disabling\n");
    }
}

// Function to read receiver values
void read_receiver(void) {
    // Read latest UART data first
    read_uart_data();
    
    // Get RC values safely
    bool valid;
    float temp_roll, temp_pitch, temp_throttle, temp_yaw;
    get_rc_values(&temp_roll, &temp_pitch, &temp_throttle, &temp_yaw, &valid);
    
    if (valid) {
        // Apply deadband to reduce oscillation around center values
        const float DEADBAND = 10.0f; // ±10 units around center
        const float DEADBAND_YAW = 200.0f; // ±10 units for yaw
        
        // Center values (adjust based on your controller)
        const float CENTER_ROLL = 1500.0f;
        const float CENTER_PITCH = 1500.0f;
        const float CENTER_YAW = 1500.0f;
        
        // Apply deadband
        if (fabs(temp_roll - CENTER_ROLL) < DEADBAND) {
            temp_roll = CENTER_ROLL;
        }
        if (fabs(temp_pitch - CENTER_PITCH) < DEADBAND) {
            temp_pitch = CENTER_PITCH;
        }
        if (fabs(temp_yaw - CENTER_YAW) < DEADBAND_YAW) {
            temp_yaw = CENTER_YAW;
        }
        
        // Update receiver values
        ReceiverValue[0] = temp_roll;
        ReceiverValue[1] = temp_pitch;
        ReceiverValue[2] = temp_throttle;
        ReceiverValue[3] = temp_yaw;
    } else {
        // Use safe default values if no valid data
        ReceiverValue[0] = 1500.0f; // Roll center
        ReceiverValue[1] = 1500.0f; // Pitch center
        ReceiverValue[2] = 0.0f;    // Throttle off
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
float RateCalibrationPitch=0, RateCalibrationRoll=0, RateCalibrationYaw=0;
float AcclCalibrationX=0, AcclCalibrationY=0, AcclCalibrationZ=0;
int RateCalibrationNumber=0;
float RadtoDeg=180/3.14159265358979323846;

uint32_t LoopTimer; //lengh of each control loop

// Define predicted angles and uncertainties
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;

// Initialize output of filter
float Kalman1DOutput[]={0/*angle prediction*/, 0/*uncertainty of the prediction*/};                       

// Define the desired roll and pitch angles and corresponding errors for the outer loop PID controller                       
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;

//Define the values necessary for the outer loop PID controller, including the P, I and D parameters
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll=10; float PAnglePitch=10;
float IAngleRoll=0; float IAnglePitch=0;
float DAngleRoll=0; float DAnglePitch=0;

// Create the function that calculates the predicted angle and uncertainty using Kalman equations
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState = KalmanState+0.005*KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.005*0.005 * 4*4; //std desviation of accl 4º
    float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3*3); //std desviation of gyro 3º
    KalmanState = KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
    KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;

    Kalman1DOutput[0]=KalmanState;        //Kalman filter output
    Kalman1DOutput[1]=KalmanUncertainty;
}
//KalmanInput is the rotation rate from the gyro
//KalmanMeasurement is the angle from the accelerometer
//KalmanState is the angle calculated by the Kalman filter

/*   PID   */
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};
float PRateRoll=9 ; float PRatePitch=9; float PRateYaw=30;  //0.6PRoll
float IRateRoll=6 ; float IRatePitch=6; float IRateYaw=2;
float DRateRoll=0.2 ; float DRatePitch=0.2; float DRateYaw=0.01; //0Dyaw
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

/*   PID Equation   */
void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.005/2;  //0.005 is the time step -> 200Hz
  if (Iterm > 100) Iterm=100;
  else if (Iterm <-100) Iterm=-100;
  float Dterm=D*(Error-PrevError)/0.005;
  float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>100) PIDOutput=100;
  else if (PIDOutput <-100) PIDOutput=-100;

  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(void) {
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;

    //reset PID error and integral values for the outer PID loop as well
    PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;
    PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}

// Initialize I2C
i2c_inst_t *i2c = init_i2c1(I2C_SDA1, I2C_SCL1, I2C_BAUD, true);

// Init BMI088
Bmi088 bmi088(i2c, BMI088_ACCL_ADDR, BMI088_GYRO_ADDR);

void setup(){

    stdio_init_all();
    sleep_ms(2000); // Allow time for USB enumeration
    printf("\nFlight Controller starting...\n");

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

    /*BMI begin*/
    int status = bmi088.begin();
    if (status < 0)
    {
        printf("Error: %d\n", status);
    }
    bmi088.setOdr(Bmi088::ODR_400HZ); // Set ODR to 400Hz
    bmi088.setRange(Bmi088::ACCEL_RANGE_24G, Bmi088::GYRO_RANGE_2000DPS); // Set accelerometer to 12G and gyro to 2000DPS

    // Initialize motors
    setup_motors();
    motor_test();

    for(RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++){
        bmi088.readSensor();
        RateCalibrationRoll+=  -(bmi088.getGyroX_rads() * RadtoDeg);
        RateCalibrationPitch+= +(bmi088.getGyroY_rads() * RadtoDeg);
        RateCalibrationYaw+=   -(bmi088.getGyroZ_rads() * RadtoDeg);
        sleep_ms(1);
    }
    RateCalibrationRoll=RateCalibrationRoll/RateCalibrationNumber;
    RateCalibrationPitch=RateCalibrationPitch/RateCalibrationNumber;
    RateCalibrationYaw=RateCalibrationYaw/RateCalibrationNumber;

    gpio_put(LED_GREEN_L, 0);
    gpio_put(LED_GREEN_R, 0);
    gpio_put(LED_RED_L, 1);
    gpio_put(LED_RED_R, 1);
    sleep_ms(1000);

    // Avoid accidental liftoff
    //throttle = 0; roll = 0; pitch = 0; yaw = 0;
    rc_data.roll = 1500.0f;
    rc_data.pitch = 1500.0f;
    rc_data.throttle = 0.0f;
    rc_data.yaw = 1500.0f;
    rc_data.data_valid = false;
    rc_data.last_update = 0;

    while (ReceiverValue[2] < 10 || ReceiverValue[2] > 60) {
      read_receiver();
      printf("Throttle %f\n", ReceiverValue[2]);
      sleep_ms(200);
    }
    printf("Armed\n");
    sleep_ms(1000);

    //Last line of setup - time variable for control loop
    LoopTimer = time_us_32();
}

void bmi_signals(){

    bmi088.readSensor();

    // Read accelerometer data   //1G = 9.81m/s^2
    AccX = -bmi088.getAccelX_mss() / 9.81; // X forward seeing the drone from the back
    AccY =  bmi088.getAccelY_mss() / 9.81; // Y left
    AccZ = -bmi088.getAccelZ_mss() / 9.81; // Z up

    // Read gyroscope data
    RateRoll  = -bmi088.getGyroX_rads() * RadtoDeg - RateCalibrationRoll;   // X forward
    RatePitch =  bmi088.getGyroY_rads() * RadtoDeg + RateCalibrationPitch;  // Y left
    RateYaw   = -bmi088.getGyroZ_rads() * RadtoDeg - RateCalibrationYaw;    // Z up

    AngleRoll  = -atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * RadtoDeg; //to degrees
    AnglePitch = atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RadtoDeg - 1.5; //to degrees // -1.5 faz andar para trás para compensar bateria
}

void loop() {

    bmi_signals();

    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll=Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll=Kalman1DOutput[1];

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch=Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

    // Read receiver values
    read_receiver();

    //Calculate desired angles from receiver inputs
    DesiredAngleRoll  = 0.02 * (ReceiverValue[0] - 1500); //limit to 10 degrees
    DesiredAnglePitch = -0.02 * (ReceiverValue[1] - 1500);

    InputThrottle = ReceiverValue[2];
    DesiredRateYaw = 0.1 * (ReceiverValue[3] - 1500); //limit to 5 degrees/s

    // Calculate difference between desired and actual angles
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
    DesiredRateRoll = PIDReturn[0];
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];

    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];

    //Calculate the difference between the desired and the actual roll, pitch and yaw rotation rates. Use these for the PID controller of the inner loop
    ErrorRateRoll=DesiredRateRoll-RateRoll;
    ErrorRatePitch=DesiredRatePitch-RatePitch;
    ErrorRateYaw=DesiredRateYaw-RateYaw;

    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
       InputRoll=PIDReturn[0];
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];
    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
       InputPitch=PIDReturn[0]; 
       PrevErrorRatePitch=PIDReturn[1]; 
       PrevItermRatePitch=PIDReturn[2];
    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
       InputYaw=PIDReturn[0]; 
       PrevErrorRateYaw=PIDReturn[1]; 
       PrevItermRateYaw=PIDReturn[2];

    if (InputThrottle > 600) InputThrottle = 600; //limit throttle to 80% to permit roll, pitch and yaw estabilization

    // Calculate motor inputs
    MotorInput1 = InputThrottle-InputRoll+InputPitch+InputYaw;
    MotorInput2 = InputThrottle-InputRoll-InputPitch-InputYaw;
    MotorInput3 = InputThrottle+InputRoll-InputPitch+InputYaw;
    MotorInput4 = InputThrottle+InputRoll+InputPitch-InputYaw;

    //Limit motor inputs to 0-1000 microseconds
    if (MotorInput1 > 1000)MotorInput1 = 999;
    if (MotorInput2 > 1000)MotorInput2 = 999; 
    if (MotorInput3 > 1000)MotorInput3 = 999; 
    if (MotorInput4 > 1000)MotorInput4 = 999;

    //Keep motors at 10% if throttle is low
    int ThrottleIdle=30;
    if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;
    
    //Make sure able to disarm motors
    int ThrottleCutOff=0;
    if (ReceiverValue[2]<10) {
        MotorInput1=ThrottleCutOff; 
        MotorInput2=ThrottleCutOff;
        MotorInput3=ThrottleCutOff; 
        MotorInput4=ThrottleCutOff;
        reset_pid();

        KalmanAngleRoll = AngleRoll;
        KalmanAnglePitch = AnglePitch;
        KalmanUncertaintyAngleRoll = 2*2;
        KalmanUncertaintyAnglePitch = 2*2;
    }

    //send commands to motors
    set_motor_speed(MOTOR1_PIN, MotorInput1);
    set_motor_speed(MOTOR2_PIN, MotorInput2);
    set_motor_speed(MOTOR3_PIN, MotorInput3);
    set_motor_speed(MOTOR4_PIN, MotorInput4);


    // Debug output (reduce frequency to avoid spam)
    static int debug_counter = 0;
    if(debug_counter++ > 100) {
        printf("                                         %.1f   %.1f\n", AngleRoll, AnglePitch);
        printf("ACCL: X=%.1f, Y=%.1f, Z=%.1f | Angles: R=%.1f P=%.1f | Rates: R=%.1f P=%.1f Y=%.1f \n", 
               AccX, AccY, AccZ, AngleRoll, AnglePitch, KalmanAngleRoll, KalmanAnglePitch, RateRoll, RatePitch, RateYaw);
        printf("Motors: %.1f  %.1f  %.1f  %.1f \n", MotorInput1, MotorInput2, MotorInput3, MotorInput4);
        printf("Receiver: R=%.1f P=%.1f T=%.1f Y=%.1f\n", 
               ReceiverValue[0], ReceiverValue[1], ReceiverValue[2], ReceiverValue[3]);
        
        debug_counter = 0;
    }
    
    
    //printf("Roll_angle: %f, Pitch_angle: %f\n", KalmanAngleRoll, KalmanAnglePitch);
    uint32_t current_time = time_us_32();
    while (current_time - LoopTimer < 5000) //200Hz
    {
        //printf("Waiting for next loop...\n");
        current_time = time_us_32();
    }
    LoopTimer = time_us_32();
}

int main() {

    int irq = uart_setup(UART_ID, UART_RX, UART_TX, 115200, 8, 1, UART_PARITY_NONE);
    //uart_enable_interrupt(UART_ID, irq, on_uart_rx);
    //last_rx_time = time_us_64();
    uart_set_fifo_enabled(UART_ID, true); // Enable FIFO for better performance

    setup();
    while (1) {
        loop();
        sleep_ms(1); // Avoid 100% CPU usage
    }
}