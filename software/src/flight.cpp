#include "pico/stdlib.h"
#include <time.h>
#include <cstring>
#include <cstdlib>   // For strtof

#include "rp_agrolib_motors.h"
#define MOTOR1_PIN 24
#define MOTOR2_PIN 14
#define MOTOR3_PIN 10
#define MOTOR4_PIN 2

//M1 = 24, M2 = 14, M3 = 10, M4 = 2

void setup_motors() {
    // Initialize PWM for each motor
    // For each pin, we need to:
    // 1. Set the GPIO function to PWM
    // 2. Find the PWM slice and channel for each GPIO
    // 3. Set the clock divider to achieve desired frequency
    // 4. Set the wrap (resolution)
    // 5. Enable the PWM slice

    // Motor 1
    gpio_set_function(MOTOR1_PIN, GPIO_FUNC_PWM);
    uint slice_num1 = pwm_gpio_to_slice_num(MOTOR1_PIN);
    uint channel1 = pwm_gpio_to_channel(MOTOR1_PIN);
    
    // Motor 2
    gpio_set_function(MOTOR2_PIN, GPIO_FUNC_PWM);
    uint slice_num2 = pwm_gpio_to_slice_num(MOTOR2_PIN);
    uint channel2 = pwm_gpio_to_channel(MOTOR2_PIN);
    
    // Motor 3
    gpio_set_function(MOTOR3_PIN, GPIO_FUNC_PWM);
    uint slice_num3 = pwm_gpio_to_slice_num(MOTOR3_PIN);
    uint channel3 = pwm_gpio_to_channel(MOTOR3_PIN);
    
    // Motor 4
    gpio_set_function(MOTOR4_PIN, GPIO_FUNC_PWM);
    uint slice_num4 = pwm_gpio_to_slice_num(MOTOR4_PIN);
    uint channel4 = pwm_gpio_to_channel(MOTOR4_PIN);

    // Calculate frequency parameters
    // The Pico's system clock runs at 125 MHz by default
    // To get 250 Hz with 2000 steps (0-1999) resolution:
    // Clock divider = 125,000,000 / (250 * 2000) = 250
    float clock_div = 250.0f;
    uint16_t wrap = 1000 - 1; // Range from 0 to 999

    // Configure PWM slices
    // Note: If motors share the same slice, we only need to configure once
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_div);
    pwm_config_set_wrap(&config, wrap);

    // Initialize each PWM slice with the configuration
    pwm_init(slice_num1, &config, true);
    
    // Only initialize other slices if they're different from slice_num1
    if (slice_num2 != slice_num1) pwm_init(slice_num2, &config, true);
    if (slice_num3 != slice_num1 && slice_num3 != slice_num2) pwm_init(slice_num3, &config, true);
    if (slice_num4 != slice_num1 && slice_num4 != slice_num2 && slice_num4 != slice_num3) 
        pwm_init(slice_num4, &config, true);
    
    // Set initial PWM level to 0 for all motors
    pwm_set_gpio_level(MOTOR1_PIN, 0);
    pwm_set_gpio_level(MOTOR2_PIN, 0);
    pwm_set_gpio_level(MOTOR3_PIN, 0);
    pwm_set_gpio_level(MOTOR4_PIN, 0);
}

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
// Format: "RC,roll,pitch,throttle,yawn"
// Example: "RC,1500,1500,1000,1500\n"

// Buffer for incoming data
char buffer[MAX_BUFFER_LEN];
volatile int buffer_index = 0;
//volatile uint64_t last_rx_time;

// RC Values - these will be updated by the UART interrupt handler
//volatile float roll = 0.0f;
//volatile float pitch = 0.0f;
//volatile float throttle = 0.0f;
//volatile float yaw = 0.0f;
//volatile bool new_data_available = false;
float roll = 0.0f;
float pitch = 0.0f;
float throttle = 0.0f;
float yaw = 0.0f;

// Function to parse RC commands from buffer
//void parse_rc_command(const char* cmd) {
//    cmd = cmd + 3; // Skip "+B + special char"
//    // Check if the command starts with "RC,"
//    printf("Command after +B: %s\n", cmd);
//
//    if (strncmp(cmd, "RC,", 3) == 0) {
//        printf("Parsing RC command: %s\n", cmd);
//        const char* ptr = cmd + 3; // Skip "RC,"
//        
//        // Parse roll
//        float new_roll = strtof(ptr, NULL);
//        
//        // Find next comma
//        ptr = strchr(ptr, ',');
//        if (!ptr) return;
//        ptr++; // Skip comma
//        
//        // Parse pitch
//        float new_pitch = strtof(ptr, NULL);
//        
//        // Find next comma
//        ptr = strchr(ptr, ',');
//        if (!ptr) return;
//        ptr++; // Skip comma
//
//        // Parse throttle
//        float new_throttle = strtof(ptr, NULL);
//        
//        // Find next comma
//        ptr = strchr(ptr, ',');
//        if (!ptr) return;
//        ptr++; // Skip comma
//        
//        // Parse yaw
//        float new_yaw = strtof(ptr, NULL);
//        
//        // Update values automatically
//        roll = new_roll;
//        pitch = new_pitch;
//        throttle = new_throttle;
//        yaw = new_yaw;
//        new_data_available = true;
//        
//        printf("RC values updated: roll=%.1f, pitch=%.1f, throttle=%.1f, yaw=%.1f\n", roll, pitch, throttle, yaw);
//    }
//}

bool parse_rc_command(const char* cmd) {
    // Skip "+B" prefix if present
    if (strncmp(cmd, "+B", 2) == 0) {
        cmd = cmd + 3; // Skip "+B + special char"
    }
    
    // Check if the command starts with "RC,"
    if (strncmp(cmd, "RC,", 3) == 0) {
        printf("Parsing RC command: %s\n", cmd);
        const char* ptr = cmd + 3; // Skip "RC,"
        
        // Parse roll
        float new_roll = strtof(ptr, NULL);
        
        // Find next comma
        ptr = strchr(ptr, ',');
        if (!ptr) return false;
        ptr++; // Skip comma
        
        // Parse pitch
        float new_pitch = strtof(ptr, NULL);
        
        // Find next comma
        ptr = strchr(ptr, ',');
        if (!ptr) return false;
        ptr++; // Skip comma

        // Parse throttle
        float new_throttle = strtof(ptr, NULL);
        
        // Find next comma
        ptr = strchr(ptr, ',');
        if (!ptr) return false;
        ptr++; // Skip comma
        
        // Parse yaw
        float new_yaw = strtof(ptr, NULL);
        
        // Update values
        roll = new_roll;
        pitch = new_pitch;
        throttle = new_throttle;
        yaw = new_yaw;
        
        printf("RC values updated: roll=%.1f, pitch=%.1f, throttle=%.1f, yaw=%.1f\n", roll, pitch, throttle, yaw);
        return true;
    }
    return false;
}

// Polling-based UART reading function
void read_uart_data() {
    static char buffer[MAX_BUFFER_LEN];
    static int buffer_index = 0;
    
    // Read all available characters
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        
        // Check for end of message
        if (c == '\n' || c == '\r') {
            if (buffer_index > 0) {
                buffer[buffer_index] = '\0';  // Null-terminate the string
                
                // Process different message types
                if (strncmp(buffer, "+C", 2) == 0) {
                    printf("Connected to RC\n");
                } else if (strncmp(buffer, "+B", 2) == 0) {
                    parse_rc_command(buffer);
                } else if (strncmp(buffer, "+D", 2) == 0) {
                    printf("Disconnected from RC\n");
                } else if (strncmp(buffer, "RC,", 3) == 0) {
                    // Direct RC command without +B prefix
                    parse_rc_command(buffer);
                }
                
                // Reset buffer for next message
                buffer_index = 0;
            }
        } 
        // Add character to buffer if there's room
        else if (buffer_index < MAX_BUFFER_LEN - 1) {
            buffer[buffer_index++] = c;
        } else {
            // Buffer overflow - reset
            buffer_index = 0;
            printf("UART buffer overflow\n");
        }
    }
}

// Process complete message in buffer
//void process_buffer() {
//    if (buffer_index > 0) {
//        buffer[buffer_index] = '\0';  // Null-terminate the string
//
//        if(strncmp(buffer, "+C", 2) == 0)
//            printf("Connected to RC\n");
//        if(strncmp(buffer, "+B", 2) == 0)
//            parse_rc_command(buffer);
//        if(strncmp(buffer, "+D", 2) == 0)
//            printf("Disconnected from RC\n");
//        else
//            printf("Unknown command: %s\n", buffer);
//        
//        // Reset buffer
//        buffer_index = 0;
//    }
//}

//void on_uart_rx() {
//    while (uart_is_readable(UART_ID)) {
//        char c = uart_getc(UART_ID);
//        last_rx_time = time_us_64();  // Update last read time
//        
//        // Check for end of message
//        if (c == 'n' || c == '\r' || c == '\n') {
//            process_buffer();
//        } 
//        // Add character to buffer if there's room
//        else if (buffer_index < MAX_BUFFER_LEN - 1) {
//            buffer[buffer_index++] = c;
//            //printf("Buffer[%d]: %02X\n", buffer_index - 1, c);
//        }
//    }
//}

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
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float RadtoDeg=180/3.14159;

/*   receiver/controller   */
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber=0;

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
float PAngleRoll=2; float PAnglePitch=PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0; float DAnglePitch=DAngleRoll;

// Create the function that calculates the predicted angle and uncertainty using Kalman equations
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState = KalmanState+0.004*KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
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
float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

/*   Receiver inputs   */
//void read_receiver(void) {
//    //printf("reading receiver values\n");
//    if(new_data_available){
//        //update receiver values
//        ReceiverValue[0] = roll;
//        ReceiverValue[1] = pitch;
//        ReceiverValue[2] = throttle;
//        ReceiverValue[3] = yaw;
//        new_data_available = false;
//    }   
//}

void read_receiver(void) {
    // Read latest UART data first
    read_uart_data();
    
    // Update receiver values with latest joystick data
    ReceiverValue[0] = roll;
    ReceiverValue[1] = pitch;
    ReceiverValue[2] = throttle;
    ReceiverValue[3] = yaw;
}

// Check for timeout on incomplete messages
//void check_timeout() {
//    if (buffer_index > 0 && (time_us_64() - last_rx_time) > TIMEOUT_US) {
//        printf("Timeout on incomplete message: %.*s\n", buffer_index, buffer);
//        buffer_index = 0;
//    }
//}

/*   PID Equation   */
void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if (Iterm > 200) Iterm=200;
  else if (Iterm <-200) Iterm=-200;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>200) PIDOutput=200;
  else if (PIDOutput <-200) PIDOutput=-200;

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

// Initialize VL53L8CX sensor
//VL53L8CX_Configuration Dev;           // ToF sensor configuration
//VL53L8CX_ResultsData Results;         // ToF results data
//uint8_t status, isReady, isAlive;              // For ranging status
//Dev.platform.address = LIDAR_ADDR; /* Default address */
//Dev.platform.i2c_instance = i2c;   /* I2C instance */

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
    int status;

    status = bmi088.begin();
    if (status < 0)
    {
        printf("Error: %d\n", status);
    }

    // Initialize motors
    setup_motors();

    set_motor_speed(MOTOR1_PIN, 100);
    sleep_ms(50);
    set_motor_speed(MOTOR1_PIN, 0);
    sleep_ms(50);
    set_motor_speed(MOTOR2_PIN, 100);
    sleep_ms(50);
    set_motor_speed(MOTOR2_PIN, 0);
    sleep_ms(50);
    set_motor_speed(MOTOR3_PIN, 100);
    sleep_ms(50);
    set_motor_speed(MOTOR3_PIN, 0);
    sleep_ms(50);
    set_motor_speed(MOTOR4_PIN, 100);
    sleep_ms(50);
    set_motor_speed(MOTOR4_PIN, 0);
    sleep_ms(50);
    printf("Motors Initialized\n");

    gpio_put(LED_GREEN_L, 0);
    gpio_put(LED_GREEN_R, 0);
    gpio_put(LED_RED_L, 1);
    gpio_put(LED_RED_R, 1);
    sleep_ms(1000);

    // Avoid accidental liftoff
    throttle = 0; roll = 0; pitch = 0; yaw = 0;
    while (ReceiverValue[2] < 10 || ReceiverValue[2] > 15) {
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
    AccX = -bmi088.getAccelX_mss() / 9.81 + 0.03; // Convert to G //invert X axis
    AccY = bmi088.getAccelY_mss() / 9.81;
    AccZ = bmi088.getAccelZ_mss() / 9.81 + 0.03;
    //printf("AccX: %f, AccY: %f, AccZ: %f\n", AccX, AccY, AccZ);
    
    // Read gyroscope data
    RateRoll = -bmi088.getGyroX_rads() * RadtoDeg; //to degrees  //invert X axis
    RatePitch = bmi088.getGyroY_rads() * RadtoDeg;
    RateYaw = bmi088.getGyroZ_rads() * RadtoDeg;  // Z axis down is positive

    AngleRoll = -atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * RadtoDeg; //to degrees
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * RadtoDeg; //to degrees
}

void loop() {

    bmi_signals();
    //printf("Accel: %f %f %f\n", AccX, AccY, AccZ);
    //printf("Roll_angle: %f, Pitch_angle: %f\n", AngleRoll, AnglePitch);
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll=Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll=Kalman1DOutput[1];

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch=Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

    printf("KalmanAngleRoll: %f, KalmanAnglePitch: %f\n", KalmanAngleRoll, KalmanAnglePitch);

    // Read receiver values
    //Doing with interrupt to save in throttle, roll, pitch and yaw variables
    read_receiver();

    //Calculate desired angles from receiver inputs
    DesiredAngleRoll = 0.6 * (ReceiverValue[0] - 1500); //limit to 30 degrees (0.06)
    DesiredAnglePitch = 0.6 * (ReceiverValue[1] - 1500);

    InputThrottle = ReceiverValue[2];
    DesiredRateYaw = 0.10 * (ReceiverValue[3] - 1500); //limit to 50 degrees/s (0.1)

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

    if (InputThrottle > 800) InputThrottle = 800; //limit throttle to 80% to permit roll, pitch and yaw estabilization

    // Calculate motor inputs
    MotorInput1 = 1.024*(InputThrottle-InputRoll-InputPitch+InputYaw);
    MotorInput2 = 1.024*(InputThrottle-InputRoll+InputPitch-InputYaw);
    MotorInput3 = 1.024*(InputThrottle+InputRoll+InputPitch+InputYaw);
    MotorInput4 = 1.024*(InputThrottle+InputRoll-InputPitch-InputYaw);

    //printf("InputRoll: %f, InputPitch: %f, InputYaw: %f\n", InputRoll, InputPitch, InputYaw);

    //Limit motor inputs to 0-2000 microseconds
    if (MotorInput1 > 1000)MotorInput1 = 999;
    if (MotorInput2 > 1000)MotorInput2 = 999; 
    if (MotorInput3 > 1000)MotorInput3 = 999; 
    if (MotorInput4 > 1000)MotorInput4 = 999;

    //Keep motors at 18% if throttle is low
    int ThrottleIdle=180;
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
    }

    printf("M1: %f, M2: %f, M3: %f, M4: %f\n", MotorInput1, MotorInput2, MotorInput3, MotorInput4);

    //send commands to motors
    set_motor_speed(MOTOR1_PIN, MotorInput1);
    set_motor_speed(MOTOR2_PIN, MotorInput2);
    set_motor_speed(MOTOR3_PIN, MotorInput3);
    set_motor_speed(MOTOR4_PIN, MotorInput4);
    
    
    printf("Roll_angle: %f, Pitch_angle: %f\n", KalmanAngleRoll, KalmanAnglePitch);
    uint32_t current_time = time_us_32();
    while (current_time - LoopTimer < 4000)
    {
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