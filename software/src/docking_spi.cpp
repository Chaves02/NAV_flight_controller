#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "pico/time.h"

/*

// Hardware Configuration
#define CAMERA_SPI_PORT spi1
#define CAMERA_SCK_PIN  26
#define CAMERA_MOSI_PIN 27
#define CAMERA_MISO_PIN 28
#define CAMERA_CS_PIN   29

#define BLE_UART_ID uart1
#define BLE_TX_PIN 8
#define BLE_RX_PIN 9
#define BLE_BAUD_RATE 115200

// Image Processing Parameters
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define MAX_BLOBS 10
#define MIN_BLOB_SIZE 5
#define MAX_BLOB_SIZE 200

// PID Controller Parameters
#define PID_KP_XY 0.5f
#define PID_KI_XY 0.1f
#define PID_KD_XY 0.2f
#define PID_KP_YAW 0.3f
#define PID_KI_YAW 0.05f
#define PID_KD_YAW 0.1f

#define CONTROL_FREQUENCY 50 // Hz
#define CONTROL_PERIOD_US (1000000 / CONTROL_FREQUENCY)

// Data Structures
typedef struct {
    float x, y;
    int size;
    uint8_t color; // 0=red, 1=green
} blob_t;

typedef struct {
    float x, y;     // Position relative to docking center
    float yaw;      // Orientation angle
    float altitude; // From LIDAR (if available)
    bool valid;     // Whether pose estimation is valid
} pose_t;

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output_min, output_max;
} pid_controller_t;

typedef struct {
    int16_t roll, pitch, throttle, yaw;
} rc_command_t;

// Global Variables
static uint8_t frame_buffer[FRAME_WIDTH * FRAME_HEIGHT * 3]; // RGB888
static blob_t detected_blobs[MAX_BLOBS];
static int blob_count = 0;
static pose_t current_pose = {0};
static pid_controller_t pid_x, pid_y, pid_yaw;
static bool system_active = true;

// Function Prototypes
void init_camera(void);
void init_ble_uart(void);
void init_pid_controllers(void);
bool capture_frame(void);
int detect_leds(blob_t* blobs);
bool estimate_pose(blob_t* blobs, int count, pose_t* pose);
rc_command_t compute_rc_commands(pose_t* pose);
void send_ble_command(rc_command_t* cmd);
void core1_entry(void);
float pid_update(pid_controller_t* pid, float error, float dt);
void reset_pid(pid_controller_t* pid);

// Camera Interface Functions
void init_camera(void) {
    // Initialize SPI for camera
    spi_init(CAMERA_SPI_PORT, 8000000); // 8MHz
    gpio_set_function(CAMERA_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(CAMERA_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(CAMERA_MISO_PIN, GPIO_FUNC_SPI);
    
    // Chip select pin
    gpio_init(CAMERA_CS_PIN);
    gpio_set_dir(CAMERA_CS_PIN, GPIO_OUT);
    gpio_put(CAMERA_CS_PIN, 1);
    
    sleep_ms(100);
    
    // Camera initialization sequence (Arducam specific)
    uint8_t init_cmd[] = {0x01, 0x80}; // Reset command
    gpio_put(CAMERA_CS_PIN, 0);
    spi_write_blocking(CAMERA_SPI_PORT, init_cmd, 2);
    gpio_put(CAMERA_CS_PIN, 1);
    
    sleep_ms(100);
    
    // Set resolution and format
    uint8_t config_cmd[] = {0x20, 0x01, 0x40, 0x01, 0xF0}; // 320x240 RGB
    gpio_put(CAMERA_CS_PIN, 0);
    spi_write_blocking(CAMERA_SPI_PORT, config_cmd, 5);
    gpio_put(CAMERA_CS_PIN, 1);
    
    sleep_ms(50);
    printf("Camera initialized\n");
}

bool capture_frame(void) {
    uint8_t capture_cmd = 0x84; // Capture single frame
    uint8_t status;
    
    // Start capture
    gpio_put(CAMERA_CS_PIN, 0);
    spi_write_blocking(CAMERA_SPI_PORT, &capture_cmd, 1);
    gpio_put(CAMERA_CS_PIN, 1);
    
    // Wait for capture complete
    int timeout = 1000;
    do {
        gpio_put(CAMERA_CS_PIN, 0);
        uint8_t status_cmd = 0x41;
        //spi_write_blocking(CAMERA_SPI_PORT, &status_cmd, 1);
        spi_read_blocking(CAMERA_SPI_PORT, status_cmd, &status, 1);
        gpio_put(CAMERA_CS_PIN, 1);
        sleep_us(100);
    } while ((status & 0x08) == 0 && --timeout > 0);
    
    if (timeout == 0) return false;
    
    // Read frame data
    uint8_t read_cmd = 0x3D;
    gpio_put(CAMERA_CS_PIN, 0);
    //spi_write_blocking(CAMERA_SPI_PORT, &read_cmd, 1);
    spi_read_blocking(CAMERA_SPI_PORT, read_cmd, frame_buffer, FRAME_WIDTH * FRAME_HEIGHT * 3);
    gpio_put(CAMERA_CS_PIN, 1);
    
    return true;
}

// LED Detection using color thresholding
int detect_leds(blob_t* blobs) {
    int blob_cnt = 0;
    bool visited[FRAME_WIDTH * FRAME_HEIGHT] = {false};
    
    // Color thresholds (RGB)
    const uint8_t red_min[3] = {150, 0, 0};
    const uint8_t red_max[3] = {255, 100, 100};
    const uint8_t green_min[3] = {0, 150, 0};
    const uint8_t green_max[3] = {100, 255, 100};
    
    for (int y = 1; y < FRAME_HEIGHT - 1; y++) {
        for (int x = 1; x < FRAME_WIDTH - 1; x++) {
            int idx = (y * FRAME_WIDTH + x) * 3;
            if (visited[y * FRAME_WIDTH + x]) continue;
            
            uint8_t r = frame_buffer[idx];
            uint8_t g = frame_buffer[idx + 1];
            uint8_t b = frame_buffer[idx + 2];
            
            uint8_t color = 2; // Invalid
            
            // Check if pixel matches red LED
            if (r >= red_min[0] && r <= red_max[0] &&
                g >= red_min[1] && g <= red_max[1] &&
                b >= red_min[2] && b <= red_max[2]) {
                color = 0; // Red
            }
            // Check if pixel matches green LED
            else if (r >= green_min[0] && r <= green_max[0] &&
                     g >= green_min[1] && g <= green_max[1] &&
                     b >= green_min[2] && b <= green_max[2]) {
                color = 1; // Green
            }
            
            if (color < 2 && blob_cnt < MAX_BLOBS) {
                // Flood fill to find blob
                int blob_pixels = 0;
                float sum_x = 0, sum_y = 0;
                
                // Simple stack-based flood fill
                int stack_x[1000], stack_y[1000];
                int stack_top = 0;
                
                stack_x[0] = x;
                stack_y[0] = y;
                stack_top = 1;
                
                while (stack_top > 0 && blob_pixels < MAX_BLOB_SIZE) {
                    int cx = stack_x[--stack_top];
                    int cy = stack_y[stack_top];
                    
                    if (cx < 0 || cx >= FRAME_WIDTH || cy < 0 || cy >= FRAME_HEIGHT)
                        continue;
                    if (visited[cy * FRAME_WIDTH + cx])
                        continue;
                    
                    int cidx = (cy * FRAME_WIDTH + cx) * 3;
                    uint8_t cr = frame_buffer[cidx];
                    uint8_t cg = frame_buffer[cidx + 1];
                    uint8_t cb = frame_buffer[cidx + 2];
                    
                    bool matches = false;
                    if (color == 0) { // Red
                        matches = (cr >= red_min[0] && cr <= red_max[0] &&
                                  cg >= red_min[1] && cg <= red_max[1] &&
                                  cb >= red_min[2] && cb <= red_max[2]);
                    } else { // Green
                        matches = (cr >= green_min[0] && cr <= green_max[0] &&
                                  cg >= green_min[1] && cg <= green_max[1] &&
                                  cb >= green_min[2] && cb <= green_max[2]);
                    }
                    
                    if (!matches) continue;
                    
                    visited[cy * FRAME_WIDTH + cx] = true;
                    sum_x += cx;
                    sum_y += cy;
                    blob_pixels++;
                    
                    // Add neighbors to stack
                    if (stack_top < 996) {
                        stack_x[stack_top] = cx - 1; stack_y[stack_top++] = cy;
                        stack_x[stack_top] = cx + 1; stack_y[stack_top++] = cy;
                        stack_x[stack_top] = cx; stack_y[stack_top++] = cy - 1;
                        stack_x[stack_top] = cx; stack_y[stack_top++] = cy + 1;
                    }
                }
                
                if (blob_pixels >= MIN_BLOB_SIZE) {
                    blobs[blob_cnt].x = sum_x / blob_pixels;
                    blobs[blob_cnt].y = sum_y / blob_pixels;
                    blobs[blob_cnt].size = blob_pixels;
                    blobs[blob_cnt].color = color;
                    blob_cnt++;
                }
            }
        }
    }
    
    return blob_cnt;
}

// Pose estimation from LED positions
bool estimate_pose(blob_t* blobs, int count, pose_t* pose) {
    // Find green and red LED pairs
    blob_t green_leds[2], red_leds[2];
    int green_count = 0, red_count = 0;
    
    for (int i = 0; i < count; i++) {
        if (blobs[i].color == 1 && green_count < 2) { // Green
            green_leds[green_count++] = blobs[i];
        } else if (blobs[i].color == 0 && red_count < 2) { // Red
            red_leds[red_count++] = blobs[i];
        }
    }
    
    if (green_count < 2 || red_count < 2) {
        pose->valid = false;
        return false;
    }
    
    // Calculate front (green) and rear (red) centers
    float front_x = (green_leds[0].x + green_leds[1].x) / 2.0f;
    float front_y = (green_leds[0].y + green_leds[1].y) / 2.0f;
    float rear_x = (red_leds[0].x + red_leds[1].x) / 2.0f;
    float rear_y = (red_leds[0].y + red_leds[1].y) / 2.0f;
    
    // Drone center position
    float center_x = (front_x + rear_x) / 2.0f;
    float center_y = (front_y + rear_y) / 2.0f;
    
    // Convert to world coordinates (center of docking station is 0,0)
    pose->x = (center_x - FRAME_WIDTH / 2.0f) / (FRAME_WIDTH / 2.0f); // Normalize to [-1, 1]
    pose->y = (center_y - FRAME_HEIGHT / 2.0f) / (FRAME_HEIGHT / 2.0f);
    
    // Calculate yaw angle from front-rear vector
    float dx = front_x - rear_x;
    float dy = front_y - rear_y;
    pose->yaw = atan2f(dy, dx) * 180.0f / M_PI;
    
    pose->valid = true;
    return true;
}

// PID Controller Implementation
void init_pid_controllers(void) {
    // X position PID
    pid_x.kp = PID_KP_XY;
    pid_x.ki = PID_KI_XY;
    pid_x.kd = PID_KD_XY;
    pid_x.integral = 0;
    pid_x.prev_error = 0;
    pid_x.output_min = -500;
    pid_x.output_max = 500;
    
    // Y position PID
    pid_y.kp = PID_KP_XY;
    pid_y.ki = PID_KI_XY;
    pid_y.kd = PID_KD_XY;
    pid_y.integral = 0;
    pid_y.prev_error = 0;
    pid_y.output_min = -500;
    pid_y.output_max = 500;
    
    // Yaw PID
    pid_yaw.kp = PID_KP_YAW;
    pid_yaw.ki = PID_KI_YAW;
    pid_yaw.kd = PID_KD_YAW;
    pid_yaw.integral = 0;
    pid_yaw.prev_error = 0;
    pid_yaw.output_min = -500;
    pid_yaw.output_max = 500;
}

float pid_update(pid_controller_t* pid, float error, float dt) {
    // Proportional term
    float proportional = pid->kp * error;
    
    // Integral term
    pid->integral += error * dt;
    float integral = pid->ki * pid->integral;
    
    // Derivative term
    float derivative = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // Calculate output
    float output = proportional + integral + derivative;
    
    // Clamp output
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    return output;
}

void reset_pid(pid_controller_t* pid) {
    pid->integral = 0;
    pid->prev_error = 0;
}

// RC Command Generation
rc_command_t compute_rc_commands(pose_t* pose) {
    rc_command_t cmd = {0};
    static float dt = 1.0f / CONTROL_FREQUENCY;
    
    if (!pose->valid) {
        // Return neutral commands if pose is invalid
        cmd.roll = 1500;
        cmd.pitch = 1500;
        cmd.throttle = 0; /////////////////////////////////////////////////////////////////////////////////////////////////////////////////7
        cmd.yaw = 1500;
        return cmd;
    }
    
    // Calculate errors (setpoint is 0,0,0 for centered position)
    float error_x = -pose->x; // Negative because we want to move drone to center
    float error_y = -pose->y;
    float error_yaw = -pose->yaw;
    
    // Normalize yaw error to [-180, 180]
    while (error_yaw > 180) error_yaw -= 360;
    while (error_yaw < -180) error_yaw += 360;
    
    // Generate PID outputs
    float roll_output = pid_update(&pid_x, error_x, dt);
    float pitch_output = pid_update(&pid_y, error_y, dt);
    float yaw_output = pid_update(&pid_yaw, error_yaw, dt);
    
    // Convert to RC PWM values (1000-2000 microseconds, 1500 = neutral)
    cmd.roll = (int16_t)(1500 + roll_output);
    cmd.pitch = (int16_t)(1500 + pitch_output);
    cmd.yaw = (int16_t)(1500 + yaw_output);
    cmd.throttle = 0; ////////////////////////////////////////////////////////////////////////////// Hover throttle - adjust based on drone weight
    
    // Clamp to valid RC range
    if (cmd.roll < 1000) cmd.roll = 1000;
    if (cmd.roll > 2000) cmd.roll = 2000;
    if (cmd.pitch < 1000) cmd.pitch = 1000;
    if (cmd.pitch > 2000) cmd.pitch = 2000;
    if (cmd.yaw < 1000) cmd.yaw = 1000;
    if (cmd.yaw > 2000) cmd.yaw = 2000;
    if (cmd.throttle < 1000) cmd.throttle = 1000;
    if (cmd.throttle > 2000) cmd.throttle = 2000;
    
    return cmd;
}

// BLE Communication
void init_ble_uart(void) {
    uart_init(BLE_UART_ID, BLE_BAUD_RATE);
    gpio_set_function(BLE_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(BLE_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(BLE_UART_ID, false, false);
    uart_set_format(BLE_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(BLE_UART_ID, false);
    printf("BLE UART initialized\n");
}

void send_ble_command(rc_command_t* cmd) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "RC,%d,%d,%d,%d\n", 
             cmd->roll, cmd->pitch, cmd->throttle, cmd->yaw);
    
    uart_puts(BLE_UART_ID, buffer);
}

// Core 1 - Image Processing and Control
void core1_entry(void) {
    printf("Core 1: Starting image processing loop\n");
    
    absolute_time_t next_control_time = get_absolute_time();
    
    while (system_active) {
        absolute_time_t start_time = get_absolute_time();
        
        // Capture frame
        if (capture_frame()) {
            // Detect LEDs
            blob_count = detect_leds(detected_blobs);
            
            // Estimate pose
            if (estimate_pose(detected_blobs, blob_count, &current_pose)) {
                // Compute RC commands
                rc_command_t cmd = compute_rc_commands(&current_pose);
                
                // Send commands via BLE
                send_ble_command(&cmd);
                
                // Debug output
                printf("Pose: x=%.3f, y=%.3f, yaw=%.1f | RC: %d,%d,%d,%d\n",
                       current_pose.x, current_pose.y, current_pose.yaw,
                       cmd.roll, cmd.pitch, cmd.throttle, cmd.yaw);
            } else {
                printf("Invalid pose - LEDs not detected properly\n");
            }
        } else {
            printf("Frame capture failed\n");
        }
        
        // Maintain control frequency
        next_control_time = delayed_by_us(next_control_time, CONTROL_PERIOD_US);
        sleep_until(next_control_time);
    }
}

// Main function - Core 0
int main(void) {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB CDC
    
    printf("Autonomous Drone Docking Station Starting...\n");
    
    // Initialize hardware
    init_camera();
    init_ble_uart();
    init_pid_controllers();
    
    printf("Hardware initialized. Starting control loop on Core 1...\n");
    
    // Launch image processing on Core 1
    multicore_launch_core1(core1_entry);
    
    // Core 0 handles system monitoring and user interface
    while (true) {
        // Monitor system status
        if (current_pose.valid) {
            printf("System Status: TRACKING | Blobs: %d | Pose Valid: %s\n",
                   blob_count, current_pose.valid ? "YES" : "NO");
        }
        
        sleep_ms(2000);
        
        // Check for stop condition (could add button or other trigger)
        // system_active = check_stop_condition();
    }
    
    return 0;
}

*/






























#define LED_RUN 11
#define CHARGER 12 //activate drone charger and led also indicate

// Monitor faults
#define SOLAR 21      //CHRG
#define FAULT 20      //FAULT
#define BAT_SENSE 13  // BAT_SENSE

// I2C
#define I2C_SDA0 16
#define I2C_SCL0 17

#define I2C_SDA1 14
#define I2C_SCL1 15
#define I2C_BAUD 400000 // 400kHz I2C speed

#include "pico/stdlib.h"
#include "tusb.h"  // TinyUSB for USB CDC (PC ↔ RP2040)
#include "rp_agrolib_bc832.h"

// UART and BLE settings
#define UART_ID uart0
#define UART_TX 0
#define UART_RX 1
#define MODE_PIN 10
#define UART_BAUDRATE 115200

// Buffer for incoming data
#define MAX_BUFFER_LEN 128
char buffer[MAX_BUFFER_LEN];
volatile int buffer_index = 0;

// Instantiate BLE interface
RP_AGROLIB_BC832_Simple ble(UART_ID, UART_TX, UART_RX, MODE_PIN, UART_BAUDRATE);

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("\n[RP2040] BLE RC Relay Starting...\n");

    // Init BLE module
    if (!ble.begin()) {
        printf("BLE module failed to initialize\n");
        while (1) tight_loop_contents();
    }

    ble.reset();
    ble.setName("DockingStation");
    ble.setRFPower(0);

    if (!ble.setupAutoconnect(10000)) {
        printf("BLE autoconnect setup failed\n");
        while (1) tight_loop_contents();
    }

    printf("BLE module initialized and ready\n");

    // Main loop
    while (true) {
        tud_task();  // USB CDC background tasks

        if(!ble.isConnected())
        {
            while (uart_is_readable(UART_ID)) {
                printf("Reading from UART...\n");
                char c = uart_getc(UART_ID);

                if (buffer_index < MAX_BUFFER_LEN - 1) {
                    buffer[buffer_index++] = c;
                }

                // Process complete messages
                if (c == '\n' || c == '\r') {
                    if (buffer_index > 0) {
                        ble.processIncomingData(buffer, buffer_index);
                        buffer_index = 0;
                    } else {
                        buffer_index = 0;
                    }
                }
            }
        }

        // Check USB for data from PC
        if (tud_cdc_available()) {
            //printf("Serial data available\n");
            char buffer[256];
            int len = tud_cdc_read(buffer, sizeof(buffer));

            if (len > 0) {
                // Forward to BLE
                if(ble.isConnected()) {
                    ble.sendMessage(std::string(buffer));
                } else {
                    printf("BLE not connected, cannot forward: %s\n", buffer);
                }
            }
        }
    }

    return 0;
}
