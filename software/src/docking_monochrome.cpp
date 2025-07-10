#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/time.h"

// HM0360 Camera Configuration
#define CAMERA_I2C_PORT i2c0
#define CAMERA_SDA_PIN 16
#define CAMERA_SCL_PIN 17
#define CAMERA_I2C_ADDR 0x24 //////////////////////////////////////////

// Parallel camera interface pins
#define CAM_PCLK_PIN 26
#define CAM_VSYNC_PIN 27
#define CAM_HREF_PIN 28
#define CAM_D0_PIN 29

// Pin definitions for BC832 module
#define BLE_UART_ID uart0
#define BLE_TX_PIN 0
#define BLE_RX_PIN 1
#define BLE_BAUD_RATE 115200

// System Control Pins
#define STATUS_LED_PIN 11
#define CHARGER 12 //activate drone charger and led also indicate
//#define SYSTEM_ENABLE_PIN 22

// Monitor faults
#define SOLAR 21      //CHRG
#define FAULT 20      //FAULT
#define BAT_SENSE 13  // BAT_SENSE

// Image Processing Parameters
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define FRAME_SIZE (FRAME_WIDTH * FRAME_HEIGHT)
#define MAX_LEDS 10
#define MIN_LED_SIZE 8
#define MAX_LED_SIZE 100

// LED Detection Parameters
#define LED_THRESHOLD 200  // Brightness threshold for LED detection
#define TRIANGLE_TOLERANCE 0.3f  // Tolerance for triangle shape detection

// PID Controller Parameters
#define PID_KP_XY 0.4f
#define PID_KI_XY 0.08f
#define PID_KD_XY 0.15f
#define PID_KP_YAW 0.25f
#define PID_KI_YAW 0.04f
#define PID_KD_YAW 0.08f

#define CONTROL_FREQUENCY 50 // Hz
#define CONTROL_PERIOD_US (1000000 / CONTROL_FREQUENCY)

// Safety Parameters
#define MAX_TRACKING_LOSS_MS 2000  // Maximum time without valid tracking
#define LANDING_THRESHOLD_PX 15    // Distance threshold for landing
#define HOVER_HEIGHT_THROTTLE 1480 // Throttle for hover (adjust based on drone)

// Data Structures
typedef struct {
    float x, y;
    int size;
    uint8_t brightness;
} led_t;

typedef struct {
    float x, y;        // Position relative to docking center
    float yaw;         // Orientation angle from triangle
    float scale;       // Size indicator (distance estimate)
    bool valid;        // Whether pose estimation is valid
    absolute_time_t timestamp; // When this pose was calculated
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

typedef enum {
    SYSTEM_INIT,
    SYSTEM_SEARCHING,
    SYSTEM_TRACKING,
    SYSTEM_LANDING,
    SYSTEM_LANDED,
    SYSTEM_ERROR
} system_state_t;

// Global Variables
static uint8_t frame_buffer[FRAME_SIZE];
static led_t detected_leds[MAX_LEDS];
static int led_count = 0;
static pose_t current_pose = {0};
static pid_controller_t pid_x, pid_y, pid_yaw;
static bool system_active = true;
static int dma_channel;
static system_state_t system_state = SYSTEM_INIT;
static absolute_time_t last_valid_tracking;
static uint32_t frame_counter = 0;
static uint32_t successful_frames = 0;

// HM0360 Register Configuration
typedef struct {
    uint16_t reg;
    uint8_t val;
} camera_reg_t;

// HM0360 initialization registers for 320x240 monochrome
static const camera_reg_t hm0360_config[] = {
    {0x0103, 0x00}, // Software reset
    {0x0100, 0x00}, // Standby
    {0x0601, 0x01}, // Test pattern off
    {0x3044, 0x0A}, // Dark row
    {0x3045, 0x00}, // Dark row
    {0x3047, 0x0A}, // Dark row
    {0x3050, 0xC0}, // Analog gain
    {0x3060, 0x01}, // Analog gain
    {0x3080, 0x02}, // Clock settings
    {0x3081, 0x3C}, // PLL settings
    {0x3082, 0x04}, // PLL settings
    {0x3083, 0x00}, // PLL settings
    {0x3084, 0x02}, // PLL settings
    {0x3085, 0x01}, // PLL settings
    {0x3086, 0x01}, // PLL settings
    {0x3087, 0x01}, // PLL settings
    {0x30A0, 0x01}, // Image orientation
    {0x30A1, 0x00}, // Image orientation
    {0x30B0, 0x05}, // Digital gain
    {0x30B1, 0x00}, // Digital gain
    {0x30B2, 0x00}, // Digital gain
    {0x3110, 0x70}, // Frame control
    {0x3111, 0x80}, // Frame control
    {0x3112, 0x7F}, // Frame control
    {0x3113, 0xC0}, // Frame control
    {0x3114, 0x00}, // Frame control
    {0x3115, 0x00}, // Frame control
    {0x0340, 0x02}, // Frame length MSB
    {0x0341, 0x16}, // Frame length LSB (534 lines)
    {0x0342, 0x01}, // Line length MSB  
    {0x0343, 0x78}, // Line length LSB (376 pixels)
    {0x0344, 0x00}, // X start MSB
    {0x0345, 0x00}, // X start LSB
    {0x0346, 0x00}, // Y start MSB
    {0x0347, 0x00}, // Y start LSB
    {0x0348, 0x01}, // X end MSB
    {0x0349, 0x3F}, // X end LSB (319)
    {0x034A, 0x00}, // Y end MSB
    {0x034B, 0xEF}, // Y end LSB (239)
    {0x034C, 0x01}, // X output size MSB
    {0x034D, 0x40}, // X output size LSB (320)
    {0x034E, 0x00}, // Y output size MSB
    {0x034F, 0xF0}, // Y output size LSB (240)
    {0x0100, 0x01}, // Streaming on
};

// Function Prototypes
void init_camera_i2c(void);
void init_camera_parallel(void);
bool configure_hm0360(void);
void init_ble_uart(void);
void init_pid_controllers(void);
void init_system_io(void);
bool capture_frame(void);
int detect_leds(led_t* leds);
bool estimate_pose_triangle(led_t* leds, int count, pose_t* pose);
rc_command_t compute_rc_commands(pose_t* pose);
void send_ble_command(rc_command_t* cmd);
void send_ble_status(void);
void core1_entry(void);
float pid_update(pid_controller_t* pid, float error, float dt);
void reset_pid(pid_controller_t* pid);
bool write_camera_reg(uint16_t reg, uint8_t val);
uint8_t read_camera_reg(uint16_t reg);
float calculate_triangle_area(led_t* p1, led_t* p2, led_t* p3);
bool is_valid_triangle(led_t* leds, int* indices);
void update_system_state(void);
bool is_drone_centered(void);
void handle_emergency_stop(void);

// System I/O Initialization
void init_system_io(void) {
    // Status LED
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    gpio_put(STATUS_LED_PIN, 1);
    
    // System enable pin (pullup, active low)
    //gpio_init(SYSTEM_ENABLE_PIN);
    //gpio_set_dir(SYSTEM_ENABLE_PIN, GPIO_IN);
    //gpio_pull_up(SYSTEM_ENABLE_PIN);
    
    printf("System I/O initialized\n");
}

// Camera I2C Interface
void init_camera_i2c(void) {
    i2c_init(CAMERA_I2C_PORT, 400000); // 400kHz
    gpio_set_function(CAMERA_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(CAMERA_SCL_PIN, GPIO_FUNC_I2C);
    //gpio_pull_up(CAMERA_SDA_PIN);
    //gpio_pull_up(CAMERA_SCL_PIN);
    
    sleep_ms(100);
    printf("Camera I2C initialized\n");
}

bool write_camera_reg(uint16_t reg, uint8_t val) {
    uint8_t data[3] = {
        (reg >> 8) & 0xFF,  // Register high byte
        reg & 0xFF,         // Register low byte
        val                 // Value
    };
    
    int result = i2c_write_blocking(CAMERA_I2C_PORT, CAMERA_I2C_ADDR, data, 3, false);
    return result == 3;
}

uint8_t read_camera_reg(uint16_t reg) {
    uint8_t reg_data[2] = {(reg >> 8) & 0xFF, reg & 0xFF};
    uint8_t val = 0;
    
    i2c_write_blocking(CAMERA_I2C_PORT, CAMERA_I2C_ADDR, reg_data, 2, true);
    i2c_read_blocking(CAMERA_I2C_PORT, CAMERA_I2C_ADDR, &val, 1, false);
    
    return val;
}

bool configure_hm0360(void) {
    printf("Configuring HM0360 camera...\n");
    
    // Check camera ID
    uint8_t id_h = read_camera_reg(0x0000);
    uint8_t id_l = read_camera_reg(0x0001);
    printf("Camera ID: 0x%02X%02X\n", id_h, id_l);
    
    if (id_h != 0x03 || id_l != 0x60) {
        printf("ERROR: Invalid camera ID\n");
        return false;
    }
    
    // Apply configuration
    for (int i = 0; i < sizeof(hm0360_config) / sizeof(camera_reg_t); i++) {
        if (!write_camera_reg(hm0360_config[i].reg, hm0360_config[i].val)) {
            printf("ERROR: Failed to write register 0x%04X\n", hm0360_config[i].reg);
            return false;
        }
        sleep_ms(1);
    }
    
    printf("HM0360 configured successfully\n");
    return true;
}

// Parallel Camera Interface using PIO
void init_camera_parallel(void) {
    // Initialize data pins
    for (int i = 0; i < 8; i++) {
        gpio_init(CAM_D0_PIN + i);
        gpio_set_dir(CAM_D0_PIN + i, GPIO_IN);
    }
    
    // Initialize control pins
    gpio_init(CAM_PCLK_PIN);
    gpio_set_dir(CAM_PCLK_PIN, GPIO_IN);
    gpio_init(CAM_VSYNC_PIN);
    gpio_set_dir(CAM_VSYNC_PIN, GPIO_IN);
    gpio_init(CAM_HREF_PIN);
    gpio_set_dir(CAM_HREF_PIN, GPIO_IN);
    
    // Setup DMA for frame capture
    dma_channel = dma_claim_unused_channel(true);
    
    printf("Camera parallel interface initialized\n");
}

// Improved frame capture with better synchronization and debugging
bool capture_frame(void) {
    int pixel_count = 0;
    bool in_frame = false;
    bool line_active = false;
    int timeout_counter = 0;
    const int MAX_TIMEOUT = 100000; // Prevent infinite loops
    
    // Add debug info every 1000 frames
    static int debug_counter = 0;
    bool debug_this_frame = (debug_counter++ % 1000 == 0);
    
    if (debug_this_frame) {
        printf("Capture attempt - VSYNC: %d, HREF: %d, PCLK: %d\n", 
               gpio_get(CAM_VSYNC_PIN), gpio_get(CAM_HREF_PIN), gpio_get(CAM_PCLK_PIN));
    }
    
    // Wait for VSYNC high (start of frame) with timeout
    timeout_counter = 0;
    while (!gpio_get(CAM_VSYNC_PIN) && timeout_counter < MAX_TIMEOUT) {
        sleep_us(10);
        timeout_counter++;
    }
    
    if (timeout_counter >= MAX_TIMEOUT) {
        if (debug_this_frame) printf("Timeout waiting for VSYNC high\n");
        return false;
    }
    
    // Wait for VSYNC low (active frame) with timeout
    timeout_counter = 0;
    while (gpio_get(CAM_VSYNC_PIN) && timeout_counter < MAX_TIMEOUT) {
        sleep_us(10);
        timeout_counter++;
    }
    
    if (timeout_counter >= MAX_TIMEOUT) {
        if (debug_this_frame) printf("Timeout waiting for VSYNC low\n");
        return false;
    }
    
    in_frame = true;
    int lines_captured = 0;
    int pixels_this_line = 0;
    
    while (in_frame && pixel_count < FRAME_SIZE) {
        // Check for end of frame
        if (gpio_get(CAM_VSYNC_PIN)) {
            in_frame = false;
            if (debug_this_frame) {
                printf("Frame end detected. Lines: %d, Pixels: %d\n", lines_captured, pixel_count);
            }
            break;
        }
        
        // Check for line active
        bool href_state = gpio_get(CAM_HREF_PIN);
        
        if (href_state && !line_active) {
            // Start of new line
            line_active = true;
            lines_captured++;
            pixels_this_line = 0;
        } else if (!href_state && line_active) {
            // End of line
            line_active = false;
        }
        
        if (line_active) {
            // Wait for pixel clock rising edge with timeout
            timeout_counter = 0;
            while (!gpio_get(CAM_PCLK_PIN) && gpio_get(CAM_HREF_PIN) && timeout_counter < 1000) {
                timeout_counter++;
                // No sleep here for faster pixel capture
            }
            
            if (gpio_get(CAM_HREF_PIN) && gpio_get(CAM_PCLK_PIN)) {
                // Read pixel data from D0-D7
                uint8_t pixel = 0;
                uint32_t gpio_state = gpio_get_all();
                
                // Extract bits from GPIO state (assuming consecutive pins)
                for (int i = 0; i < 8; i++) {
                    if (gpio_state & (1u << (CAM_D0_PIN + i))) {
                        pixel |= (1 << i);
                    }
                }
                
                if (pixel_count < FRAME_SIZE) {
                    frame_buffer[pixel_count++] = pixel;
                    pixels_this_line++;
                }
                
                // Wait for pixel clock falling edge (optional, might be too slow)
                // while (gpio_get(CAM_PCLK_PIN) && gpio_get(CAM_HREF_PIN)) {
                //     // Brief wait
                // }
            }
        }
        
        // Prevent infinite loops
        if (++timeout_counter > MAX_TIMEOUT * 10) {
            if (debug_this_frame) printf("Overall timeout in capture loop\n");
            break;
        }
    }
    
    frame_counter++;
    
    // More lenient success criteria - even partial frames might be useful
    bool success = pixel_count > (FRAME_SIZE / 4); // At least quarter frame
    if (success) successful_frames++;
    
    if (debug_this_frame) {
        printf("Capture result: %s, pixels: %d/%d, lines: %d\n", 
               success ? "SUCCESS" : "FAIL", pixel_count, FRAME_SIZE, lines_captured);
        
        // Sample a few pixels to verify data
        if (pixel_count > 10) {
            printf("Sample pixels: ");
            for (int i = 0; i < 10 && i < pixel_count; i++) {
                printf("%02X ", frame_buffer[i]);
            }
            printf("\n");
        }
    }
    
    return success;
}

// Additional debugging function to check camera signals
void debug_camera_signals(void) {
    printf("=== Camera Signal Debug ===\n");
    
    for (int i = 0; i < 100; i++) {
        bool vsync = gpio_get(CAM_VSYNC_PIN);
        bool href = gpio_get(CAM_HREF_PIN);
        bool pclk = gpio_get(CAM_PCLK_PIN);
        
        // Check data pins
        uint8_t data = 0;
        for (int j = 0; j < 8; j++) {
            if (gpio_get(CAM_D0_PIN + j)) {
                data |= (1 << j);
            }
        }
        
        printf("VSYNC:%d HREF:%d PCLK:%d DATA:0x%02X\n", vsync, href, pclk, data);
        sleep_ms(10);
    }
    
    printf("=== End Signal Debug ===\n");
}

// Alternative simpler capture method for testing
bool capture_frame_simple(void) {
    // Just fill buffer with test pattern to verify LED detection works
    static bool test_mode = false;
    static int test_counter = 0;
    
    // Enable test mode after many failed attempts
    if (successful_frames == 0 && frame_counter > 100) {
        test_mode = true;
    }
    
    if (test_mode) {
        // Create a test pattern with bright spots (simulated LEDs)
        memset(frame_buffer, 50, FRAME_SIZE); // Dark background
        
        // Add some bright "LED" spots
        int centers[][2] = {{80, 60}, {240, 60}, {160, 180}}; // Triangle pattern
        
        for (int led = 0; led < 3; led++) {
            int cx = centers[led][0];
            int cy = centers[led][1];
            
            // Create circular bright region
            for (int dy = -5; dy <= 5; dy++) {
                for (int dx = -5; dx <= 5; dx++) {
                    int x = cx + dx;
                    int y = cy + dy;
                    
                    if (x >= 0 && x < FRAME_WIDTH && y >= 0 && y < FRAME_HEIGHT) {
                        float dist = sqrtf(dx*dx + dy*dy);
                        if (dist <= 5.0f) {
                            int idx = y * FRAME_WIDTH + x;
                            frame_buffer[idx] = 255 - (int)(dist * 20); // Bright center, dimmer edges
                        }
                    }
                }
            }
        }
        
        frame_counter++;
        successful_frames++;
        
        // Disable test mode after proving LED detection works
        if (test_counter++ > 50) {
            test_mode = false;
            test_counter = 0;
            printf("Test pattern mode disabled - try real camera again\n");
        }
        
        return true;
    }
    
    return capture_frame(); // Use real capture
}

// LED Detection for monochrome image
int detect_leds(led_t* leds) {
    int led_cnt = 0;
    bool visited[FRAME_SIZE] = {false};
    
    for (int y = 1; y < FRAME_HEIGHT - 1; y++) {
        for (int x = 1; x < FRAME_WIDTH - 1; x++) {
            int idx = y * FRAME_WIDTH + x;
            
            if (visited[idx] || frame_buffer[idx] < LED_THRESHOLD) {
                continue;
            }
            
            // Found bright pixel - start flood fill
            int led_pixels = 0;
            float sum_x = 0, sum_y = 0;
            uint32_t sum_brightness = 0;
            
            // Simple queue-based flood fill
            int queue_x[1000], queue_y[1000];
            int queue_head = 0, queue_tail = 0;
            
            queue_x[queue_tail] = x;
            queue_y[queue_tail] = y;
            queue_tail++;
            
            while (queue_head < queue_tail && led_pixels < MAX_LED_SIZE) {
                int cx = queue_x[queue_head];
                int cy = queue_y[queue_head];
                queue_head++;
                
                if (cx < 0 || cx >= FRAME_WIDTH || cy < 0 || cy >= FRAME_HEIGHT) {
                    continue;
                }
                
                int cidx = cy * FRAME_WIDTH + cx;
                if (visited[cidx] || frame_buffer[cidx] < LED_THRESHOLD) {
                    continue;
                }
                
                visited[cidx] = true;
                sum_x += cx;
                sum_y += cy;
                sum_brightness += frame_buffer[cidx];
                led_pixels++;
                
                // Add neighbors to queue
                if (queue_tail < 996) {
                    queue_x[queue_tail] = cx - 1; queue_y[queue_tail++] = cy;
                    queue_x[queue_tail] = cx + 1; queue_y[queue_tail++] = cy;
                    queue_x[queue_tail] = cx; queue_y[queue_tail++] = cy - 1;
                    queue_x[queue_tail] = cx; queue_y[queue_tail++] = cy + 1;
                }
            }
            
            if (led_pixels >= MIN_LED_SIZE && led_cnt < MAX_LEDS) {
                leds[led_cnt].x = sum_x / led_pixels;
                leds[led_cnt].y = sum_y / led_pixels;
                leds[led_cnt].size = led_pixels;
                leds[led_cnt].brightness = sum_brightness / led_pixels;
                led_cnt++;
            }
        }
    }
    
    return led_cnt;
}

// Calculate triangle area using cross product
float calculate_triangle_area(led_t* p1, led_t* p2, led_t* p3) {
    float area = fabsf((p1->x * (p2->y - p3->y) + 
                       p2->x * (p3->y - p1->y) + 
                       p3->x * (p1->y - p2->y)) / 2.0f);
    return area;
}

// Check if three LEDs form a valid triangle
bool is_valid_triangle(led_t* leds, int* indices) {
    led_t* p1 = &leds[indices[0]];
    led_t* p2 = &leds[indices[1]];
    led_t* p3 = &leds[indices[2]];
    
    // Calculate distances between points
    float d12 = sqrtf((p1->x - p2->x) * (p1->x - p2->x) + (p1->y - p2->y) * (p1->y - p2->y));
    float d23 = sqrtf((p2->x - p3->x) * (p2->x - p3->x) + (p2->y - p3->y) * (p2->y - p3->y));
    float d13 = sqrtf((p1->x - p3->x) * (p1->x - p3->x) + (p1->y - p3->y) * (p1->y - p3->y));
    
    // Check if points are not collinear (area > 0)
    float area = calculate_triangle_area(p1, p2, p3);
    if (area < 50.0f) return false; // Too small or collinear
    
    // Check if triangle is reasonably equilateral (adjust tolerance as needed)
    float avg_side = (d12 + d23 + d13) / 3.0f;
    float max_deviation = 0;
    max_deviation = fmaxf(max_deviation, fabsf(d12 - avg_side) / avg_side);
    max_deviation = fmaxf(max_deviation, fabsf(d23 - avg_side) / avg_side);
    max_deviation = fmaxf(max_deviation, fabsf(d13 - avg_side) / avg_side);
    
    return max_deviation < TRIANGLE_TOLERANCE;
}

// Pose estimation from triangle LED pattern
bool estimate_pose_triangle(led_t* leds, int count, pose_t* pose) {
    if (count < 3) {
        pose->valid = false;
        return false;
    }
    
    // Find the best triangle among detected LEDs
    int best_triangle[3] = {-1, -1, -1};
    float best_score = 0;
    
    // Try all combinations of 3 LEDs
    for (int i = 0; i < count - 2; i++) {
        for (int j = i + 1; j < count - 1; j++) {
            for (int k = j + 1; k < count; k++) {
                int indices[3] = {i, j, k};
                
                if (is_valid_triangle(leds, indices)) {
                    // Score based on brightness and size consistency
                    float brightness_score = (leds[i].brightness + leds[j].brightness + leds[k].brightness) / 3.0f;
                    float size_score = (leds[i].size + leds[j].size + leds[k].size) / 3.0f;
                    float total_score = brightness_score + size_score;
                    
                    if (total_score > best_score) {
                        best_score = total_score;
                        best_triangle[0] = i;
                        best_triangle[1] = j;
                        best_triangle[2] = k;
                    }
                }
            }
        }
    }
    
    if (best_triangle[0] == -1) {
        pose->valid = false;
        return false;
    }
    
    // Calculate centroid of triangle
    led_t* p1 = &leds[best_triangle[0]];
    led_t* p2 = &leds[best_triangle[1]];
    led_t* p3 = &leds[best_triangle[2]];
    
    float center_x = (p1->x + p2->x + p3->x) / 3.0f;
    float center_y = (p1->y + p2->y + p3->y) / 3.0f;
    
    // Convert to normalized coordinates
    pose->x = (center_x - FRAME_WIDTH / 2.0f) / (FRAME_WIDTH / 2.0f);
    pose->y = (center_y - FRAME_HEIGHT / 2.0f) / (FRAME_HEIGHT / 2.0f);
    
    // Calculate orientation from triangle orientation
    // Find the vertex that's most different (assuming one LED is at the "front")
    float d1 = (p1->x - center_x) * (p1->x - center_x) + (p1->y - center_y) * (p1->y - center_y);
    float d2 = (p2->x - center_x) * (p2->x - center_x) + (p2->y - center_y) * (p2->y - center_y);
    float d3 = (p3->x - center_x) * (p3->x - center_x) + (p3->y - center_y) * (p3->y - center_y);
    
    led_t* front_led;
    if (d1 >= d2 && d1 >= d3) {
        front_led = p1;
    } else if (d2 >= d3) {
        front_led = p2;
    } else {
        front_led = p3;
    }
    
    // Calculate yaw from center to front LED
    float dx = front_led->x - center_x;
    float dy = front_led->y - center_y;
    pose->yaw = atan2f(dy, dx) * 180.0f / M_PI;
    
    // Estimate scale/distance from triangle size
    float triangle_area = calculate_triangle_area(p1, p2, p3);
    pose->scale = triangle_area / 1000.0f; // Normalize
    
    pose->valid = true;
    pose->timestamp = get_absolute_time();
    
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
    
    // Integral term with windup protection
    pid->integral += error * dt;
    // Clamp integral to prevent windup
    float max_integral = pid->output_max / pid->ki;
    if (pid->integral > max_integral) pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;
    
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

// Check if drone is centered for landing
bool is_drone_centered(void) {
    if (!current_pose.valid) return false;
    
    float distance = sqrtf(current_pose.x * current_pose.x + current_pose.y * current_pose.y);
    float distance_px = distance * (FRAME_WIDTH / 2.0f);
    
    return distance_px < LANDING_THRESHOLD_PX;
}

// RC Command Generation
rc_command_t compute_rc_commands(pose_t* pose) {
    rc_command_t cmd = {0};
    static float dt = 1.0f / CONTROL_FREQUENCY;
    
    if (!pose->valid || system_state == SYSTEM_ERROR) {
        // Return neutral commands if pose is invalid or system error
        cmd.roll = 1500;
        cmd.pitch = 1500;
        cmd.throttle = 1500;
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
    
    // Throttle control based on system state
    switch (system_state) {
        case SYSTEM_TRACKING:
            cmd.throttle = HOVER_HEIGHT_THROTTLE;
            break;
        case SYSTEM_LANDING:
            // Gradual descent
            cmd.throttle = HOVER_HEIGHT_THROTTLE - 50;
            break;
        case SYSTEM_LANDED:
            cmd.throttle = 1000; // Minimum throttle
            break;
        default:
            cmd.throttle = 1500; // Neutral
            break;
    }
    
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
    
    // Set FIFO threshold and enable
    uart_set_fifo_enabled(BLE_UART_ID, true);
    
    sleep_ms(100);
    printf("BLE UART initialized at %d baud\n", BLE_BAUD_RATE);
}

void send_ble_command(rc_command_t* cmd) {
    if (!uart_is_enabled(BLE_UART_ID)) return;
    
    // Send RC command in MSP format or simple protocol
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), 
                      "RC,%d,%d,%d,%d\n", 
                      cmd->roll, cmd->pitch, cmd->throttle, cmd->yaw);
    
    if (len > 0 && len < sizeof(buffer)) {
        uart_puts(BLE_UART_ID, buffer);
    }
}

void send_ble_status(void) {
    if (!uart_is_enabled(BLE_UART_ID)) return;
    
    char buffer[128];
    int len = snprintf(buffer, sizeof(buffer),
                      "STATUS,%d,%d,%.2f,%.2f,%.2f,%.2f,%d,%d\n",
                      system_state, current_pose.valid ? 1 : 0,
                      current_pose.x, current_pose.y, current_pose.yaw, current_pose.scale,
                      led_count, successful_frames);
    
    if (len > 0 && len < sizeof(buffer)) {
        uart_puts(BLE_UART_ID, buffer);
    }
}

void update_system_state(void) {
    static absolute_time_t state_entry_time;
    static system_state_t prev_state = SYSTEM_INIT;
    
    // Initialize state entry time on state change
    if (prev_state != system_state) {
        state_entry_time = get_absolute_time();
        prev_state = system_state;
    }
    
    switch (system_state) {
        case SYSTEM_INIT:
            if (frame_counter > 10) {
                system_state = SYSTEM_SEARCHING;
                printf("State: SEARCHING\n");
            }
            break;
            
        case SYSTEM_SEARCHING:
            if (current_pose.valid) {
                system_state = SYSTEM_TRACKING;
                last_valid_tracking = get_absolute_time();
                reset_pid(&pid_x);
                reset_pid(&pid_y);
                reset_pid(&pid_yaw);
                printf("State: TRACKING\n");
            }
            break;
            
        case SYSTEM_TRACKING:
            if (!current_pose.valid) {
                // Check if tracking lost for too long
                if (absolute_time_diff_us(last_valid_tracking, get_absolute_time()) > 
                    MAX_TRACKING_LOSS_MS * 1000) {
                    system_state = SYSTEM_SEARCHING;
                    printf("State: TRACKING LOST -> SEARCHING\n");
                }
            } else {
                last_valid_tracking = get_absolute_time();
                
                // Check for landing condition
                if (is_drone_centered() && current_pose.scale > 0.8f) {
                    system_state = SYSTEM_LANDING;
                    printf("State: LANDING\n");
                }
            }
            break;
            
        case SYSTEM_LANDING:
            if (!current_pose.valid) {
                system_state = SYSTEM_SEARCHING;
                printf("State: LANDING LOST -> SEARCHING\n");
            } else if (current_pose.scale > 1.5f) { // Very close
                system_state = SYSTEM_LANDED;
                printf("State: LANDED\n");
            } else if (!is_drone_centered()) {
                system_state = SYSTEM_TRACKING;
                printf("State: LANDING -> TRACKING\n");
            }
            break;
            
        case SYSTEM_LANDED:
            // Stay landed until manual reset or drone moves away
            if (current_pose.valid && current_pose.scale < 0.5f) {
                system_state = SYSTEM_SEARCHING;
                printf("State: DRONE DEPARTED -> SEARCHING\n");
            }
            break;
            
        case SYSTEM_ERROR:
            // Stay in error state until manual reset
            break;
    }
}

void handle_emergency_stop(void) {
    // Check system enable pin (active low)
    if (0==1) {
        system_state = SYSTEM_ERROR;
        system_active = false;
        
        // Send emergency stop command
        rc_command_t emergency_cmd = {1500, 1500, 1000, 1500}; // Throttle to minimum
        send_ble_command(&emergency_cmd);
        
        printf("EMERGENCY STOP ACTIVATED\n");
        
        // Flash LED rapidly
        for (int i = 0; i < 10; i++) {
            gpio_put(STATUS_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(STATUS_LED_PIN, 0);
            sleep_ms(100);
        }
    }
}

// Core 1 - Image processing and control loop
void core1_entry(void) {
    absolute_time_t last_control_time = get_absolute_time();
    absolute_time_t last_status_time = get_absolute_time();
    
    while (system_active) {
        absolute_time_t current_time = get_absolute_time();
        
        // Control loop at fixed frequency
        if (absolute_time_diff_us(last_control_time, current_time) >= CONTROL_PERIOD_US) {
            last_control_time = current_time;
            
            // Capture frame
            bool frame_captured = capture_frame();
            
            if (frame_captured) {
                // Detect LEDs
                led_count = detect_leds(detected_leds);
                
                // Estimate pose
                if (led_count >= 3) {
                    estimate_pose_triangle(detected_leds, led_count, &current_pose);
                } else {
                    current_pose.valid = false;
                }
                
                // Update system state
                update_system_state();
                
                // Generate and send RC commands
                rc_command_t rc_cmd = compute_rc_commands(&current_pose);
                send_ble_command(&rc_cmd);
                
                // Update status LED
                gpio_put(STATUS_LED_PIN, current_pose.valid ? 1 : 0);
            }
            
            // Check for emergency stop
            handle_emergency_stop();
        }
        
        // Send status at lower frequency (5 Hz)
        if (absolute_time_diff_us(last_status_time, current_time) >= 200000) {
            last_status_time = current_time;
            send_ble_status();
        }
        
        sleep_us(1000); // Small delay to prevent tight loop
    }
}

// Main function
int main(void) {
    stdio_init_all();
    sleep_ms(2000); // Allow USB to settle
    
    printf("=== Drone Docking Station v1.0 ===\n");
    
    // Initialize all systems
    init_system_io();
    init_camera_i2c();
    init_camera_parallel();
    init_ble_uart();
    init_pid_controllers();
    
    // Configure camera
    if (!configure_hm0360()) {
        printf("FATAL: Camera configuration failed\n");
        system_state = SYSTEM_ERROR;
        while (1) {
            gpio_put(STATUS_LED_PIN, 1);
            sleep_ms(500);
            gpio_put(STATUS_LED_PIN, 0);
            sleep_ms(500);
        }
    }
    
    printf("All systems initialized successfully\n");
    printf("Starting vision processing on core 1...\n");
    
    // Start core 1 for image processing
    multicore_launch_core1(core1_entry);
    
    // Core 0 - Main monitoring loop
    absolute_time_t last_debug_time = get_absolute_time();
    
    while (system_active) {
        // Print debug info every 5 seconds
        if (absolute_time_diff_us(last_debug_time, get_absolute_time()) >= 5000000) {
            last_debug_time = get_absolute_time();
            
            printf("=== DEBUG INFO ===\n");
            printf("System State: %d\n", system_state);
            printf("Frame Counter: %lu\n", frame_counter);
            printf("Success Rate: %.1f%%\n", 
                   frame_counter ? (100.0f * successful_frames / frame_counter) : 0);
            printf("LED Count: %d\n", led_count);
            printf("Pose Valid: %s\n", current_pose.valid ? "YES" : "NO");
            if (current_pose.valid) {
                printf("Position: (%.3f, %.3f) Yaw: %.1f° Scale: %.2f\n",
                       current_pose.x, current_pose.y, current_pose.yaw, current_pose.scale);
            }
            printf("==================\n");

            // Debug camera signals
            debug_camera_signals();
        }
        
        sleep_ms(100);
    }
    
    printf("System shutdown\n");
    return 0;
}