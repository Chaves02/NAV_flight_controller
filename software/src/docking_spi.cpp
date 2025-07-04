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

#include <string>
#include <fstream>
#include <sstream>

// Add these global variables
std::ofstream telemetry_file;
bool logging_enabled = false;
uint32_t telemetry_counter = 0;

// Message reconstruction variables
#define MAX_MESSAGE_LEN 1024
char message_buffer[MAX_MESSAGE_LEN];
volatile int message_index = 0;
volatile bool building_message = false;

// Add this function to handle telemetry data
void handle_telemetry(const std::string& data) {
    // Parse telemetry data (format: TELEM,roll,pitch,yaw,...)
        uint32_t timestamp = time_us_32() / 1000; // Convert to milliseconds
        
        // Create CSV line with timestamp
        std::string csv_line = std::to_string(timestamp) + "," + data.substr(data.find("TELEM,") + 6);
        
        // Print to console
        printf("Telemetry: %s", csv_line.c_str());
        
        // Write to file if logging is enabled
        if (logging_enabled && telemetry_file.is_open()) {
            telemetry_file << csv_line;
            telemetry_file.flush(); // Ensure data is written immediately
            telemetry_counter++;
        }
}

// Function to clean and filter incoming data
std::string clean_message(const char* raw_data, int len) {
    std::string cleaned;
    std::string raw(raw_data, len);
    
    // Remove "+B" prefixes and "SEND\n" confirmations
    size_t pos = 0;
    while (pos < raw.length()) {
        // Skip "+B" at the beginning of fragments
        if (pos < raw.length() - 1 && raw[pos] == '+' && raw[pos + 1] == 'B') {
            pos += 2;
            continue;
        }
        
        // Skip "SEND\n" confirmations
        if (pos < raw.length() - 4 && raw.substr(pos, 4) == "SEND") {
            // Skip until we find the next newline or end of string
            while (pos < raw.length() && raw[pos] != '\n') {
                pos++;
            }
            if (pos < raw.length() && raw[pos] == '\n') {
                pos++;
            }
            continue;
        }
        
        // Keep the character if it's not part of unwanted sequences
        cleaned += raw[pos];
        pos++;
    }
    
    return cleaned;
}

// Function to reconstruct fragmented messages
void reconstruct_message(const char* data, int len) {
    std::string cleaned = clean_message(data, len);
    
    for (char c : cleaned) {
        // Check if we're starting a new telemetry message
        if (!building_message && c == 'T') {
            building_message = true;
            message_index = 0;
            message_buffer[message_index++] = c;
        }
        // If we're building a message, add characters
        else if (building_message) {
            if (message_index < MAX_MESSAGE_LEN - 1) {
                message_buffer[message_index++] = c;
            }
            
            // Check if we've reached the end of telemetry message (ends with 't')
            if (c == 't') {
                message_buffer[message_index] = '\0';
                std::string complete_message(message_buffer);
                
                // Only process if it looks like a complete telemetry message
                if (complete_message.find("TELEM,") != std::string::npos) {
                    handle_telemetry(complete_message);
                } else {
                    // If it doesn't contain TELEM, it might be a fragmented start
                    // Keep building until we get a proper message
                    printf("Incomplete message fragment: %s\n", complete_message.c_str());
                }
                
                // Reset for next message
                building_message = false;
                message_index = 0;
            }
        }
        // Handle other characters that might be standalone messages
        else if (c != '\n' && c != '\r' && c != ' ') {
            // Start a non-telemetry message
            message_buffer[0] = c;
            message_index = 1;
            
            // For now, just add this character and process immediately
            // You can extend this for other message types if needed
            message_buffer[message_index] = '\0';
            // Don't process single characters as messages
        }
    }
}

// Add this function to start/stop logging
void toggle_logging() {
    if (!logging_enabled) {
        // Start logging
        std::string filename = "telemetry_" + std::to_string(time_us_32()) + ".csv";
        telemetry_file.open(filename);
        
        if (telemetry_file.is_open()) {
            // Write CSV header
            telemetry_file << "timestamp_ms,roll,pitch,yaw,rate_roll,rate_pitch,rate_yaw,";
            telemetry_file << "error_angle_roll,error_angle_pitch,error_rate_yaw,";
            telemetry_file << "error_rate_roll,error_rate_pitch,error_rate_yaw,";
            telemetry_file << "motor1,motor2,motor3,motor4,";
            telemetry_file << "altitude,velocity,lidar_raw,armed\n";
            
            logging_enabled = true;
            telemetry_counter = 0;
            printf("Started logging to: %s\n", filename.c_str());
        } else {
            printf("Failed to open telemetry log file\n");
        }
    } else {
        // Stop logging
        if (telemetry_file.is_open()) {
            telemetry_file.close();
        }
        logging_enabled = false;
        printf("Stopped logging. Saved %d telemetry records\n", telemetry_counter);
    }
}

// Buffer for incoming data
#define MAX_BUFFER_LEN 256
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

        // Check for BLE data (telemetry from drone)
        if(ble.isConnected()) {
            while (uart_is_readable(UART_ID)) {
                char c = uart_getc(UART_ID);

                if (buffer_index < MAX_BUFFER_LEN - 1) {
                    buffer[buffer_index++] = c;
                }

                // Process complete messages
                if (/*c == '\n' || c == '\r'*/ c == 't') {
                    if (buffer_index > 0) {
                        //buffer[buffer_index] = '\0';
                        //std::string message(buffer);
                        
                        // Handle different message types
                        //if (message.find("T,") != std::string::npos) {
                            reconstruct_message(buffer, buffer_index + 1);
                        //} else {
                        //    printf("Unknown message: %s\n", message.c_str());
                        //}
                        
                        buffer_index = 0;
                    }
                }
            }
        }

        // Check USB for data from PC
        if (tud_cdc_available()) {
            char usb_buffer[256];
            int len = tud_cdc_read(usb_buffer, sizeof(usb_buffer));

            if (len > 0) {
                std::string command(usb_buffer);
                
                // Handle special commands
                if (command.find("log") != std::string::npos) {
                    toggle_logging();
                } else if (command.find("status") != std::string::npos) {
                    printf("Logging: %s, Records: %d\n", 
                           logging_enabled ? "ON" : "OFF", telemetry_counter);
                } else {
                    // Forward RC commands to drone
                    if(ble.isConnected()) {
                        ble.sendMessage(command);
                    } else {
                        printf("BLE not connected, cannot forward: %s\n", command.c_str());
                    }
                }
            }
        }
    }

    return 0;
}
