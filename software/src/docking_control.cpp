#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tusb.h" // TinyUSB for USB CDC (PC ↔ RP2040)
#include "rp_agrolib_bc832.h"

#define LED_RUN 11
#define CHARGER 12 // activate drone charger and led also indicate

// Monitor faults
#define SOLAR 21     // CHRG
#define FAULT 20     // FAULT
#define BAT_SENSE 13 // BAT_SENSE

// I2C
#define I2C_SDA0 16
#define I2C_SCL0 17

#define I2C_SDA1 14
#define I2C_SCL1 15
#define I2C_BAUD 400000 // 400kHz I2C speed

// UART and BLE settings
#define UART_ID uart0
#define UART_TX 0
#define UART_RX 1
#define MODE_PIN 10
#define UART_BAUDRATE 115200

bool logging_enabled = false;
uint32_t telemetry_counter = 0;
bool csv_header_sent = false;

bool waiting_for_send_confirmation = false;
uint32_t send_timeout_start = 0;
const uint32_t SEND_TIMEOUT_MS = 100; // 100 ms timeout

// Message reconstruction variables
#define MAX_MESSAGE_LEN 1024
char message_buffer[MAX_MESSAGE_LEN];
volatile int message_index = 0;
volatile bool building_message = false;

// Function to send CSV header over USB
void send_csv_header() {
    const char* header = "CSV_HEADER: timestamp_ms,roll,pitch,rate_roll,rate_pitch,rate_yaw,"
                        "error_angle_roll,error_angle_pitch,"
                        "error_rate_roll,error_rate_pitch,error_rate_yaw,"
                        "motor1,motor2,motor3,motor4,"
                        "altitude,velocity,lidar_raw,armed\n";
    
    // Send over USB CDC
    tud_cdc_write(header, strlen(header));
    tud_cdc_write_flush();
    
    printf("CSV header sent over USB\n");
    csv_header_sent = true;
}

// REPLACE the handle_telemetry function:
void handle_telemetry(const char* data) {
    uint32_t timestamp = time_us_32() / 1000; // Convert to milliseconds
    
    // Create CSV line with timestamp
    char csv_line[512];
    // Extract telemetry data after "TELEM,"
    const char* telem_data = strstr(data, "TELEM,");
    if (telem_data) {
        telem_data += 6; // Skip "TELEM,"
        snprintf(csv_line, sizeof(csv_line), "%lu,%s", timestamp, telem_data);
    } else {
        snprintf(csv_line, sizeof(csv_line), "%lu,%s", timestamp, data);
    }
    
    // Always print to console for debugging
    printf("TELEM: %s\n", csv_line);
    
    // If logging is enabled, send over USB CDC with special prefix
    if (logging_enabled) {
        if (!csv_header_sent) {
            send_csv_header();
        }
        
        // Send CSV data over USB with special prefix for PC to capture
        char usb_message[600];
        snprintf(usb_message, sizeof(usb_message), "CSV_DATA: %s\n", csv_line);
        
        tud_cdc_write(usb_message, strlen(usb_message));
        tud_cdc_write_flush();
        
        telemetry_counter++;
        
        // Print progress every 50 records
        if (telemetry_counter % 50 == 0) {
            printf("Logged %lu telemetry records\n", telemetry_counter);
        }
    }
}

// REPLACE the toggle_logging function:
void toggle_logging() {
    if (!logging_enabled) {
        // Start logging
        logging_enabled = true;
        telemetry_counter = 0;
        csv_header_sent = false;
        
        printf("Started CSV logging over USB\n");
        
        // Send start marker
        char start_msg[100];
        snprintf(start_msg, sizeof(start_msg), "CSV_START: telemetry_%lu.csv\n", time_us_32());
        tud_cdc_write(start_msg, strlen(start_msg));
        tud_cdc_write_flush();
        
    } else {
        // Stop logging
        logging_enabled = false;
        
        printf("Stopped CSV logging. Saved %lu telemetry records\n", telemetry_counter);
        
        // Send stop marker
        char stop_msg[100];
        snprintf(stop_msg, sizeof(stop_msg), "CSV_STOP: %lu records saved\n", telemetry_counter);
        tud_cdc_write(stop_msg, strlen(stop_msg));
        tud_cdc_write_flush();
    }
}

// REPLACE the clean_message function (remove std::string usage):
void clean_message(const char *raw_data, int len, char *cleaned, int max_len) {
    int cleaned_len = 0;
    
    for (int i = 0; i < len && cleaned_len < max_len - 1; i++) {
        char c = raw_data[i];
        
        // Skip "+B" prefixes
        if (i < len - 1 && c == '+' && raw_data[i+1] == 'B') {
            i += 2; // Skip "+B"
            continue;
        }
        
        // Skip "SEND\r\n" confirmations
        if (i < len - 4 && strncmp(&raw_data[i], "SEND", 4) == 0) {
            // Skip until we find the next newline or end of string
            while (i < len && raw_data[i] != '\n') {
                i++;
            }
            continue;
        }
        
        // Keep only valid characters
        if (isalnum(c) || c == ',' || c == '.' || c == '-') {
            cleaned[cleaned_len++] = c;
        }
    }
    cleaned[cleaned_len] = '\0';
}

// REPLACE the reconstruct_message function:
void reconstruct_message(const char *data, int len) {
    static char cleaned[MAX_MESSAGE_LEN];
    clean_message(data, len, cleaned, MAX_MESSAGE_LEN);
    
    int cleaned_len = strlen(cleaned);
    
    // Process cleaned message character by character
    for (int i = 0; i < cleaned_len; i++) {
        char c = cleaned[i];
        
        // Check if we're starting a new telemetry message
        if (!building_message) {
            if (c == 'T' && message_index == 0) {
                building_message = true;
                message_index = 0;
                message_buffer[message_index++] = c;
            }
        } else {
            if (message_index < MAX_MESSAGE_LEN - 1) {
                message_buffer[message_index++] = c;
            }
            
            // Check if we've reached the end of telemetry message (ends with 't')
            if (c == 't') {
                message_buffer[message_index] = '\0';
                
                // Validate that this is actually a complete telemetry message
                if (strncmp(message_buffer, "TELEM,", 6) == 0) {
                    handle_telemetry(message_buffer);
                } else {
                    printf("Invalid message format: %s\n", message_buffer);
                }
                
                // Reset for next message
                building_message = false;
                message_index = 0;
            }
            // If we encounter another 'T' while building, start over
            else if (c == 'T' && message_index > 5) {
                building_message = true;
                message_index = 0;
                message_buffer[message_index++] = c;
            }
        }
    }
}

// Function to wait for SEND confirmation (keep as is)
bool wait_for_send_confirmation() {
    waiting_for_send_confirmation = true;
    send_timeout_start = time_us_32() / 1000; // Convert to milliseconds

    static char send_buffer[128];
    static int send_index = 0;
    send_index = 0; // Reset buffer at start

    while (waiting_for_send_confirmation) {
        // Check for timeout
        uint32_t current_time = time_us_32() / 1000;
        if (current_time - send_timeout_start > SEND_TIMEOUT_MS) {
            waiting_for_send_confirmation = false;
            return false; // Timeout occurred
        }

        // Check for incoming BLE data
        if (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);

            // Add character to buffer if there's space
            if (send_index < sizeof(send_buffer) - 1) {
                send_buffer[send_index++] = c;
                send_buffer[send_index] = '\0'; // Null terminate
            }

            // Check if we have enough characters to check for "SEND\r\n"
            if (c == '\n') {
                // Check if we received SEND\r\n confirmation
                if (strstr(send_buffer, "SEND\r\n") != NULL) {
                    waiting_for_send_confirmation = false;
                    return true; // Success
                }
            }

            // Reset buffer if it gets too full (prevent overflow)
            if (send_index >= sizeof(send_buffer) - 1) {
                send_index = 0;
            }
        }

        // Small delay to prevent busy waiting
        sleep_ms(1);
    }

    return false; // Should not reach here normally
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
    printf("CSV logging available over USB CDC\n");

    // Init BLE module
    if (!ble.begin()) {
        printf("BLE module failed to initialize\n");
        while (1)
            tight_loop_contents();
    }

    ble.reset();
    ble.setName("DockingStation");
    ble.setRFPower(0);

    if (!ble.setupAutoconnect(10000)) {
        printf("BLE autoconnect setup failed\n");
        while (1)
            tight_loop_contents();
    }

    printf("BLE module initialized and ready\n");

    // Main loop
    while (true) {
        tud_task(); // USB CDC background tasks

        if (!ble.isConnected()) {
            while (uart_is_readable(UART_ID)) {
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
        if (ble.isConnected()) {
            while (uart_is_readable(UART_ID)) {
                char c = uart_getc(UART_ID);

                if (buffer_index < MAX_BUFFER_LEN - 1) {
                    buffer[buffer_index++] = c;
                }

                // Process complete messages
                if (c == 't') {
                    if (buffer_index > 0) {
                        reconstruct_message(buffer, buffer_index + 1);
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
                usb_buffer[len] = '\0'; // Null terminate
                
                // Handle special commands
                if (strstr(usb_buffer, "log") != NULL) {
                    toggle_logging();
                }
                else if (strstr(usb_buffer, "status") != NULL) {
                    printf("Logging: %s, Records: %lu\n",
                           logging_enabled ? "ON" : "OFF", telemetry_counter);
                }
                else {
                    // Forward RC commands to drone
                    if (ble.isConnected()) {
                        ble.sendMessage(usb_buffer);
                        // Wait for SEND confirmation before continuing
                        if (!wait_for_send_confirmation()) {
                            printf("Failed to receive SEND confirmation\n");
                        }
                    }
                    else {
                        printf("BLE not connected, cannot forward: %s\n", usb_buffer);
                    }
                }
            }
        }
    }

    return 0;
}