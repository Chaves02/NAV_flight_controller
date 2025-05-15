/* Test conections and leds of TL.HEART_BLC */

#include "pico/stdlib.h"
#include "rp_agrolib_uart.h"
#include "rp_agrolib_i2c.h"

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

#define UART_ID uart0
#define UART_TX 0
#define UART_RX 1
#define UART_BAUDRATE 115200

// Buffer for incoming data
#define MAX_BUFFER_LEN 128
#define TIMEOUT_US 50000  // 50ms timeout
char buffer[MAX_BUFFER_LEN];
volatile int buffer_index = 0;
volatile uint64_t last_rx_time;

// Process complete message in buffer
void process_buffer() {
    if (buffer_index > 0) {
        buffer[buffer_index] = '\0';  // Null-terminate the string

        printf("Received: %s\n", buffer);
        
        // Reset buffer
        buffer_index = 0;
    }
}

void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        last_rx_time = time_us_64();  // Update last read time
        
        // Check for end of message
        if (c == 'n' || c == '\r') {
            process_buffer();
        }
        else if (buffer_index > 0 && (time_us_64() - last_rx_time) > TIMEOUT_US) {
            printf("Timeout on incomplete message: %.*s\n", buffer_index, buffer);
            buffer_index = 0;
        }
        // Add character to buffer if there's room
        else if (buffer_index < MAX_BUFFER_LEN - 1) {
            buffer[buffer_index++] = c;
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Allow time for USB enumeration
    printf("\nDocking Controller starting...\n");

    // Initialize GPIO for LEDs
    gpio_init(LED_RUN);
    gpio_set_dir(LED_RUN, GPIO_OUT);
    gpio_put(LED_RUN, 0);
    sleep_ms(1000);

    // Initialize GPIO for Charger and Fault
    gpio_init(CHARGER);
    gpio_set_dir(CHARGER, GPIO_OUT);
    gpio_put(CHARGER, 1);
    sleep_ms(1000);

    gpio_init(SOLAR);
    gpio_set_dir(SOLAR, GPIO_IN);

    gpio_init(FAULT);
    gpio_set_dir(FAULT, GPIO_IN);

    gpio_init(BAT_SENSE);
    gpio_set_dir(BAT_SENSE, GPIO_IN);

    // Initialize I2C
    i2c_inst_t *i2c = init_i2c0(I2C_SDA0, I2C_SCL0, I2C_BAUD, true);
    printf("I2C0 initialized\n");

    // Initialize UART
    int irq = uart_setup(UART_ID, UART_RX, UART_TX, UART_BAUDRATE, 8, 1, UART_PARITY_NONE);
    uart_enable_interrupt(UART_ID, irq, on_uart_rx);
    last_rx_time = time_us_64();
    printf("UART initialized\n");
}