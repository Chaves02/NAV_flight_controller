#include "madflight.h"

// Define the AhrMahony instance
// Initialize with nullptr, will be set up properly in madflight_setup()
AhrMahony ahr(nullptr);

//===============================================================================================
// madflight_setup()
//===============================================================================================

// Initialize I2C
i2c_inst_t *i2c = init_i2c1(I2C_SDA1, I2C_SCL1, I2C_BAUD, true);

// Initialize sensors
Bmi088 bmi088(i2c, BMI088_ACCL_ADDR, BMI088_GYRO_ADDR);

void madflight_setup()
{
    stdio_init_all(); // start console serial

    // 5 second startup sleep_ms
    for (int i = 10; i > 0; i--)
    {
        printf(MADFLIGHT_VERSION " starting %d ...\n", i);
        sleep_ms(500);
    }

    // droneLeds - Setup droneLeds
    droneLeds.setup();
    droneLeds.allOn();         // turn on to signal startup
    droneLeds.enabled = false; // do not change state until setup completed

    
    VL53L8CX_Configuration Dev;       // ToF sensor configuration
    VL53L8CX_ResultsData Results;     // ToF results data
    uint8_t status, isReady, isAlive; // For ranging status

    ahr = AhrMahony(&bmi088); // Attitude and Heading Reference System /////////////////////////////

    // Initialize sensors
    int stat;
    stat = bmi088.begin();
    if (stat < 0)
    {
        printf("BMI_Error: %d\n", stat);
        while (1)
        {
        }
    }

    // Initialize VL53L8CX sensor
    Dev.platform.address = LIDAR_ADDR; /* Default address */
    Dev.platform.i2c_instance = i2c;   /* I2C instance */

    // Init VL53L8CX sensor
    status = vl53l8cx_init(&Dev);
    if (status)
    {
        printf("VL53L8CX ULD Loading failed\n");
        while (1)
        {
        }
    }

    // AHR - setup low pass filters for AHRS filters
    ahr.setup(60, 70);

    // AHR - calibrate sensors
    ahr.calibrateSensors(500);

    // CLI - Command Line Interface
    // cli.begin();

    // Enable droneLeds, and switch it off signal end of startup.
    droneLeds.enabled = true;
    droneLeds.allOff();
}

//===============================================================================================
// HELPERS
//===============================================================================================

void madflight_die(std::string msg)
{
    bool do_print = true;
    droneLeds.enabled = true;
    for (;;)
    {
        if (do_print)
            printf("FATAL ERROR: %s . Use CLI or reboot.\n", msg);
        for (int i = 0; i < 20; i++)
        {
            droneLeds.toggleAll();
            // uint32_t ts = millis();
            // while (millis() - ts < 50)
            //{
            //   if (cli.update())
            //     do_print = false; // process CLI commands, stop error output after first command
            //   rcl.update();       // keep rcl (mavlink?) running
            // }
        }
    }
}
void madflight_warn(std::string msg)
{
    printf("WARNING: %s \n", msg);
    // flash LED for 1 second
    for (int i = 0; i < 20; i++)
    {
        droneLeds.toggleAll();
        sleep_ms(50);
    }
}

void madflight_warn_or_die(std::string msg, bool die)
{
    if (die)
    {
        madflight_die(msg);
    }
    else
    {
        madflight_warn(msg);
    }
}