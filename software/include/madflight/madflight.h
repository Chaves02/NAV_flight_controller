#define MADFLIGHT_VERSION "Chaves_dev"

// #pragma once //don't use here, we want to get an error if included twice

#include <pico/stdlib.h>
#include <stdio.h>
#include <string>
#include "led/led.h"
#include "ahr/ahr.h"
#include "out/out.h"
#include "pid/pid.h"
#include "rclsim/rclsim.h"
#include "rp_agrolib_i2c.h"
#include "rp_agrolib_bmi088.h"
#include "rp_agrolib_vl53l8cx_api.h"

#define I2C_SDA1 18     // I2C1 SDA on GPIO18
#define I2C_SCL1 19     // I2C1 SCL on GPIO19
#define I2C_BAUD 400000 // 400kHz I2C speed

#define BMI088_ACCL_ADDR 0x18
#define BMI088_GYRO_ADDR 0x68
#define LIDAR_ADDR 0x29

// Helpers
void madflight_die(std::string msg);
void madflight_warn(std::string msg);
void madflight_warn_or_die(std::string msg, bool die);

extern AhrMahony ahr;

//===============================================================================================
// madflight_setup()
//===============================================================================================

void madflight_setup();