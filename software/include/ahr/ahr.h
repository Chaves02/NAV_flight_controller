#pragma once
#include <stdint.h>
#include "rp_agrolib_bmi088.h"
#include <algorithm>

// Forward declarations of constants or enums if needed
#define AHR_USE_MAHONY 1 //default when AHR_USE is not defined
#define AHR_USE_MAHONY_BF 2 //betaflight flavored: only use accel if near 1G

class Ahr {
protected:
    Bmi088* bmi088Ptr = nullptr;

public:
    Ahr(Bmi088* bmi088 = nullptr) : bmi088Ptr(bmi088) {}
    
    // Shared member variables (based on your implementation)
    float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
    float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
    float B_gyr = 0, B_acc = 0, B_mag = 0;
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float mx = 0, my = 0, mz = 0;
    float q[4] = {1, 0, 0, 0};
    //float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
    float roll = 0, pitch = 0, yaw = 0;
    float dt = 0;
    float rad_to_deg = 180.0f / M_PI;
    float deg_to_rad = M_PI / 180.0f;
    float mss_to_g = 1.0f / 9.80665f;

    virtual void calibrateSensors(int samples);
    virtual void update();
    virtual void setFromMag(float *q);
    virtual void computeAngles();
    float getAccelUp();

protected:
    virtual void fusionUpdate() = 0; // Pure virtual method
};

#if AHR_USE == AHR_USE_MAHONY || AHR_USE == AHR_USE_MAHONY_BF || !defined AHR_USE
class AhrMahony : public Ahr {
public:
    AhrMahony(Bmi088* bmi088 = nullptr) : Ahr(bmi088) {}

    void setup(float gyrLpFreq, float accLpFreq, float magLpFreq=0);
    void setInitalOrientation();

protected:
    void fusionUpdate() override;
};
#endif