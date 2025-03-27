#include "ahr.h"               //AHR class definition
#include "mahony/mahony.h"
#include <math.h>

//// Declare global BMI088 instance
//Bmi088* bmi088;

void Ahr::calibrateSensors(int samples)
{
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;

  printf("Calibrating BMI088. Keep sensor still...");

  // Wait a bit for sensor to stabilize
  sleep_ms(1000);

  // Collect samples
  for (int i = 0; i < samples; i++)
  {
    if(bmi088Ptr == nullptr) {
      printf("AHR: ERROR - BMI088 pointer not set");
      return;
    }
    bmi088Ptr->readSensor();

    sumAx += bmi088Ptr->getAccelX_mss();
    sumAy += bmi088Ptr->getAccelY_mss();
    sumAz += bmi088Ptr->getAccelZ_mss();

    sumGx += bmi088Ptr->getGyroX_rads();
    sumGy += bmi088Ptr->getGyroY_rads();
    sumGz += bmi088Ptr->getGyroZ_rads();

    sleep_ms(5); // Small sleep_ms between readings
  }

  // Calculate average offsets
  accel_offset_x = sumAx / samples;
  accel_offset_y = sumAy / samples;
  accel_offset_z = sumAz / samples - 1.0f; // Subtract 1g from Z-axis (assuming Z is pointing down)

  gyro_offset_x = sumGx / samples;
  gyro_offset_y = sumGy / samples;
  gyro_offset_z = sumGz / samples;

  printf("Calibration complete!\n");
  printf("Accel offsets: %.3f, %.3f, %.3f\n", accel_offset_x, accel_offset_y, accel_offset_z);
  printf("Gyro offsets: %.3f, %.3f, %.3f\n", gyro_offset_x, gyro_offset_y, gyro_offset_z);
}

void Ahr::update()
{
  // get sensor data from BMI088
  if(bmi088Ptr == nullptr) {
    printf("AHR: ERROR - BMI088 pointer not set\n");
    return;
  }
  bmi088Ptr->readSensor();
  // correct the sensor data with the internal calibration values
  // use simple first-order low-pass filter to get rid of high frequency noise
  // store filtered data in ax ay az gx gy gz
  // call the sensor fusion algorithm to update q
  // compute euler angles from q

  // Low-pass filtered, corrected accelerometer data
  ax += B_acc * ((bmi088Ptr->getAccelX_mss() - accel_offset_x) - ax);
  ay += B_acc * ((bmi088Ptr->getAccelY_mss() - accel_offset_y) - ay);
  az += B_acc * ((bmi088Ptr->getAccelZ_mss() - accel_offset_z) - az);

  // Low-pass filtered, corrected gyro data
  gx += B_gyr * ((bmi088Ptr->getGyroX_rads() - gyro_offset_x) - gx);
  gy += B_gyr * ((bmi088Ptr->getGyroY_rads() - gyro_offset_y) - gy);
  gz += B_gyr * ((bmi088Ptr->getGyroZ_rads() - gyro_offset_z) - gz);

  // No magnetometer, so we don't update mx, my, mz
  mx = 0;
  my = 0;
  mz = 0;

  //printf("ax:%.2f ay:%.2f az:%.2f gx:%.2f gy:%.2f gz:%.2f\n", ax, ay, az, gx, gy, gz);

  // call fusionUpdate to update q
  fusionUpdate();

  // update euler angles
  computeAngles();

  dt = bmi088Ptr->getTime_ps() / 1e12f;
}

void Ahr::setFromMag(float *q)
{
  // Since we don't have a magnetometer, we initialize with zero yaw
  yaw = 0;
  pitch = 0;
  roll = 0;
  q[0] = 1;
  q[1] = 0;
  q[2] = 0;
  q[3] = 0;
  printf("AHR: No Magnetometer, yaw:0.00");
}

// compute euler angles from q
void Ahr::computeAngles()
{
  roll = atan2(q[0] * q[1] + q[2] * q[3], 0.5f - q[1] * q[1] - q[2] * q[2]) * rad_to_deg; // degrees - roll right is positive
  pitch = asin(std::clamp(-2.0f * (q[1] * q[3] - q[0] * q[2]), -1.0f, 1.0f)) * rad_to_deg;// degrees - pitch up is positive - use constrain() to prevent NaN due to rounding
  yaw = atan2(q[1] * q[2] + q[0] * q[3], 0.5f - q[2] * q[2] - q[3] * q[3]) * rad_to_deg; // degrees - yaw right is positive

  printf("AHR: roll:%.2f pitch:%.2f yaw:%.2f\n", roll, pitch, yaw);
}

// get acceleration in earth-frame up direction in [m/s^2]
float Ahr::getAccelUp()
{
  return 9.80665 * ((2 * q[1] * q[3] - 2 * q[0] * q[2]) * ax + (2 * q[2] * q[3] + 2 * q[0] * q[1]) * ay + (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * az - 1.0);
}

// Implement AhrMahony methods
#if AHR_USE == AHR_USE_MAHONY || AHR_USE == AHR_USE_MAHONY_BF || !defined AHR_USE
void AhrMahony::setup(float gyrLpFreq, float accLpFreq, float magLpFreq) {
    #if AHR_USE == AHR_USE_MAHONY_BF
        printf("AHR: MAHONY_BF (NO MAG)");
    #else
        printf("AHR: MAHONY (NO MAG)");
    #endif

    if (!bmi088Ptr) {
        printf("AHR: ERROR - BMI088 pointer not set");
        return;
    }

    float odr = bmi088Ptr->ODR_2000HZ;

    B_gyr = std::clamp(static_cast<float>(1 - exp(-2 * M_PI * gyrLpFreq / odr)), 0.0f, 1.0f);
    B_acc = std::clamp(static_cast<float>(1 - exp(-2 * M_PI * accLpFreq / odr)), 0.0f, 1.0f);
    B_mag = 0; // No magnetometer

    // Initialize with zeros until calibrated
    accel_offset_x = 0;
    accel_offset_y = 0;
    accel_offset_z = 0;
    gyro_offset_x = 0;
    gyro_offset_y = 0;
    gyro_offset_z = 0;
}

void AhrMahony::setInitalOrientation() {
    float qnew[4];
    setFromMag(qnew);
    q0 = qnew[0];
    q1 = qnew[1];
    q2 = qnew[2];
    q3 = qnew[3];
}

void AhrMahony::fusionUpdate() {
    
    // Call Mahony with zero values for magnetometer
    //convert acc to g
    ahrs_Mahony(gx, gy, gz, ax * mss_to_g, ay * mss_to_g, az * mss_to_g, 0, 0, 0, bmi088Ptr->getTime_ps() / 1e12f);

    //printf("gettime: %f\n", bmi088Ptr->getTime_ps() / 1e12f);
    
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}
#endif

/*=================================================================================================
// Mahony or undefined
//=================================================================================================
#if AHR_USE == AHR_USE_MAHONY || AHR_USE == AHR_USE_MAHONY_BF || !defined AHR_USE

class AhrMahony : public Ahr
{
public:
  void setup(float gyrLpFreq, float accLpFreq, float magLpFreq)
  {
#if AHR_USE == AHR_USE_MAHONY_BF
    printf("AHR: MAHONY_BF (NO MAG)");
#else
    printf("AHR: MAHONY (NO MAG)");
#endif

    if (!bmi088Ptr) {
      // Handle error - Bmi088 pointer not set
      printf("AHR: ERROR - BMI088 pointer not set");
      return;
    }

    float odr = bmi088Ptr->ODR_2000HZ; ////////////////////////////////////////////////

    B_gyr = std::clamp(static_cast<float>(1 - exp(-2 * M_PI * gyrLpFreq / odr)), 0.0f, 1.0f);
    B_acc = std::clamp(static_cast<float>(1 - exp(-2 * M_PI * accLpFreq / odr)), 0.0f, 1.0f);
    B_mag = 0; // No magnetometer

    // Initialize with zeros until calibrated
    accel_offset_x = 0;
    accel_offset_y = 0;
    accel_offset_z = 0;
    gyro_offset_x = 0;
    gyro_offset_y = 0;
    gyro_offset_z = 0;
  }

  // estimate initial orientation (without magnetometer)
  void setInitalOrientation()
  {
    float qnew[4];
    setFromMag(qnew); // This will initialize to default values now
    q0 = qnew[0];
    q1 = qnew[1];
    q2 = qnew[2];
    q3 = qnew[3];
  }

protected:
  void fusionUpdate()
  {
    if (!bmi088Ptr) {
      // Handle error - Bmi088 pointer not set
      printf("AHR: ERROR - BMI088 pointer not set");
      return;
    }
    // Call Mahony with zero values for magnetometer
    ahrs_Mahony(gx * deg_to_rad, gy * deg_to_rad, gz * deg_to_rad, ax, ay, az, 0, 0, 0, bmi088Ptr->getTime_ps());
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
  }
};

//AhrMahony ahr_instance;
//Ahr& ahr = ahr_instance; // Global reference pointing to the concrete instance
#endif*/