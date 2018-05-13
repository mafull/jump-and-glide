#include <stdint.h>


#ifndef __FEATHER_IMU_HPP
#define __FEATHER_IMU_HPP

#define GRAVITY 9.80665f

class FeatherIMU
{
  public:
    bool init();

    void calibrateGyros();
    void setInitialOrientation();

    void updateRawData();
    void updateAngles();
    void update();

    void getRPH(float *r, float *p, float *h);
    void getAccRPY(float *r, float *p, float *y);
    void getGyroRPY(float *r, float *p, float *y);
    float getVertAcc();


  private:
    // Calibration data
    int gx_off, gy_off, gz_off;

    // Raw data
    float ax, ay, az;
    float gx, gy, gz;
    float temperature;

    // Calculated angles
    float aRoll, aPitch, aYaw;
    float gRoll, gPitch, gYaw;
    float roll, pitch, heading;

    float vercial_acc;


    void calibrateGyros();
    void updateRaw();

    int16_t AcX, AcY, AcZ;
    int16_t GyX, GyY, GyZ;
    int32_t GyX_offset, GyY_offset, GyZ_offset;

    int16_t gravity_angleX, gravity_angleY, gravity_angleZ;
};


// Preinstantiate
extern FeatherIMU imu;


#endif  // __FEATHER_IMU_HPP
