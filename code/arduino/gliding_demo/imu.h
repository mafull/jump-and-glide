#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

class IMU
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
};

extern IMU imu;

#endif

