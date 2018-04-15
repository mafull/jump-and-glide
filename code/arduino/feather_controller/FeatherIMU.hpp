#include <stdint.h>


#ifndef __FEATHER_IMU_HPP
#define __FEATHER_IMU_HPP

#define GRAVITY 9.80665f

class FeatherIMU 
{
  public:
    bool init();
    void calibrateGravity(); // Only call this when under no acceleration.
    void updateData();

    float roll, pitch, heading;
    float vertical_acc;

  private:
    void calibrateGyros();
    void updateRaw();
      
    int16_t AcX, AcY, AcZ;
    int16_t GyX, GyY, GyZ;
    int32_t GyX_offset, GyY_offset, GyZ_offset;

    int16_t gravity_angleX, gravity_angleY, gravity_angleZ;
};


// Preinstantiate
extern FeatherIMU IMU;


#endif  // __FEATHER_IMU_HPP

