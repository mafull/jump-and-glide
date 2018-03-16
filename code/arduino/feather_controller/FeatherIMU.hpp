#ifndef __FEATHER_IMU_HPP
#define __FEATHER_IMU_HPP


class FeatherIMU {
  public:
    void init();


    float roll, pitch, yaw;

  private:
    
};


// Preinstantiate
extern FeatherIMU IMU;


#endif  // __FEATHER_IMU_HPP

