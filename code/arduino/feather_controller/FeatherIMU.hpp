#ifndef __FEATHER_IMU_HPP
#define __FEATHER_IMU_HPP


class FeatherIMU {
  public:
    void init();
    void calibrate();
    void update();

    float roll, pitch, heading;

  private:
    
};


// Preinstantiate
extern FeatherIMU IMU;


#endif  // __FEATHER_IMU_HPP

